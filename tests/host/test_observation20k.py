"""完整帧、错误拒绝、配置/收包时序与停止确认；不打开串口。"""
import csv
import io
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
import zlib

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'tools'))
import verify_observation20k as trial
import verify_passive_observation as protocol


def frame(sequence=1, tick=0xFFFFFC18, **changes):
    values = dict(sid=7, sequence=sequence, tick=tick, rate=20000, period=1, count=5,
                  channels=4, flags=0, drops=0, ids=trial.CHANNELS + (0,) * 4)
    values.update(changes)
    v = values
    packet = struct.pack('<4sHHIIIHHBBHI8H40f', b'AXSF', 1, 212, v['sid'], v['sequence'], v['tick'],
                         v['rate'], v['period'], v['count'], v['channels'], v['flags'], v['drops'],
                         *v['ids'], *([0.25, -0.5, 0.25, 24.0] * 5 + [0.0] * 20))
    return protocol.crc_frame(packet)


def alter(packet, offset, fmt, value):
    data = bytearray(packet[:-4])
    struct.pack_into(fmt, data, offset, value)
    return protocol.crc_frame(data)


class RecordingTests(unittest.TestCase):
    def test_real_c_one_second_and_csv(self):
        path = Path(os.environ.get('AXDR_20K_FIXTURE', str(REPO / 'artifacts/expert-20k-20261028/offline-verified/stream-20k.axsf.bin')))
        self.assertTrue(path.is_file(), '先运行 check_observation20k_offline.ps1 生成真实 C 帧')
        data = path.read_bytes()
        self.assertEqual(len(data), 848000)
        parser = protocol.FrameParser()
        recording = trial.Recording()
        recording.bind(1)
        # 非帧对齐碎片输入，包含 4 包合发的跨边界。
        for offset in range(0, len(data), 997):
            for packet in parser.feed(data[offset:offset + 997]):
                recording.add(packet)
        self.assertTrue(recording.complete)
        self.assertEqual(recording.samples, 20000)
        self.assertEqual(len(recording.values), 320000)
        self.assertEqual(len(parser.buffer), 0)
        for i, row in enumerate(struct.iter_unpack('<4f', recording.values)):
            self.assertEqual(row, (i + .125, -i - .375, .25, 24.0))
        self.assertEqual(recording.metadata()['last_control_tick'], 18999)
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            recording.save(out, True)
            with (out / 'recording.csv').open(encoding='utf-8') as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 20000)
            self.assertEqual(rows[1000]['control_tick'], '0')
            self.assertEqual(rows[-1]['relative_time_s'], '0.99995000')
            self.assertEqual((out / 'recording.float32.bin').read_bytes(), recording.values)

    def test_bad_fields_never_complete(self):
        for changes in ({'sid': 0}, {'sequence': 2}, {'rate': 1000}, {'period': 20}, {'count': 4},
                        {'channels': 3}, {'flags': 1}, {'flags': 2}, {'drops': 1},
                        {'ids': (1, 2, 3, 5, 0, 0, 0, 0)}):
            with self.subTest(changes=changes):
                rec = trial.Recording()
                with self.assertRaises(protocol.VerificationError): rec.add(frame(**changes))
                self.assertFalse(rec.complete)
                self.assertEqual(rec.samples, 0)

    def test_gap_duplicate_tick_and_stream(self):
        for packet in (frame(), frame(3, 10), frame(2, 0), frame(2, 0xFFFFFC1D, sid=8)):
            with self.subTest(packet=packet[:24]):
                rec = trial.Recording()
                rec.add(frame())
                with self.assertRaises(protocol.VerificationError): rec.add(packet)
                self.assertEqual(rec.samples, 5)
                with self.assertRaises(protocol.VerificationError): rec.add(frame(2, 0xFFFFFC1D))

    def test_crc_nan_and_header(self):
        crc_bad = bytearray(frame())
        crc_bad[48] ^= 1
        for packet in (bytes(crc_bad), alter(frame(), 48, '<I', 0x7FC00000),
                       alter(frame(), 48, '<I', 0x7F800000), frame()[:-1], alter(frame(), 4, '<H', 2)):
            with self.subTest(packet=packet[:8]):
                with self.assertRaises(protocol.VerificationError): trial.Recording().add(packet)

    def test_early_ack_binding_and_wrong_ack(self):
        rec = trial.Recording()
        rec.add(frame())
        rec.bind(7)
        with self.assertRaises(protocol.VerificationError): rec.bind(8)

    def test_tail_and_failure_export(self):
        rec = trial.Recording()
        for i in range(4001): rec.add(frame(i + 1, i * 5))
        self.assertEqual(rec.samples, 20000)
        self.assertEqual(rec.metadata()['tail_packets_checked'], 1)
        with self.assertRaises(protocol.VerificationError): rec.add(frame(4003, 20010))
        self.assertFalse(rec.complete)
        with tempfile.TemporaryDirectory() as directory:
            rec.save(Path(directory), False)
            self.assertTrue((Path(directory) / 'incomplete.csv').exists())
            self.assertFalse((Path(directory) / 'recording.csv').exists())

    def test_parser_corruption_does_not_hide_gap(self):
        parser = protocol.FrameParser()
        bad = bytearray(frame(2, 5)); bad[50] ^= 1
        packets = parser.feed(frame(1, 0) + bad + frame(3, 10))
        rec = trial.Recording()
        rec.add(packets[0])
        self.assertEqual(parser.stats['bad_crc'], 1)
        with self.assertRaises(protocol.VerificationError): rec.add(packets[1])


class FakeClient:
    def __init__(self, missing=False, early=False, wrong_sid=False, stuck=False):
        self.now = 0.0
        self.stream_handler = None
        self.next_seq = 1
        self.commands = []
        self.missing, self.early, self.wrong_sid, self.stuck = missing, early, wrong_sid, stuck
        self.observation_permitted = True

    def clock(self): return self.now

    def pump(self, cleanup=False):
        self.now += .001
        if not self.missing and self.next_seq <= 4000:
            for _ in range(4):
                if self.next_seq > 4000: break
                self.stream_handler(frame(self.next_seq, (self.next_seq - 1) * 5))
                self.next_seq += 1

    def command(self, opcode, payload=b'', cleanup=False):
        self.commands.append(opcode)
        if opcode == 0x30:
            if payload[0] and self.early: self.pump()
            return struct.pack('<IHBB8HI', 8 if self.wrong_sid else 7, 20000, 4, 5, *trial.CHANNELS, 0, 0, 0, 0, 0)
        if opcode == 0x35:
            values = [20000, int(self.stuck), 20000, 0, 0, 0, 0, 100, 200, 0, 160000000, 1000, 0, 0, 2, 20000, 0, 8000, 0]
            return struct.pack('<HH19I', 1, 80, *values)
        if opcode == 3: return bytes(36)
        if opcode == 8:
            value = bytearray(40); value[38] = 1; return bytes(value)
        if opcode == 12: return bytes(40)
        raise AssertionError(opcode)


class LifecycleTests(unittest.TestCase):
    def test_data_failure_does_not_relabel_confirmed_stop(self):
        client = FakeClient(missing=True)
        report = {}
        with patch.object(protocol, 'diagnostic_delta', side_effect=protocol.VerificationError('dropped_samples=45')):
            trial.finish_observation(client, {}, report)
        self.assertTrue(report['stop_confirmed'])
        self.assertEqual(report['after']['stream_id'], 0)
        self.assertEqual(report['diagnostic_error'], 'dropped_samples=45')
        self.assertNotIn('cleanup_error', report)
        report = {}
        trial.finish_observation(FakeClient(missing=True, stuck=True), {}, report)
        self.assertFalse(report['stop_confirmed'])

    def test_phase_outputs_and_moe_are_distinct(self):
        state = {'tim1_ccer': 0x1000, 'tim1_bdtr': 0x0200A050, 'faults': 0}
        self.assertTrue(trial.check_output_registers(state)['moe_enabled'])
        for bit in (1, 4, 16, 64, 256, 1024):
            with self.assertRaises(protocol.VerificationError):
                trial.check_output_registers(dict(state, tim1_ccer=0x1000 | bit))
        with self.assertRaises(protocol.VerificationError): trial.check_output_registers({})
        with self.assertRaises(protocol.VerificationError): trial.check_output_registers(dict(state, faults=1))

    def test_runtime_extension_preserves_fault_gate(self):
        class Extended(FakeClient):
            size = 44
            first_fault = 0
            def command(self, opcode, payload=b'', cleanup=False):
                if opcode == 3:
                    value = bytearray(self.size)
                    if self.size == 44: struct.pack_into('<I', value, 36, self.first_fault)
                    return bytes(value)
                return super().command(opcode, payload, cleanup)
        client = Extended()
        self.assertEqual(protocol.stopped_state(client)['runtime_payload_bytes'], 44)
        client.first_fault = 1
        with self.assertRaises(protocol.VerificationError): protocol.stopped_state(client)
        client.first_fault = 0
        client.size = 40
        with self.assertRaises(protocol.VerificationError): protocol.stopped_state(client)

    def test_collect_early_frames_and_stop(self):
        client = FakeClient(early=True)
        rec = trial.Recording()
        trial.collect(client, rec)
        self.assertTrue(rec.complete)
        trial.stop_stream(client)
        self.assertTrue(set(client.commands) <= {0x30, 0x35, 3, 8, 12})

    def test_timeout_does_not_pass(self):
        client = FakeClient(missing=True)
        rec = trial.Recording()
        with self.assertRaises(protocol.VerificationError): trial.collect(client, rec)
        self.assertFalse(rec.complete)

    def test_wrong_ack_and_unconfirmed_stop(self):
        client = FakeClient(early=True, wrong_sid=True)
        with self.assertRaises(protocol.VerificationError): trial.collect(client, trial.Recording())
        client = FakeClient(missing=True, stuck=True)
        with self.assertRaises(protocol.VerificationError): trial.stop_stream(client)

    def test_old_identity_rejected_before_configure(self):
        class OldClient:
            observation_permitted = False
            def command(self, opcode):
                if opcode == 1:
                    value = bytearray(24); struct.pack_into('<I', value, 12, 1 << 25); return bytes(value)
                if opcode == 2:
                    value = bytearray(40); struct.pack_into('<3H', value, 8, 0, 7, 7)
                    struct.pack_into('<I', value, 20, 0x20261013)
                    value[24:35] = b'AxDrService'; return bytes(value)
                raise AssertionError('旧身份不应继续访问状态或配置')
        client = OldClient()
        with self.assertRaises(protocol.VerificationError): trial.identity_gate(client)
        self.assertFalse(client.observation_permitted)



def block(sequence=1, tick=0, **changes):
    v = dict(sid=7, sequence=sequence, tick=tick, rate=20000, period=1, count=32,
             channels=4, flags=0, drops=0, ids=trial.CHANNELS + (0,) * 4)
    v.update(changes)
    return protocol.crc_frame(struct.pack('<4sHHIIIHHBBHI8H128f', b'AXSB', 1, 564,
        v['sid'], v['sequence'], v['tick'], v['rate'], v['period'], v['count'],
        v['channels'], v['flags'], v['drops'], *v['ids'], *([.25, -.5, .25, 24.] * 32)))


class CompactTests(unittest.TestCase):
    def test_real_c_blocks_values_ticks_csv(self):
        path = Path(os.environ.get('AXDR_BLOCK_FIXTURE', str(REPO / 'artifacts/expert-20k-compact-20261028/offline/stream-compact.axsb.bin')))
        data = path.read_bytes()
        self.assertEqual(len(data), 352500)
        parser = protocol.FrameParser(); rec = trial.Recording(compact=True); rec.bind(1)
        for offset in range(0, len(data), 997):
            for packet in parser.feed(data[offset:offset+997]): rec.add(packet)
        self.assertTrue(rec.complete)
        self.assertEqual(rec.received_packets, 625)
        self.assertEqual(rec.metadata()['last_control_tick'], 18999)
        self.assertEqual(rec.metadata()['first_to_last_span_s'], .99995)
        self.assertFalse(parser.buffer)
        self.assertEqual(parser.stats, {'AXSB': 625})
        for i, row in enumerate(struct.iter_unpack('<4f', rec.values)):
            self.assertEqual(row, (i+.125, -i-.375, .25, 24.))
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory); rec.save(out, True)
            self.assertEqual((out/'recording.axsb.bin').read_bytes(), data)
            with (out/'recording.csv').open(encoding='utf-8') as file: rows=list(csv.DictReader(file))
            self.assertEqual(len(rows), 20000)
            self.assertEqual(rows[1000]['control_tick'], '0')
            self.assertEqual(rows[-1]['relative_time_s'], '.99995000'.rjust(10, '0'))

    def test_bad_block_fields_and_tail_rejected(self):
        for changes in ({'sid':0}, {'sequence':2}, {'rate':1000}, {'period':2}, {'count':31},
                        {'channels':3}, {'flags':1}, {'flags':2}, {'drops':32},
                        {'ids':(1,2,3,8,1,0,0,0)}):
            rec=trial.Recording(True)
            with self.subTest(changes=changes), self.assertRaises(protocol.VerificationError): rec.add(block(**changes))
            self.assertFalse(rec.complete)
        for bad in (frame(), block()[:-1], alter(block(),4,'<H',2), alter(block(),48,'<I',0x7fc00000),
                    alter(block(),48,'<I',0x7f800000)):
            with self.assertRaises(protocol.VerificationError): trial.Recording(True).add(bad)
        rec=trial.Recording(True)
        for i in range(626): rec.add(block(i+1, i*32))
        self.assertEqual(rec.samples,20000); self.assertEqual(rec.metadata()['tail_packets_checked'],1)
        with self.assertRaises(protocol.VerificationError): rec.add(block(627,20032,drops=32))
        self.assertFalse(rec.complete)
        with self.assertRaises(protocol.VerificationError): rec.add(block(627,20032))

    def test_corruption_tick_and_session(self):
        bad=bytearray(block(2,32)); bad[50]^=1
        parser=protocol.FrameParser(); packets=parser.feed(block()+bad+block(3,64))
        self.assertEqual(parser.stats['bad_crc'],1)
        for packet in (packets[1], block(2,31),block(2,32,sid=8),block()):
            rec=trial.Recording(True); rec.add(block())
            with self.assertRaises(protocol.VerificationError): rec.add(packet)
        rec=trial.Recording(True); rec.add(block()); rec.bind(7)
        with self.assertRaises(protocol.VerificationError): rec.bind(8)

    def test_compact_collect_and_stop(self):
        class CompactClient(FakeClient):
            def pump(self, cleanup=False):
                self.now += .0016
                if not self.missing and self.next_seq <= 625:
                    self.stream_handler(block(self.next_seq,(self.next_seq-1)*32)); self.next_seq+=1
            def command(self, opcode, payload=b'', cleanup=False):
                if opcode==0x37:
                    self.commands.append(opcode); self.pump()
                    return struct.pack('<IHBB8HI',7,20000,4,32,*trial.CHANNELS,0,0,0,0,0)
                return super().command(opcode,payload,cleanup)
        client=CompactClient(); rec=trial.Recording(True)
        trial.collect(client,rec); self.assertTrue(rec.complete); trial.stop_stream(client)
        self.assertEqual(client.commands.count(0x37),1)
        self.assertTrue(set(client.commands)<={0x37,0x30,0x35,3,8,12})
        with self.assertRaises(protocol.VerificationError): trial.collect(CompactClient(missing=True),trial.Recording(True))
        class WrongAck(CompactClient):
            def command(self, opcode, payload=b'', cleanup=False):
                if opcode==0x37: return FakeClient.command(self,0x30,payload,cleanup)
                return super().command(opcode,payload,cleanup)
        with self.assertRaises(protocol.VerificationError): trial.collect(WrongAck(),trial.Recording(True))

    def test_identity_capability_and_whitelist(self):
        class Identity(FakeClient):
            patch=9; caps=7<<25
            def command(self, opcode, payload=b'', cleanup=False):
                if opcode==1:
                    data=bytearray(24); struct.pack_into('<I',data,12,self.caps); return bytes(data)
                if opcode==2:
                    data=bytearray(40); struct.pack_into('<3H',data,8,0,7,self.patch)
                    struct.pack_into('<I',data,20,0x20261028); data[24:35]=b'AxDrService'; return bytes(data)
                return super().command(opcode,payload,cleanup)
        client=Identity(); trial.identity_gate(client,True)
        client.patch=8
        with self.assertRaises(protocol.VerificationError): trial.identity_gate(client,True)
        client.patch=9; client.caps=3<<25
        with self.assertRaises(protocol.VerificationError): trial.identity_gate(client,True)
        self.assertEqual(protocol.make_request(1,0x37)[:4],b'AXCQ')
        for opcode in (0x38,0x13,0x1c):
            with self.assertRaises(protocol.VerificationError): protocol.make_request(1,opcode)


if __name__ == '__main__': unittest.main()
