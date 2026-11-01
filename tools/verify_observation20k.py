"""四路 20 kHz 一秒被动采集试验。默认离线，--run 才打开串口。

只使用已有只读和被动观测协议；不使能、不运动、不复位、不刷写。
实机入口要求另行完成并保存供电/调试器/镜像/输出预检。
原始 RX/TX 与失败结果始终保留；成功只指停机采集，不代表运动工况通过。
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import time
import unittest
import zlib

import verify_passive_observation as protocol

RATE = 20000
SAMPLES = 20000
CHANNELS = (1, 2, 3, 8)
MASK = 0xFFFFFFFF
require = protocol.require


class Recording:
    """按设备 tick 收一秒；超出的停流尾包仍检查，但不并入选定片段。"""

    def __init__(self, compact=False):
        self.compact = compact
        self.per_packet = 32 if compact else 5
        self.frame_size = 564 if compact else 212
        self.magic = b"AXSB" if compact else b"AXSF"
        self.stream_id = None
        self.sequence = 1
        self.expected_tick = None
        self.first_tick = None
        self.frames = bytearray()
        self.values = bytearray()
        self.received_packets = 0
        self.samples = 0
        self.error = None

    def bind(self, sid):
        require(sid != 0 and self.stream_id in (None, sid), "配置 ACK 与收到的流 ID 不一致")
        self.stream_id = sid

    def add(self, frame):
        try:
            require(self.error is None, "记录已失败，不能恢复为通过")
            require(len(frame) == self.frame_size and struct.unpack_from("<4sHH", frame) == (self.magic, 1, self.frame_size), "帧头错误")
            require(zlib.crc32(frame[:-4]) & MASK == struct.unpack_from("<I", frame, self.frame_size - 4)[0], "CRC 错误")
            sid, seq, tick, rate, period, count, channels, flags, drops = struct.unpack_from("<IIIHHBBHI", frame, 8)
            require(sid != 0 and self.stream_id in (None, sid), "流 ID 改变")
            require((rate, period, count, channels) == (RATE, 1, self.per_packet, 4), "不是四路 20 kHz 同拍数据")
            require(struct.unpack_from("<8H", frame, 32) == CHANNELS + (0,) * 4, "通道顺序或未用字段错误")
            require(seq == self.sequence, "序号缺口或重复；不拼接、不补点")
            require(self.expected_tick is None or tick == self.expected_tick, "控制 tick 不连续")
            require(flags == drops == 0, "设备报告缺口、无效采样或丢样")
            data = frame[48:48 + self.per_packet * 16]
            require(all(math.isfinite(x) for x in struct.unpack(f"<{self.per_packet * 4}f", data)), "非有限样本")
            require(self.received_packets < 8000, "停流尾包超过有界上限")
            self.stream_id = sid
            self.sequence = (seq + 1) & MASK
            self.expected_tick = (tick + self.per_packet) & MASK
            if self.first_tick is None:
                self.first_tick = tick
            self.received_packets += 1
            if self.samples < SAMPLES:
                self.frames.extend(frame)
                self.values.extend(data)
                self.samples += self.per_packet
        except protocol.VerificationError as error:
            if self.error is None:
                self.error = str(error)
            raise

    @property
    def complete(self):
        return self.samples == SAMPLES and self.error is None

    def metadata(self):
        return {"wire_format": self.magic.decode(), "samples_per_packet": self.per_packet, "sample_rate_hz": RATE, "channels": list(CHANNELS), "samples_per_channel": self.samples,
                "stream_id": self.stream_id, "first_control_tick": self.first_tick,
                "last_control_tick": None if not self.samples else (self.first_tick + self.samples - 1) & MASK,
                "sample_window_s": self.samples / RATE,
                "first_to_last_span_s": max(0, self.samples - 1) / RATE,
                "tail_packets_checked": max(0, self.received_packets - SAMPLES // self.per_packet),
                "data_complete": self.complete, "error": self.error,
                "raw_float32_sha256": hashlib.sha256(self.values).hexdigest().upper()}

    def save(self, output: Path, accepted: bool):
        # 失败也保存已收到的前缀；文件名和元数据不得冒充成功片段。
        prefix = "recording" if accepted and self.complete else "incomplete"
        (output / (prefix + (".axsb.bin" if self.compact else ".axsf.bin"))).write_bytes(self.frames)
        (output / (prefix + ".float32.bin")).write_bytes(self.values)
        with (output / (prefix + ".csv")).open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(("source", "stream_id", "control_tick", "relative_time_s", *protocol.CSV_NAMES))
            for index, row in enumerate(struct.iter_unpack("<4f", self.values)):
                writer.writerow(("passive_20k_1s", self.stream_id, (self.first_tick + index) & MASK,
                                 format(index / RATE, ".8f"), *row))


def identity_gate(client, compact=False):
    info = client.command(1)
    device = client.command(2)
    require(len(info) == 24 and len(device) == 40, "身份响应长度错误")
    version = struct.unpack_from("<3H", device, 8)
    build = struct.unpack_from("<I", device, 20)[0]
    product = device[24:40].split(b"\0", 1)[0].decode("ascii", errors="replace")
    caps = struct.unpack_from("<I", info, 12)[0]
    client.identity = {"version": version, "build_id": build, "product": product, "capabilities": caps}
    require(version == (0, 7, 9 if compact else 8) and build == 0x20261028 and product == "AxDrService", "不是本轮指定的普通版候选")
    require(caps & (3 << 25) == (3 << 25) and caps & (3 << 17) == 0, "20 kHz 被动能力未声明或与主动扫频混用")
    require(not compact or caps & (1 << 27), "固件未声明紧凑块能力")
    client.identity["stopped_state"] = protocol.stopped_state(client)
    client.observation_permitted = True


def collect(client, recording):
    client.stream_handler = recording.add
    started = client.clock()
    accepted = client.command(0x37 if recording.compact else 0x30, struct.pack("<BBH8H", 1, 4, RATE, *CHANNELS, 0, 0, 0, 0))
    require(len(accepted) == 28, "配置响应长度错误")
    sid, rate, count, per_packet = struct.unpack_from("<IHBB", accepted)
    require((rate, count, per_packet) == (RATE, 4, recording.per_packet), "固件没有接受四路 20 kHz")
    require(struct.unpack_from("<8H", accepted, 8) == CHANNELS + (0,) * 4, "配置应答通道错误")
    require(struct.unpack_from("<I", accepted, 24)[0] == 0, "设备已有丢样，不接受脏状态")
    recording.bind(sid)
    while not recording.complete and client.clock() - started < 3.0:
        client.pump()
    require(recording.complete, f"三秒内未收齐：{recording.samples}/{SAMPLES} 点")


def stop_stream(client):
    # 即使配置响应丢失也尝试关闭本轮可能已启动的流；不取消旧短窗、不发运动 STOP。
    client.command(0x30, protocol.configure_payload(False), cleanup=True)
    until = client.clock() + 0.02
    while client.clock() < until:
        client.pump(cleanup=True)
    after = protocol.decode_diagnostics(client.command(0x35, cleanup=True))
    require(after["stream_id"] == 0 and after["queued_packets"] == 0, "停流尚未回读确认")
    protocol.stopped_state(client, cleanup=True)
    return after


def check_output_registers(preflight):
    ccer, bdtr = preflight.get("tim1_ccer"), preflight.get("tim1_bdtr")
    require(type(ccer) is int and type(bdtr) is int and
            0 <= ccer <= MASK and 0 <= bdtr <= MASK, "TIM1 原始输出寄存器证据缺失")
    # CH1/1N、CH2/2N、CH3/3N 的六路使能位必须全部关闭。
    # CH4 仍用于 ADC 触发，故 MOE 可能为 1；如实记录，不能伪造为关闭。
    require(ccer & 0x555 == 0, "三相 PWM 输出仍使能")
    require(preflight.get("faults") == 0, "故障预检未通过")
    return {"phase_outputs_off": True, "moe_enabled": bool(bdtr & 0x8000),
            "tim1_ccer": ccer, "tim1_bdtr": bdtr}


def finish_observation(client, before, report):
    # 停止确认与采集质量分开；丢样不能改写成“停止未确认”。
    try:
        after = stop_stream(client)
        report["after"] = after
        report["stop_confirmed"] = True
    except Exception as error:
        report["cleanup_error"] = str(error)
        report["stop_confirmed"] = False
        return
    try:
        report["delta"] = protocol.diagnostic_delta(before, after)
    except Exception as error:
        report["diagnostic_error"] = str(error)


def run_board(args):
    # 预检证据由独立 SWD/人工步骤产生；本工具不声称自己验证了镜像和寄存器。
    preflight = json.loads(args.preflight.read_text(encoding="utf-8-sig"))
    image_sha = hashlib.sha256(args.image.read_bytes()).hexdigest().upper()
    require(preflight.get("flash_readback_sha256", "").upper() == image_sha, "镜像读回证据与候选不一致")
    output_state = check_output_registers(preflight)
    require(isinstance(preflight.get("vtref_v"), (int, float)) and 1.0 <= preflight["vtref_v"] <= 3.6,
            "VTref 预检缺失或异常")
    require(preflight.get("jlink_connected") is True, "调试器预检未通过")
    require(isinstance(preflight.get("checked_at_unix"), (int, float)) and
            0 <= time.time() - preflight["checked_at_unix"] <= 600, "预检超过十分钟，不能继承历史状态")
    import serial
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {"status": "FAILED", "kind": "hardware-stopped-20k-1s", "image_sha256": image_sha,
              "preflight_external": preflight, "output_state": output_state, "hardware_motion_tested": False}
    recording = Recording(compact=args.compact)
    client = None
    attempted = False
    try:
        with (output / "rx.bin").open("wb") as rx, (output / "tx.bin").open("wb") as tx:
            with serial.Serial(args.port, 115200, timeout=0.002, write_timeout=0.5) as port:
                client = protocol.Client(port, rx, tx, ack_limit_ms=100)
                try:
                    identity_gate(client, compact=args.compact)
                    before = protocol.decode_diagnostics(client.command(0x35))
                    require(before["stream_id"] == 0 and before["queued_packets"] == 0 and before["dropped_samples"] == 0,
                            "已有流或丢样状态；保留现场，不自动取消或复位")
                    state = protocol.decode_capture(client.command(0x32))
                    require(state["state"] not in (1, 2), "已有短窗录波；保留现场")
                    client.wait(1.0)
                    baseline = protocol.decode_diagnostics(client.command(0x35))
                    report["baseline"] = {"before": before, "after": baseline}
                    report["baseline"]["delta"] = protocol.diagnostic_delta(before, baseline)
                    require(report["baseline"]["delta"]["produced_samples"] == 0, "静默基线仍在产生观测样本")
                    attempted = True
                    collect(client, recording)
                    report["status"] = "DATA_COLLECTED"
                except Exception as error:
                    report["error"] = str(error)
                finally:
                    if attempted:
                        finish_observation(client, baseline, report)
                    report["identity"] = client.identity
                    report["parser"] = dict(client.parser.stats)
                    report["receive_errors"] = list(client.receive_errors)
                    report["ack_max_ms"] = client.ack_max_ms
                    protocol.write_json(output / "ack-log.json", list(client.ack_log))
                    protocol.write_json(output / "slow-state.json", list(client.telemetry_log))
                    report["status"] = "PASSED" if (
                        report["status"] == "DATA_COLLECTED" and recording.complete and report.get("stop_confirmed")
                        and not report.get("diagnostic_error") and not client.receive_errors and client.ack_max_ms <= 100
                    ) else "FAILED"
    except Exception as error:
        report["status"] = "FAILED"
        report["io_error"] = str(error)
    finally:
        report["recording"] = recording.metadata()
        try:
            recording.save(output, report["status"] == "PASSED")
        except Exception as error:
            report["status"] = "FAILED"
            report["export_error"] = str(error)
        protocol.write_json(output / "result.json", report)
    print(json.dumps({"status": report["status"], "samples": recording.samples, "output": str(output)}, ensure_ascii=False))
    return int(report["status"] != "PASSED")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--compact", action="store_true", help="使用 0.7.9 的四路紧凑块协议")
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--preflight", type=Path, help="本轮十分钟内的独立硬件预检 JSON")
    parser.add_argument("--image", type=Path, help="已独立读回核对的候选 BIN")
    args = parser.parse_args()
    if not args.run:
        suite = unittest.defaultTestLoader.discover(str(Path(__file__).resolve().parents[1] / "tests/host"),
                                                   pattern="test_observation20k.py")
        return int(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
    require(args.output is not None and args.preflight is not None and args.image is not None,
            "实机试验需要新证据目录、当前预检和候选镜像")
    return run_board(args)


if __name__ == "__main__":
    raise SystemExit(main())
