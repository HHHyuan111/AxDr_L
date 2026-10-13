"""普通版被动观测验收；不发送使能、对齐、目标、STOP 或心跳命令。

默认仅执行离线自检。硬件运行必须显式指定 --run --port COM3 --output 新目录。
串口身份必须为 AxDrService 0.7.7 / 0x20261013，且声明 bit25；确认失能、
STOP、无故障后，才能配置被动采集。独立 SWD/供电/Flash 读回预检由操作者完成，
本工具不能根据串口版本号证明 Flash 与候选逐字节相同。
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import io
import json
import math
from pathlib import Path
import secrets
import struct
import tempfile
import time
from typing import Callable
import zlib


MASK = 0xFFFFFFFF
READ_ONLY = frozenset((0x01, 0x02, 0x03, 0x08, 0x0C))
OBSERVATION = frozenset(range(0x30, 0x37))
ALLOWED = READ_ONLY | OBSERVATION
CHANNELS = (1, 2, 3, 8)  # 停机也更新的 Ia/Ib/Ic/Vbus，不能用陈旧 dq 做验收。
CSV_NAMES = ("ia_A", "ib_A", "ic_A", "vbus_V")
DIAG_NAMES = (
    "control_rate_hz", "stream_id", "produced_samples", "dropped_samples", "queued_packets",
    "capture_id", "capture_gaps", "irq_last_cycles", "irq_max_cycles", "irq_overruns",
    "core_clock_hz", "tx_completed", "tx_errors", "tx_aborted", "tx_queue_peak", "irq_count",
    "irq_gap_count", "irq_max_interval_cycles", "adc_overruns",
)


class VerificationError(RuntimeError):
    pass


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise VerificationError(reason)


def crc_frame(data: bytes) -> bytes:
    return data + struct.pack("<I", zlib.crc32(data) & MASK)


def make_request(sequence: int, opcode: int, payload: bytes = b"") -> bytes:
    require(opcode in ALLOWED, f"白名单禁止 opcode 0x{opcode:02X}")
    require(len(payload) <= 44, "请求超过 64 B 契约")
    return crc_frame(struct.pack("<4sHHIHH", b"AXCQ", 1, 20 + len(payload), sequence, opcode, len(payload)) + payload)


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.stats: Counter = Counter()

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 8:
            magic = bytes(self.buffer[:4])
            if magic not in (b"AXCA", b"AXDR", b"AXSF"):
                del self.buffer[0]
                self.stats["resync_bytes"] += 1
                continue
            version, length = struct.unpack_from("<HH", self.buffer, 4)
            valid_length = (magic == b"AXCA" and 24 <= length <= 160) or (magic == b"AXDR" and length == 160) or (magic == b"AXSF" and length == 212)
            if version != 1 or not valid_length:
                del self.buffer[0]
                self.stats["bad_header"] += 1
                continue
            if len(self.buffer) < length:
                break
            frame = bytes(self.buffer[:length])
            del self.buffer[:length]
            if zlib.crc32(frame[:-4]) & MASK != struct.unpack_from("<I", frame, length - 4)[0]:
                self.stats["bad_crc"] += 1
                continue
            self.stats[magic.decode()] += 1
            frames.append(frame)
        return frames


def decode_ack(frame: bytes) -> dict:
    require(frame[:4] == b"AXCA", "不是命令响应")
    sequence, opcode, ack, reason, length, reserved = struct.unpack_from("<IHBBHH", frame, 8)
    require(len(frame) == 24 + length and reserved == 0, "响应长度/保留字段异常")
    return {"sequence": sequence, "opcode": opcode, "ack": ack, "reason": reason, "payload": frame[20:-4]}


def decode_stream(frame: bytes) -> dict:
    require(len(frame) == 212 and frame[:4] == b"AXSF", "高速帧尺寸错误")
    sid, sequence, tick, rate, period, count, channels, flags, drops = struct.unpack_from("<IIIHHBBHI", frame, 8)
    ids = struct.unpack_from("<8H", frame, 32)
    require(1 <= channels <= 4 and count == 5, "高速帧通道/点数异常")
    require(rate == 20000 and period in (20, 40), "高速帧控制时基异常")
    require(all(i == 0 for i in ids[channels:]), "未用通道不为 0")
    values = struct.unpack_from("<40f", frame, 48)
    return {"stream_id": sid, "sequence": sequence, "first_tick": tick, "period": period,
            "sample_count": count, "channel_ids": ids[:channels], "flags": flags, "drops": drops,
            "samples": [values[i * channels:(i + 1) * channels] for i in range(count)]}


def decode_capture(payload: bytes) -> dict:
    require(len(payload) == 52, "捕获状态必须为 52 B")
    schema, size, cid, state, count, trigger, reserved, rate, period = struct.unpack_from("<HHIBBBBHH", payload)
    total, captured, nbytes, first_tick, drops = struct.unpack_from("<5I", payload, 16)
    ids = struct.unpack_from("<4H", payload, 36)
    control_rate, sample_format, reserved1 = struct.unpack_from("<IHH", payload, 44)
    require(schema == 1 and size == 52 and reserved == reserved1 == 0, "捕获 schema/保留字段错误")
    return {"capture_id": cid, "state": state, "channel_count": count, "trigger": trigger,
            "sample_rate_hz": rate, "period": period, "total_samples": total, "captured_samples": captured,
            "total_bytes": nbytes, "first_tick": first_tick, "drops": drops, "channel_ids": ids,
            "control_rate_hz": control_rate, "sample_format": sample_format}


def decode_diagnostics(payload: bytes) -> dict:
    require(len(payload) == 80 and struct.unpack_from("<HH", payload) == (1, 80), "诊断必须为 schema1/80 B")
    result = dict(zip(DIAG_NAMES, struct.unpack_from("<19I", payload, 4)))
    require(result["control_rate_hz"] == 20000, "控制频率不是 20 kHz")
    for field in DIAG_NAMES[7:]:
        require(result[field] != MASK, "硬件诊断尚不可用：" + field)
    require(1_000_000 <= result["core_clock_hz"] <= 1_000_000_000, "核心频率诊断不合理")
    return result


class Client:
    def __init__(self, port, rx_file, tx_file, clock=time.monotonic, timeout=0.8, ack_limit_ms=100.0):
        self.port = port
        self.rx_file = rx_file
        self.tx_file = tx_file
        self.clock = clock
        self.timeout = timeout
        self.ack_limit_ms = ack_limit_ms
        self.parser = FrameParser()
        self.sequence = secrets.randbits(31) + 1
        self.responses: list[dict] = []
        self.stream_handler: Callable[[bytes], None] | None = None
        self.ack_log: list[dict] = []
        self.telemetry_log: list[dict] = []
        self.observation_permitted = False
        self.last_slow_sequence: int | None = None
        self.receive_errors: list[str] = []

    def pump(self, cleanup=False) -> None:
        data = self.port.read(max(1, min(getattr(self.port, "in_waiting", 0), 4096)))
        if not data:
            return
        self.rx_file.write(data)
        previous = self.parser.stats.copy()
        errors = []
        for frame in self.parser.feed(data):
            try:
                if frame[:4] == b"AXCA":
                    self.responses.append(decode_ack(frame))
                elif frame[:4] == b"AXSF":
                    if self.stream_handler:
                        self.stream_handler(frame)
                else:
                    sequence, tick = struct.unpack_from("<II", frame, 8)
                    faults = struct.unpack_from("<I", frame, 32)[0]
                    vbus = struct.unpack_from("<f", frame, 136)[0]
                    sequence_ok = self.last_slow_sequence is None or (sequence - self.last_slow_sequence) & MASK == 1
                    self.last_slow_sequence = sequence
                    self.telemetry_log.append({"sequence": sequence, "tick_ms": tick, "request": frame[16],
                                               "state": frame[17], "faults": faults, "vbus_V": vbus if math.isfinite(vbus) else None})
                    require(sequence_ok, "慢遥测序号缺口")
                    require(frame[16] == frame[17] == 0 and faults == 0, "观测期间检测到运行请求/非STOP/故障；停止验收")
            except VerificationError as error:
                errors.append(str(error))
        for key in ("bad_crc", "bad_header", "resync_bytes"):
            if self.parser.stats[key] != previous[key]:
                errors.append(f"接收异常 {key}: +{self.parser.stats[key] - previous[key]}")
        self.receive_errors.extend(errors)
        # 整批解析后才失败，保留同批 ACK；清理只容忍接收错误，不放宽命令白名单。
        require(cleanup or not errors, "；".join(errors))

    def command(self, opcode: int, payload: bytes = b"", cleanup=False) -> bytes:
        require(opcode in ALLOWED, f"白名单禁止 0x{opcode:02X}")
        require(opcode not in OBSERVATION or self.observation_permitted, "新固件身份/停止门未通过，不发送观测指令")
        self.sequence = (self.sequence + 1) & MASK
        frame = make_request(self.sequence, opcode, payload)
        self.tx_file.write(frame)
        started = self.clock()
        record = {"sequence": self.sequence, "opcode": opcode, "cleanup": cleanup, "status": "pending"}
        self.ack_log.append(record)
        require(self.port.write(frame) == len(frame), "串口写入不完整")
        while self.clock() - started < self.timeout:
            self.pump(cleanup=cleanup)
            for index, response in enumerate(self.responses):
                if response["sequence"] != self.sequence:
                    continue
                self.responses.pop(index)
                elapsed = (self.clock() - started) * 1000.0
                record.update(latency_ms=elapsed, ack=response["ack"], reason=response["reason"], status="received")
                require(response["opcode"] == opcode, "ACK opcode 不一致")
                require(response["reason"] == 0 and response["ack"] in (1, 3), f"命令 0x{opcode:02X} 被拒绝，reason={response['reason']}")
                require(cleanup or elapsed <= self.ack_limit_ms, f"ACK 延迟 {elapsed:.2f} ms 超限")
                return response["payload"]
        record["status"] = "timeout"
        raise VerificationError(f"命令 0x{opcode:02X} 响应超时")

    def wait(self, seconds: float) -> None:
        until = self.clock() + seconds
        while self.clock() < until:
            self.pump()


def stopped_state(client: Client, cleanup=False) -> dict:
    runtime = client.command(0x03, cleanup=cleanup)
    safety = client.command(0x08, cleanup=cleanup)
    control = client.command(0x0C, cleanup=cleanup)
    require(len(runtime) == 36 and len(safety) == len(control) == 40, "状态响应尺寸异常")
    fault = struct.unpack_from("<I", runtime, 16)[0]
    session, sfault = struct.unpack_from("<I", safety, 12)[0], struct.unpack_from("<I", safety, 24)[0]
    control_session = struct.unpack_from("<I", control, 4)[0]
    targets = struct.unpack_from("<4f", control, 24)
    result = {"runtime_request": runtime[4], "runtime_state": runtime[5], "runtime_faults": fault,
              "safety_session": session, "safety_faults": sfault, "safety_state": safety[36],
              "interlocks": safety[38], "run_authorized": safety[39], "control_session": control_session,
              "control_mode": control[20], "control_active": control[21], "targets": targets}
    require(runtime[4] == runtime[5] == fault == sfault == session == control_session == safety[36] == safety[39] == control[20] == control[21] == 0,
            "状态门未通过：必须 STOP、失能、无会话、无授权、无故障")
    require((safety[38] & 1) != 0, "Safety 未确认三相输出关闭")
    require(all(math.isfinite(value) and value == 0.0 for value in targets), "停止状态仍有非零/非有限目标")
    return result


def identity_gate(client: Client) -> dict:
    protocol = client.command(0x01)
    device = client.command(0x02)
    require(len(protocol) == 24 and len(device) == 40, "身份响应尺寸异常")
    version = struct.unpack_from("<3H", device, 8)
    build = struct.unpack_from("<I", device, 20)[0]
    product = device[24:40].split(b"\0", 1)[0].decode("ascii", errors="replace")
    caps = struct.unpack_from("<I", protocol, 12)[0]
    result = {"firmware": version, "build_id": build, "product": product, "capabilities": caps}
    require(version == (0, 7, 7) and build == 0x20261013 and product == "AxDrService", "当前不是配套普通版 0.7.7 / 0x20261013，拒绝启动观测")
    require(caps & (1 << 25) != 0, "固件未声明被动观测能力")
    require(caps & ((1 << 17) | (1 << 18)) == 0, "旧 Scope 能力仍错误广播，拒绝混用主动扫频")
    result["stopped_state"] = stopped_state(client)
    client.observation_permitted = True
    return result


def configure_payload(enabled: bool) -> bytes:
    return struct.pack("<BBH8H", 1 if enabled else 0, 4 if enabled else 0, 1000 if enabled else 0,
                       *(CHANNELS + (0,) * 4 if enabled else (0,) * 8))


def diagnostic_delta(before: dict, after: dict) -> dict:
    require(after["core_clock_hz"] == before["core_clock_hz"], "核心时基发生变化")
    fields = ("produced_samples", "dropped_samples", "capture_gaps", "irq_overruns", "tx_completed", "tx_errors", "tx_aborted", "irq_count", "irq_gap_count", "adc_overruns")
    delta = {name: (after[name] - before[name]) & MASK for name in fields}
    require(delta["irq_count"] > 0, "控制中断未继续执行")
    for field in ("dropped_samples", "capture_gaps", "irq_overruns", "tx_errors", "tx_aborted", "irq_gap_count", "adc_overruns"):
        require(delta[field] == 0, f"窗口内异常计数增加：{field}={delta[field]}")
    return delta


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")


def baseline(client: Client, output: Path, seconds: float) -> dict:
    client.command(0x36)
    client.wait(0.02)
    client.command(0x30, configure_payload(False))
    before = decode_diagnostics(client.command(0x35))
    require(before["stream_id"] == 0, "基线仍有高速流")
    client.wait(seconds)
    after = decode_diagnostics(client.command(0x35))
    result = {"hypothesis": "关闭高速采集时，控制和命令服务无新增时序/传输异常", "duration_s": seconds,
              "before": before, "after": after}
    write_json(output / "baseline.json", result)
    result["delta"] = diagnostic_delta(before, after)
    require(result["delta"]["produced_samples"] == 0, "关闭流时仍产生高速样本")
    result["stopped_state"] = stopped_state(client)
    write_json(output / "baseline.json", result)
    return result


def inspect_stream(frames: list[dict], stream_id: int | None, minimum_samples: int) -> dict:
    expected_sequence = 1
    expected_tick = None
    nan_count = 0
    errors = []
    counts: Counter = Counter()
    for index, frame in enumerate(frames):
        for key, invalid in (("stream_id", frame["stream_id"] != stream_id),
                             ("channels_or_period", tuple(frame["channel_ids"]) != CHANNELS or frame["period"] != 20),
                             ("sequence", frame["sequence"] != expected_sequence),
                             ("tick", expected_tick is not None and frame["first_tick"] != expected_tick),
                             ("flags", frame["flags"] != 0), ("drops", frame["drops"] != 0)):
            if invalid:
                counts[key] += 1
                errors.append(f"packet[{index}] {key} 异常")
        expected_sequence = (frame["sequence"] + 1) & MASK
        expected_tick = (frame["first_tick"] + frame["period"] * frame["sample_count"]) & MASK
        nan_count += sum(not math.isfinite(value) for row in frame["samples"] for value in row)
    count = sum(frame["sample_count"] for frame in frames)
    if nan_count:
        errors.append(f"真实采样通道出现 {nan_count} 个非有限值")
    if not frames:
        errors.append("未收到高速帧")
    if count < minimum_samples:
        errors.append(f"样本不足：{count} < {minimum_samples}")
    return {"packets": len(frames), "samples": count, "nonfinite_values": nan_count,
            "first_control_tick": frames[0]["first_tick"] if frames else None,
            "last_control_tick": (expected_tick - frames[-1]["period"]) & MASK if frames else None,
            "nominal_rate_hz": 1000, "anomaly_counts": dict(counts), "errors": errors,
            "note": "tick 连续性与实际中断间隔计数分别检查；序号/时间异常按相邻实际帧统计"}


def validate_stream(frames: list[dict], stream_id: int, minimum_samples: int) -> dict:
    result = inspect_stream(frames, stream_id, minimum_samples)
    require(not result["errors"], "；".join(result["errors"]))
    return result


def stream_test(client: Client, output: Path, seconds: float) -> dict:
    before = decode_diagnostics(client.command(0x35))
    frames: list[dict] = []
    sid = None
    result = {"hypothesis": "四路 1 kHz 原始流无缺帧，并保持命令和控制中断服务", "duration_s": seconds, "before": before}
    with (output / "stream.axsf.bin").open("wb") as raw, (output / "stream.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("source", "stream_id", "packet_sequence", "control_tick", "relative_time_s", *CSV_NAMES, "flags", "dropped_samples"))
        first_tick = None
        total_samples = 0

        def receive(data: bytes) -> None:
            nonlocal first_tick, total_samples
            raw.write(data)
            frame = decode_stream(data)
            frames.append(frame)
            if first_tick is None:
                first_tick = frame["first_tick"]
            for index, row in enumerate(frame["samples"]):
                tick = (frame["first_tick"] + index * frame["period"]) & MASK
                writer.writerow(("passive_stream", frame["stream_id"], frame["sequence"], tick,
                                 ((tick - first_tick) & MASK) / 20000, *row, frame["flags"], frame["drops"]))
                total_samples += 1

        client.stream_handler = receive
        try:
            accepted = client.command(0x30, configure_payload(True))
            require(len(accepted) == 28, "高速配置响应尺寸异常")
            sid, rate, count, per_packet = struct.unpack_from("<IHBB", accepted)
            require(sid != 0 and (rate, count, per_packet) == (1000, 4, 5), "固件未接受四路 1 kHz")
            until = client.clock() + seconds
            next_status = client.clock() + 1.0
            while client.clock() < until:
                client.pump()
                if client.clock() >= next_status:
                    stopped_state(client)
                    next_status = client.clock() + 1.0
            client.command(0x30, configure_payload(False))
            client.wait(0.05)
            after = decode_diagnostics(client.command(0x35))
            result["after"] = after
            result["waveform"] = inspect_stream(frames, sid, int(seconds * 1000 * 0.95))
            require(not result["waveform"]["errors"], "；".join(result["waveform"]["errors"]))
            result["delta"] = diagnostic_delta(before, after)
            require(after["stream_id"] == 0, "停止指令后高速流未关闭")
            result["stopped_state"] = stopped_state(client)
            return result
        except Exception as error:
            result["error"] = str(error)
            raise
        finally:
            client.stream_handler = None
            result["received_samples"] = total_samples
            result["waveform"] = inspect_stream(frames, sid, int(seconds * 1000 * 0.95))
            write_json(output / "stream.json", result)


def capture_test(client: Client, output: Path) -> dict:
    client.command(0x30, configure_payload(False))
    client.wait(0.02)
    before = decode_diagnostics(client.command(0x35))
    require(before["stream_id"] == 0, "短窗前必须关闭实时流")
    request = struct.pack("<BBBBHHI4H", 4, 1, 0, 0, 20000, 0, 0, *CHANNELS)
    accepted = decode_capture(client.command(0x31, request))
    until = client.clock() + 1.0
    state = accepted
    while client.clock() < until:
        client.wait(0.01)
        state = decode_capture(client.command(0x32))
        if state["state"] in (3, 4):
            break
    write_json(output / "capture-state.json", state)
    require(state["state"] == 3 and state["capture_id"] == accepted["capture_id"], "捕获失败/未在 1 s 内完成")
    require(state["total_samples"] == state["captured_samples"] == 1024 and state["total_bytes"] == 16384, "捕获点数/字节数错误")
    require(state["sample_rate_hz"] == state["control_rate_hz"] == 20000 and state["period"] == state["sample_format"] == 1, "捕获时基/格式错误")
    require(state["trigger"] == 1 and state["drops"] == 0 and state["channel_ids"] == CHANNELS, "捕获通道/触发/连续性错误")
    data = bytearray()
    with (output / "capture.float32.bin").open("wb") as raw:
        while len(data) < state["total_bytes"]:
            payload = client.command(0x33, struct.pack("<II", state["capture_id"], len(data)))
            require(len(payload) == 136, "捕获 chunk 必须为 136 B")
            cid, total, offset, length, flags = struct.unpack_from("<IIIHH", payload)
            require(cid == state["capture_id"] and total == 16384 and offset == len(data), "捕获 chunk 元信息不一致")
            require(length == min(120, total - offset), "捕获 chunk 长度错误")
            require(flags == (1 if offset + length == total else 0), "捕获最后一块标志错误")
            data.extend(payload[16:16 + length])
            raw.write(payload[16:16 + length])
    rows = list(struct.iter_unpack("<4f", data))
    nonfinite = sum(not math.isfinite(value) for row in rows for value in row)
    with (output / "capture.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("source", "capture_id", "sample_index", "control_tick", "relative_time_s", *CSV_NAMES))
        for index, row in enumerate(rows):
            writer.writerow(("passive_capture", state["capture_id"], index, (state["first_tick"] + index) & MASK, index / 20000, *row))
    after = decode_diagnostics(client.command(0x35))
    frozen = decode_capture(client.command(0x32))
    result = {"hypothesis": "人工触发的 1024 点原始短窗连续、冻结、可完整读回", "state": state,
              "before": before, "after": after, "nonfinite_values": nonfinite}
    write_json(output / "capture.json", result)
    require(nonfinite == 0, "捕获含非有限采样值")
    require(state == frozen, "读回过程中捕获状态发生变化")
    result["delta"] = diagnostic_delta(before, after)
    result["stopped_state"] = stopped_state(client)
    write_json(output / "capture.json", result)
    return result


def cleanup(client: Client) -> list[str]:
    errors = []
    if not client.observation_permitted:
        return errors
    for opcode, payload in ((0x36, b""), (0x30, configure_payload(False))):
        try:
            client.command(opcode, payload, cleanup=True)
        except Exception as error:
            errors.append(str(error))
    return errors


def run_board(args) -> int:
    import serial  # 离线自检无需 pyserial，也不会枚举/打开任何端口。
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "kind": "hardware-passive-observation",
              "port": args.port, "phase": args.phase, "status": "RUNNING", "flash_hash_verified_by_this_tool": False,
              "note": "仅停机被动观测；不代表运动性能/降噪通过。IRQ最大值为启动以来高水位，窗口判断使用计数差值。"}
    client = None
    port = None
    try:
        with (output / "rx.bin").open("wb") as rx, (output / "tx.bin").open("wb") as tx:
            port = serial.Serial(args.port, 115200, timeout=0.01, write_timeout=0.5)
            client = Client(port, rx, tx, ack_limit_ms=args.ack_limit_ms)
            try:
                report["identity"] = identity_gate(client)
                report["baseline"] = baseline(client, output, args.baseline_seconds)
                if args.phase in ("stream", "all"):
                    report["stream"] = stream_test(client, output, args.stream_seconds)
                if args.phase in ("capture", "all"):
                    report["capture"] = capture_test(client, output)
                report["status"] = "PASSED"
            except Exception as error:
                report["status"] = "FAILED"
                report["error"] = str(error)
            finally:
                report["cleanup_errors"] = cleanup(client)
                if report["cleanup_errors"]:
                    report["status"] = "FAILED"
                if client.observation_permitted:
                    try:
                        report["final_stopped_state"] = stopped_state(client, cleanup=True)
                        report["final_diagnostics"] = decode_diagnostics(client.command(0x35, cleanup=True))
                        require(report["final_diagnostics"]["stream_id"] == 0, "结束时高速流仍开启")
                    except Exception as error:
                        report["status"] = "FAILED"
                        report["final_check_error"] = str(error)
                report["parser"] = dict(client.parser.stats)
                report["receive_errors"] = client.receive_errors
                if client.receive_errors:
                    report["status"] = "FAILED"
                report["uncompleted_rx_bytes"] = len(client.parser.buffer)
                write_json(output / "ack-latency.json", client.ack_log)
                write_json(output / "slow-state.json", client.telemetry_log)
    except Exception as error:
        report["status"] = "FAILED"
        report["open_or_io_error"] = str(error)
    finally:
        if port is not None:
            port.close()
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        write_json(output / "result.json", report)
    print(json.dumps({"status": report["status"], "output": str(output)}, ensure_ascii=False))
    return 0 if report["status"] == "PASSED" else 1


def selftest() -> int:
    checks = 0

    def check(value, label):
        nonlocal checks
        checks += 1
        assert value, label

    for opcode in (0x09, 0x0A, 0x0B, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x1B, 0x1C, 0x28):
        try:
            make_request(1, opcode)
            raise AssertionError("危险命令未被拒绝")
        except VerificationError:
            check(True, "白名单")
    request = make_request(MASK, 0x30, configure_payload(True))
    check(len(request) == 40 and struct.unpack_from("<I", request, 36)[0] == zlib.crc32(request[:-4]) & MASK, "请求CRC")
    frame = crc_frame(struct.pack("<4sHHIHBBHH", b"AXCA", 1, 28, 7, 0x02, 1, 0, 4, 0) + b"test")
    for split in range(len(frame) + 1):
        parser = FrameParser()
        decoded = parser.feed(frame[:split]) + parser.feed(frame[split:])
        check(decoded == [frame] and decode_ack(frame)["payload"] == b"test", "所有拆包边界")
    damaged = bytearray(frame)
    damaged[20] ^= 1
    parser = FrameParser()
    check(parser.feed(bytes(damaged)) == [] and parser.stats["bad_crc"] == 1, "CRC异常")

    def stream_frame(sequence, tick, flags=0):
        header = struct.pack("<4sHHIIIHHBBHI8H", b"AXSF", 1, 212, 7, sequence, tick, 20000, 20, 5, 4, flags, 0, *CHANNELS, 0, 0, 0, 0)
        return crc_frame(header + struct.pack("<40f", *([0.25, -0.5, 0.25, 24.0] * 5), *([0.0] * 20)))

    frames = [decode_stream(stream_frame(1, MASK - 39)), decode_stream(stream_frame(2, 60))]
    check(validate_stream(frames, 7, 10)["samples"] == 10, "控制tick回绕")
    for replacement in (stream_frame(3, 60), stream_frame(2, 61), stream_frame(2, 60, 1)):
        try:
            validate_stream([frames[0], decode_stream(replacement)], 7, 10)
            raise AssertionError("缺口未被拒绝")
        except VerificationError:
            check(True, "序号/时间/flag拒绝")

    class Clock:
        now = 0.0
        def __call__(self):
            return self.now

    class FakePort:
        def __init__(self, clock, old=False):
            self.clock, self.old, self.received = clock, old, bytearray()
            self.opcodes = []
        @property
        def in_waiting(self):
            return len(self.received)
        def read(self, length):
            self.clock.now += 0.001
            data = bytes(self.received[:length])
            del self.received[:length]
            return data
        def write(self, data):
            sequence, opcode = struct.unpack_from("<IH", data, 8)
            self.opcodes.append(opcode)
            if opcode == 1:
                payload = struct.pack("<6HIIHH", 1, 1, 160, 64, 160, 1, 1 << 25, 0, 0, 0)
            elif opcode == 2:
                payload = struct.pack("<II6HI16s", 0, 0, 0, 7, 6 if self.old else 7, 1, 1, 0, 0x20261013, b"AxDrService")
            elif opcode == 3:
                payload = bytes(36)
            elif opcode == 8:
                payload = bytes(38) + bytes((1, 0))
            elif opcode == 12:
                payload = bytes(40)
            else:
                payload = b""
            self.received += crc_frame(struct.pack("<4sHHIHBBHH", b"AXCA", 1, 24 + len(payload), sequence, opcode, 1, 0, len(payload), 0) + payload)
            return len(data)

    for old in (True, False):
        clock = Clock()
        port = FakePort(clock, old)
        client = Client(port, io.BytesIO(), io.BytesIO(), clock=clock)
        if old:
            try:
                identity_gate(client)
                raise AssertionError("旧固件未被拒绝")
            except VerificationError:
                check(port.opcodes == [1, 2] and not client.observation_permitted and cleanup(client) == [], "旧固件不发观测命令")
        else:
            identity_gate(client)
            check(client.observation_permitted and port.opcodes == [1, 2, 3, 8, 12], "身份状态门")
            check(cleanup(client) == [] and port.opcodes[-2:] == [0x36, 0x30], "finally仅取消和关流")
    values = [20000, 0, 0, 0, 0, 0, 0, 1000, 1200, 0, 160000000, 1, 0, 0, 0, 20, 0, 8000, 0]
    before = decode_diagnostics(struct.pack("<HH19I", 1, 80, *values))
    after = dict(before, irq_count=40)
    check(diagnostic_delta(before, after)["irq_count"] == 20, "诊断差值")
    try:
        diagnostic_delta(before, dict(after, irq_gap_count=1))
        raise AssertionError("硬件时序异常未失败")
    except VerificationError:
        check(True, "硬件异常拒绝")

    # 一批数据前部坏帧不能吞掉后部的取消 ACK，已有错误不能阻断关流。
    clock = Clock()
    port = FakePort(clock)
    client = Client(port, io.BytesIO(), io.BytesIO(), clock=clock)
    identity_gate(client)
    port.received += damaged
    check(cleanup(client) == [] and client.receive_errors and client.parser.stats["bad_crc"] == 1,
          "坏帧后仍完成两条清理命令并保留失败")
    check(stopped_state(client, cleanup=True)["control_active"] == 0, "错误后仍能只读确认最终停机")

    invalid = dict(frames[1], sequence=8, first_tick=61, flags=3, drops=5, samples=[(float("nan"),) * 4] * 5)
    summary = inspect_stream([frames[0], invalid], 7, 10)
    check(summary["nonfinite_values"] == 20 and summary["anomaly_counts"] == {"sequence": 1, "tick": 1, "flags": 1, "drops": 1},
          "所有异常同时计入统计")

    class StagePort(FakePort):
        def __init__(self, clock, corrupt_chunk=False, invalid_sample=False):
            super().__init__(clock)
            self.stream_enabled, self.next_packet, self.stream_sequence = False, 0.0, 0
            self.produced, self.corrupt_chunk = 0, corrupt_chunk
            self.invalid_sample = invalid_sample
            self.capture_bytes = struct.pack("<4096f", *([0.25, -0.5, 0.25, 24.0] * 1024))
            self.state = struct.pack("<HHIBBBBHH5I4HIHH", 1, 52, 11, 3, 4, 1, 0, 20000, 1,
                                     1024, 1024, 16384, MASK - 511, 0, *CHANNELS, 20000, 1, 0)

        def read(self, length):
            data = super().read(length)
            if self.stream_enabled and self.clock.now >= self.next_packet:
                self.next_packet += 0.005
                self.stream_sequence += 1
                self.produced += 5
                packet = stream_frame(self.stream_sequence, (MASK - 39 + (self.stream_sequence - 1) * 100) & MASK)
                if self.invalid_sample and self.stream_sequence == 2:
                    packet = bytearray(packet[:-4])
                    struct.pack_into("<I", packet, 48, 0x7FC00000)
                    packet = crc_frame(packet)
                self.received += packet
            return data

        def write(self, data):
            sequence, opcode = struct.unpack_from("<IH", data, 8)
            if opcode in READ_ONLY or opcode == 0x36:
                return super().write(data)
            self.opcodes.append(opcode)
            if opcode == 0x30:
                self.stream_enabled = bool(data[16])
                if self.stream_enabled:
                    self.next_packet, self.stream_sequence = self.clock.now, 0
                payload = struct.pack("<IHBB8HI", 7 if self.stream_enabled else 0, 1000, 4, 5, *CHANNELS, 0, 0, 0, 0, 0)
            elif opcode == 0x35:
                diagnostics = list(values)
                diagnostics[1] = 7 if self.stream_enabled else 0
                diagnostics[2] = self.produced
                diagnostics[15] = int(self.clock.now * 20000)
                payload = struct.pack("<HH19I", 1, 80, *diagnostics)
            elif opcode in (0x31, 0x32):
                payload = self.state
            elif opcode == 0x33:
                cid, offset = struct.unpack_from("<II", data, 16)
                length = min(120, 16384 - offset)
                payload = struct.pack("<IIIHH", cid, 16384, offset + int(self.corrupt_chunk), length, int(offset + length == 16384))
                payload += self.capture_bytes[offset:offset + length] + bytes(120 - length)
            else:
                raise AssertionError("非预期测试命令")
            self.received += crc_frame(struct.pack("<4sHHIHBBHH", b"AXCA", 1, 24 + len(payload), sequence, opcode, 1, 0, len(payload), 0) + payload)
            return len(data)

    with tempfile.TemporaryDirectory(prefix="axdr-passive-selftest-") as directory:
        output = Path(directory)
        clock = Clock()
        port = StagePort(clock)
        client = Client(port, io.BytesIO(), io.BytesIO(), clock=clock)
        identity_gate(client)
        check(baseline(client, output, 0.05)["delta"]["produced_samples"] == 0, "完整关流基线")
        check(stream_test(client, output, 1.0)["waveform"]["samples"] >= 950, "完整四路1k流程")
        check(capture_test(client, output)["state"]["captured_samples"] == 1024, "完整20k短窗流程")
        check((output / "capture.float32.bin").read_bytes() == port.capture_bytes, "137块完整原始字节读回")
        with (output / "capture.csv").open(encoding="utf-8") as file:
            rows = list(csv.DictReader(file))
        check(len(rows) == 1024 and rows[511]["control_tick"] == str(MASK) and rows[512]["control_tick"] == "0", "短窗CSV回绕与行数")
        check(cleanup(client) == [] and not port.stream_enabled and set(port.opcodes) <= ALLOWED, "完整流程只用被动白名单")
        port.corrupt_chunk = True
        try:
            capture_test(client, output)
            raise AssertionError("错误块偏移未被拒绝")
        except VerificationError:
            check((output / "capture.float32.bin").stat().st_size == 0 and cleanup(client) == [], "错误块未写入数据且仍可清理")
        port.invalid_sample = True
        try:
            stream_test(client, output, 1.0)
            raise AssertionError("NaN流未被拒绝")
        except VerificationError:
            evidence = json.loads((output / "stream.json").read_text(encoding="utf-8"))
            check(evidence["waveform"]["nonfinite_values"] == 1 and evidence["waveform"]["errors"], "失败流持久化无效值统计")

    fixture_dir = Path(__file__).resolve().parents[1] / "artifacts/verification-20261013/observation-offline"
    if fixture_dir.is_dir():
        fixture_parser = FrameParser()
        c_frames = fixture_parser.feed((fixture_dir / "stream.axsf.bin").read_bytes())
        parsed = [decode_stream(value) for value in c_frames]
        check(len(parsed) == 1 and parsed[0]["channel_ids"] == (1, 2, 3, 5), "真实C流通道解码")
        check(parsed[0]["first_tick"] == 0xFFFFFFD8 and parsed[0]["samples"][-1] == (80.125, -80.375, 0.25, 160.25), "真实C流回绕点与数值解码")
        for name, decoder in (("capture-state.axca.bin", decode_capture), ("diagnostics.axca.bin", decode_diagnostics)):
            value = FrameParser().feed((fixture_dir / name).read_bytes())
            check(len(value) == 1 and bool(decoder(decode_ack(value[0])["payload"])), "真实C响应解码 " + name)
    print(f"被动观测工具离线自检：{checks} 项通过；未打开串口、未访问设备。")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="显式执行已批准的停机板端验收")
    parser.add_argument("--self-test", action="store_true", help="只运行离线自检（默认）")
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--phase", choices=("baseline", "stream", "capture", "all"), default="all")
    parser.add_argument("--baseline-seconds", type=float, default=1.0)
    parser.add_argument("--stream-seconds", type=float, default=5.0)
    parser.add_argument("--ack-limit-ms", type=float, default=100.0)
    args = parser.parse_args()
    if not args.run or args.self_test:
        return selftest()
    require(args.output is not None, "板端运行必须指定新的证据目录 --output")
    require(0.5 <= args.baseline_seconds <= 10 and 1 <= args.stream_seconds <= 30, "测量时长超出受控范围")
    require(10 <= args.ack_limit_ms <= 500, "ACK 门限超出 10～500 ms")
    return run_board(args)


if __name__ == "__main__":
    raise SystemExit(main())
