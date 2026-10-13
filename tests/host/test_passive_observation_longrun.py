"""离线验证30分钟数据量下的统计完整性与内存上界；不导入串口或访问设备。"""
from pathlib import Path
import json
import sys
import tracemalloc

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from verify_passive_observation import CHANNELS, MASK, StreamSummary


def main():
    summary = StreamSummary(7)
    first = MASK - 19999
    frame = {"stream_id": 7, "sequence": 0, "first_tick": first, "period": 20,
             "sample_count": 5, "channel_ids": CHANNELS, "flags": 0, "drops": 0,
             "samples": [(0.25, -0.5, 0.25, 24.0)] * 5}
    tracemalloc.start()
    retained_warm = 0
    for packet in range(360000):
        frame["sequence"] = packet + 1
        frame["first_tick"] = (first + packet * 100) & MASK
        summary.add(frame)
        if packet == 9999:
            retained_warm = tracemalloc.get_traced_memory()[0]
    retained, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    result = summary.result(7, 1800000)
    assert not result["errors"], result
    assert result["packets"] == 360000 and result["samples"] == 1800000, result
    assert result["first_control_tick"] == first
    assert result["last_control_tick"] == (first + 1799999 * 20) & MASK
    assert retained - retained_warm < 65536, (retained, retained_warm)
    assert peak < 262144, peak
    print(json.dumps({"status": "PASSED_OFFLINE", "equivalent_duration_s": 1800,
          "packets": result["packets"], "samples_per_channel": result["samples"],
          "retained_growth_bytes": retained - retained_warm, "peak_traced_bytes": peak,
          "hardware_access": False, "note": "仅统计器合成数据测试，不代表USB、板端或30分钟实测。"},
          ensure_ascii=False))


if __name__ == "__main__":
    main()
