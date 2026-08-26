#!/usr/bin/env python3
"""Compare cluster-connected and cluster-disconnected MK60 CAN JSONL captures."""

from __future__ import annotations

import argparse
import collections
import json
from pathlib import Path


def load_capture(path: Path):
    frames = []
    stats = []
    for raw_line in path.read_text(errors="replace").splitlines():
        start = raw_line.find("{")
        if start < 0:
            continue
        try:
            record = json.loads(raw_line[start:])
        except json.JSONDecodeError:
            continue
        if record.get("type") == "frame":
            frames.append(record)
        elif record.get("type") == "stats":
            stats.append(record)
    return frames, stats


def summarise(frames):
    by_id = collections.defaultdict(list)
    for frame in frames:
        by_id[(frame["id"], frame.get("extended", False), frame.get("rtr", False))].append(frame)
    result = {}
    for key, items in by_id.items():
        timestamps = [item["timestamp_us"] for item in items]
        duration_s = max((timestamps[-1] - timestamps[0]) / 1_000_000, 0.001)
        result[key] = {
            "count": len(items),
            "rate_hz": (len(items) - 1) / duration_s if len(items) > 1 else 0.0,
            "payloads": {tuple(item.get("data", [])) for item in items},
        }
    return result


def print_summary(label, summary):
    print(f"\n{label}")
    for (can_id, extended, rtr), values in sorted(summary.items()):
        flags = (" EXT" if extended else "") + (" RTR" if rtr else "")
        print(f"  0x{can_id:03X}{flags}: count={values['count']} rate={values['rate_hz']:.2f}Hz "
              f"payload_variants={len(values['payloads'])}")


def print_610_windows(frames, window_us):
    print("\nCluster-connected frames following each 0x610 RTR")
    for request in (f for f in frames if f["id"] == 0x610 and f.get("rtr") and not f.get("extended")):
        start = request["timestamp_us"]
        following = [f for f in frames if start <= f["timestamp_us"] <= start + window_us and not f.get("rtr")]
        rendered = []
        for frame in following:
            delta = frame["timestamp_us"] - start
            data = " ".join(f"{value:02X}" for value in frame.get("data", [])[: frame.get("dlc", 0)])
            rendered.append(f"+{delta}us 0x{frame['id']:03X} [{frame.get('dlc', 0)}] {data}")
        print(f"  request @{start}: " + ("; ".join(rendered) if rendered else "no data frames in window"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("connected", type=Path)
    parser.add_argument("disconnected", type=Path)
    parser.add_argument("--window-ms", type=float, default=50.0)
    args = parser.parse_args()

    connected, connected_stats = load_capture(args.connected)
    disconnected, disconnected_stats = load_capture(args.disconnected)
    connected_summary = summarise(connected)
    disconnected_summary = summarise(disconnected)
    print_summary("Cluster connected", connected_summary)
    print_summary("Cluster disconnected", disconnected_summary)

    connected_ids = {key[0] for key in connected_summary}
    disconnected_ids = {key[0] for key in disconnected_summary}
    print("\nIDs absent after cluster removal:", " ".join(f"0x{x:03X}" for x in sorted(connected_ids - disconnected_ids)) or "none")
    print("IDs appearing after cluster removal:", " ".join(f"0x{x:03X}" for x in sorted(disconnected_ids - connected_ids)) or "none")
    print_610_windows(connected, int(args.window_ms * 1000))

    for label, stats in (("connected", connected_stats), ("disconnected", disconnected_stats)):
        if stats:
            last = stats[-1]
            print(f"\n{label} logger health: rx_missed={last.get('rx_missed', 'unknown')} "
                  f"rx_overrun={last.get('rx_overrun', 'unknown')} "
                  f"bus_errors={last.get('bus_errors', 'unknown')} "
                  f"state={last.get('state', 'unknown')}")


if __name__ == "__main__":
    main()
