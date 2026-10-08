#!/usr/bin/env python3
"""Decode a browser-downloaded esp32-canboard .canlog capture."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from pathlib import Path


HEADER = struct.Struct("<8sHHHHIIII")
RECORD = struct.Struct("<IIB8s3x")
TRAILER = struct.Struct("<8sIIIIII")
HEADER_MAGIC = b"CANCAP1\0"
TRAILER_MAGIC = b"CANEND1\0"


def load_capture(path: Path):
    data = path.read_bytes()
    if len(data) < HEADER.size + TRAILER.size:
        raise ValueError("file is too short to be a CAN capture")

    magic, version, header_size, record_size, _reserved, bitrate, duration_ms, start_ms, _reserved2 = HEADER.unpack_from(data)
    if magic != HEADER_MAGIC:
        raise ValueError("header magic is not CANCAP1")
    if version != 1 or header_size != HEADER.size or record_size != RECORD.size:
        raise ValueError(
            f"unsupported format version/sizes: version={version}, "
            f"header={header_size}, record={record_size}"
        )

    trailer_offset = len(data) - TRAILER.size
    trailer = TRAILER.unpack_from(data, trailer_offset)
    if trailer[0] != TRAILER_MAGIC:
        raise ValueError("capture is incomplete: CANEND1 trailer is missing")
    payload_size = trailer_offset - header_size
    if payload_size < 0 or payload_size % record_size:
        raise ValueError("capture record area has an invalid length")

    records = [
        RECORD.unpack_from(data, offset)
        for offset in range(header_size, trailer_offset, record_size)
    ]
    metadata = {
        "bitrate_kbps": bitrate,
        "duration_ms": duration_ms,
        "start_uptime_ms": start_ms,
        "frames_seen": trailer[1],
        "frames_streamed": trailer[2],
        "queue_drops": trailer[3],
        "rx_missed": trailer[4],
        "rx_overrun": trailer[5],
        "bus_errors": trailer[6],
    }
    return records, metadata


def write_candump(records, output):
    for timestamp_us, identifier_flags, dlc, payload in records:
        identifier = identifier_flags & 0x1FFFFFFF
        extended = bool(identifier_flags & (1 << 29))
        rtr = bool(identifier_flags & (1 << 30))
        identifier_text = f"{identifier:08X}" if extended else f"{identifier:03X}"
        if rtr:
            frame = f"{identifier_text}#R{dlc}"
        else:
            frame = f"{identifier_text}#{payload[:dlc].hex().upper()}"
        output.write(f"({timestamp_us / 1_000_000:.6f}) can0 {frame}\n")


def write_csv(records, output):
    writer = csv.writer(output)
    writer.writerow(["timestamp_us", "identifier", "extended", "rtr", "dlc", "data_hex"])
    for timestamp_us, identifier_flags, dlc, payload in records:
        writer.writerow(
            [
                timestamp_us,
                f"0x{identifier_flags & 0x1FFFFFFF:X}",
                int(bool(identifier_flags & (1 << 29))),
                int(bool(identifier_flags & (1 << 30))),
                dlc,
                payload[:dlc].hex().upper(),
            ]
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("--format", choices=("candump", "csv"), default="candump")
    parser.add_argument("-o", "--output", type=Path)
    args = parser.parse_args()

    try:
        records, metadata = load_capture(args.capture)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    output = args.output.open("w", newline="") if args.output else sys.stdout
    try:
        if args.format == "csv":
            write_csv(records, output)
        else:
            write_candump(records, output)
    finally:
        if args.output:
            output.close()

    print(
        "capture: " + ", ".join(f"{key}={value}" for key, value in metadata.items()),
        file=sys.stderr,
    )
    if len(records) != metadata["frames_streamed"]:
        print(
            f"warning: decoded {len(records)} records but trailer reports "
            f"{metadata['frames_streamed']}",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
