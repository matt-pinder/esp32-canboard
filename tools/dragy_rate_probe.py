#!/usr/bin/env python3
"""Probe a Dragy BLE telemetry stream and compare persisted rate settings."""

from __future__ import annotations

import argparse
import asyncio
import collections
import json
import statistics
import struct
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

FD00 = "0000fd00-0000-1000-8000-00805f9b34fb"
FD01 = "0000fd01-0000-1000-8000-00805f9b34fb"
FD02 = "0000fd02-0000-1000-8000-00805f9b34fb"
FD03 = "0000fd03-0000-1000-8000-00805f9b34fb"
GPS_WEEK_MS = 604_800_000
MAX_UBX_PAYLOAD = 4096
CFG_RATE_MEAS = 0x30210001
CFG_RATE_NAV = 0x30210002
CFG_MSGOUT_UBX_NAV_PVT_I2C = 0x20910006
CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007
CFG_MSGOUT_UBX_NAV_PVT_SPI = 0x2091000A
CFG_MSGOUT_UBX_NAV_SAT_I2C = 0x20910015
CFG_MSGOUT_UBX_NAV_DOP_UART1 = 0x20910039
CFG_MSGOUT_UBX_NAV_SAT_UART1 = 0x20910016
CFG_MSGOUT_UBX_NAV_SAT_SPI = 0x20910019
CFG_MSGOUT_UBX_NAV_DOP_I2C = 0x20910038
CFG_MSGOUT_UBX_NAV_DOP_SPI = 0x2091003C
CFG_SIGNAL_GPS_ENA = 0x1031001F
CFG_SIGNAL_GAL_ENA = 0x10310021
CFG_SIGNAL_BDS_ENA = 0x10310022
CFG_SIGNAL_GLO_ENA = 0x10310025

MSGOUT_KEYS = {
    CFG_MSGOUT_UBX_NAV_PVT_I2C: "NAV-PVT I2C",
    CFG_MSGOUT_UBX_NAV_PVT_UART1: "NAV-PVT UART1",
    CFG_MSGOUT_UBX_NAV_PVT_SPI: "NAV-PVT SPI",
    CFG_MSGOUT_UBX_NAV_DOP_I2C: "NAV-DOP I2C",
    CFG_MSGOUT_UBX_NAV_DOP_UART1: "NAV-DOP UART1",
    CFG_MSGOUT_UBX_NAV_DOP_SPI: "NAV-DOP SPI",
    CFG_MSGOUT_UBX_NAV_SAT_I2C: "NAV-SAT I2C",
    CFG_MSGOUT_UBX_NAV_SAT_UART1: "NAV-SAT UART1",
    CFG_MSGOUT_UBX_NAV_SAT_SPI: "NAV-SAT SPI",
}

SIGNAL_KEYS = {
    CFG_SIGNAL_GPS_ENA: "GPS",
    CFG_SIGNAL_GAL_ENA: "Galileo",
    CFG_SIGNAL_BDS_ENA: "BeiDou",
    CFG_SIGNAL_GLO_ENA: "GLONASS",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0, help="capture duration in seconds")
    parser.add_argument("--output", type=Path, default=Path("dragy-rate.json"), help="JSON output path")
    parser.add_argument("--label", default="", help="capture label, for example 10Hz or 25Hz")
    parser.add_argument("--name", default="DRGPR-7E084E", help="advertised Dragy name; empty matches FD00")
    parser.add_argument(
        "--probe-command-path",
        action="store_true",
        help="send a read-only UBX-MON-VER poll to FD01 and look for its response on FD02",
    )
    parser.add_argument(
        "--set-rate",
        type=int,
        choices=(10, 20, 25),
        help="experimentally set the receiver navigation rate in RAM through FD01",
    )
    parser.add_argument(
        "--disable-extra-nav",
        action="store_true",
        help="disable NAV-DOP and NAV-SAT on I2C/UART1/SPI in RAM to test FD02 throughput",
    )
    parser.add_argument(
        "--single-gnss",
        action="store_true",
        help="keep GPS enabled and disable Galileo/BeiDou/GLONASS in RAM",
    )
    parser.add_argument(
        "--compare",
        type=Path,
        nargs="+",
        metavar="CAPTURE",
        help="compare existing capture JSON files instead of connecting",
    )
    return parser.parse_args()


def ubx_checksum_valid(frame: bytes) -> bool:
    ck_a = 0
    ck_b = 0
    for value in frame[2:-2]:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return frame[-2:] == bytes((ck_a, ck_b))


def build_ubx_frame(message_class: int, message_id: int, payload: bytes = b"") -> bytes:
    body = bytes((message_class, message_id)) + len(payload).to_bytes(2, "little") + payload
    ck_a = 0
    ck_b = 0
    for value in body:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b"\xb5\x62" + body + bytes((ck_a, ck_b))


def build_cfg_valset_u2(key: int, value: int) -> bytes:
    # Version 0, RAM layer only, no transaction. This deliberately avoids BBR
    # and flash so an experimental rate change is not made persistent.
    payload = b"\x00\x01\x00\x00" + struct.pack("<IH", key, value)
    return build_ubx_frame(0x06, 0x8A, payload)


def build_cfg_valset_u1(key: int, value: int) -> bytes:
    # Version 0, RAM layer only, no transaction.
    payload = b"\x00\x01\x00\x00" + struct.pack("<IB", key, value)
    return build_ubx_frame(0x06, 0x8A, payload)


def build_cfg_valset_u1_many(values: dict[int, int]) -> bytes:
    # Applying all signal changes in one message causes one GNSS subsystem reset
    # instead of resetting separately for every disabled constellation.
    cfg_data = b"".join(struct.pack("<IB", key, value) for key, value in values.items())
    return build_ubx_frame(0x06, 0x8A, b"\x00\x01\x00\x00" + cfg_data)


def build_cfg_valget(keys: list[int]) -> bytes:
    # Version 0, RAM layer, position 0, followed by the requested key IDs.
    payload = b"\x00\x00\x00\x00" + b"".join(struct.pack("<I", key) for key in keys)
    return build_ubx_frame(0x06, 0x8B, payload)


def decode_u1_cfg_values(
    messages: list[dict[str, Any]], key_names: dict[int, str]
) -> dict[str, int]:
    values: dict[str, int] = {}
    for message in messages:
        if (message["class"], message["id"]) != (0x06, 0x8B):
            continue
        frame = bytes.fromhex(message["raw_hex"])
        payload = frame[6:-2]
        if len(payload) < 4 or payload[0] != 1:
            continue
        offset = 4
        while offset + 5 <= len(payload):
            key = int.from_bytes(payload[offset : offset + 4], "little")
            if key not in key_names:
                break
            values[key_names[key]] = payload[offset + 4]
            offset += 5
    return values


class UbxStreamParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.discarded_bytes = 0
        self.bad_checksums = 0

    def feed(self, fragment: bytes, host_time_ns: int) -> list[dict[str, Any]]:
        self.buffer.extend(fragment)
        messages: list[dict[str, Any]] = []

        while True:
            sync = self.buffer.find(b"\xb5\x62")
            if sync < 0:
                keep = 1 if self.buffer.endswith(b"\xb5") else 0
                self.discarded_bytes += len(self.buffer) - keep
                if keep:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if sync:
                self.discarded_bytes += sync
                del self.buffer[:sync]
            if len(self.buffer) < 6:
                break

            payload_length = int.from_bytes(self.buffer[4:6], "little")
            if payload_length > MAX_UBX_PAYLOAD:
                self.discarded_bytes += 2
                del self.buffer[:2]
                continue

            frame_length = payload_length + 8
            if len(self.buffer) < frame_length:
                break

            frame = bytes(self.buffer[:frame_length])
            del self.buffer[:frame_length]
            if not ubx_checksum_valid(frame):
                self.bad_checksums += 1
                continue

            message_class = frame[2]
            message_id = frame[3]
            i_tow_ms = None
            if (message_class, message_id) in ((0x01, 0x07), (0x28, 0x00)) and payload_length >= 4:
                i_tow_ms = int.from_bytes(frame[6:10], "little")

            messages.append(
                {
                    "host_time_ns": host_time_ns,
                    "class": message_class,
                    "id": message_id,
                    "payload_length": payload_length,
                    "i_tow_ms": i_tow_ms,
                    "raw_hex": frame.hex(),
                }
            )

        return messages


def elapsed_itow_ms(newer: int, older: int) -> int:
    return newer - older if newer >= older else GPS_WEEK_MS - older + newer


def rate_summary(messages: list[dict[str, Any]], message_class: int, message_id: int) -> dict[str, Any]:
    selected = [
        message
        for message in messages
        if message["class"] == message_class
        and message["id"] == message_id
        and message["i_tow_ms"] is not None
    ]
    deltas = [
        elapsed_itow_ms(current["i_tow_ms"], previous["i_tow_ms"])
        for previous, current in zip(selected, selected[1:])
    ]
    positive_deltas = [delta for delta in deltas if delta > 0]
    median_delta = statistics.median(positive_deltas) if positive_deltas else None
    host_span_s = (
        (selected[-1]["host_time_ns"] - selected[0]["host_time_ns"]) / 1_000_000_000
        if len(selected) > 1
        else None
    )
    return {
        "count": len(selected),
        "itow_median_delta_ms": median_delta,
        "itow_rate_hz": 1000.0 / median_delta if median_delta else None,
        "itow_delta_counts": dict(sorted(collections.Counter(positive_deltas).items())),
        "host_rate_hz": (len(selected) - 1) / host_span_s if host_span_s and host_span_s > 0 else None,
        "first_itow_ms": selected[0]["i_tow_ms"] if selected else None,
        "last_itow_ms": selected[-1]["i_tow_ms"] if selected else None,
    }


def build_summary(
    messages: list[dict[str, Any]],
    notification_count: int,
    notification_lengths: collections.Counter[int],
    duration_s: float,
    parser: UbxStreamParser,
) -> dict[str, Any]:
    message_counts = collections.Counter(
        f"{message['class']:02X}/{message['id']:02X}" for message in messages
    )
    return {
        "fd02_notification_count": notification_count,
        "fd02_notification_rate_hz": notification_count / duration_s if duration_s > 0 else None,
        "fd02_notification_lengths": dict(sorted(notification_lengths.items())),
        "ubx_message_counts": dict(sorted(message_counts.items())),
        "discarded_stream_bytes": parser.discarded_bytes,
        "bad_ubx_checksums": parser.bad_checksums,
        "nav_pvt": rate_summary(messages, 0x01, 0x07),
        "hnr_pvt": rate_summary(messages, 0x28, 0x00),
        "msgout_ram_values": decode_u1_cfg_values(messages, MSGOUT_KEYS),
        "signal_ram_values": decode_u1_cfg_values(messages, SIGNAL_KEYS),
    }


async def characteristic_snapshot(client: Any) -> tuple[list[dict[str, Any]], bytes | None]:
    snapshot: list[dict[str, Any]] = []
    fd03_challenge = None

    for service in client.services:
        for characteristic in service.characteristics:
            item: dict[str, Any] = {
                "service_uuid": str(service.uuid).lower(),
                "uuid": str(characteristic.uuid).lower(),
                "handle": characteristic.handle,
                "properties": sorted(characteristic.properties),
            }
            if "read" in characteristic.properties:
                try:
                    value = bytes(await client.read_gatt_char(characteristic))
                    item["value_hex"] = value.hex()
                    if item["uuid"] == FD03:
                        fd03_challenge = value
                except Exception as error:  # BLE devices may reject otherwise advertised reads.
                    item["read_error"] = str(error)
            snapshot.append(item)

    return snapshot, fd03_challenge


async def capture(args: argparse.Namespace) -> dict[str, Any]:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError as error:
        raise RuntimeError("bleak is required: python3 -m pip install bleak") from error

    expected_name = args.name.upper()
    device = await BleakScanner.find_device_by_filter(
        lambda dev, adv: (adv.local_name or "").upper() == expected_name
        or (
            expected_name == ""
            and FD00 in [str(value).lower() for value in adv.service_uuids or []]
        ),
        timeout=15.0,
    )
    if device is None:
        raise RuntimeError(f"Dragy {args.name!r} was not found; disconnect it from the app first")

    parser = UbxStreamParser()
    messages: list[dict[str, Any]] = []
    notification_count = 0
    notification_lengths: collections.Counter[int] = collections.Counter()
    commands_sent: list[dict[str, Any]] = []

    def on_fd02(_characteristic: Any, data: bytearray) -> None:
        nonlocal notification_count
        payload = bytes(data)
        notification_count += 1
        notification_lengths[len(payload)] += 1
        messages.extend(parser.feed(payload, time.time_ns()))

    print(f"Connecting to {device.name or args.name} ({device.address})")
    async with BleakClient(device, timeout=20.0) as client:
        characteristics, challenge = await characteristic_snapshot(client)
        fd00_characteristics = [item for item in characteristics if item["service_uuid"] == FD00]
        print("FD00 characteristics:")
        for item in fd00_characteristics:
            value = f" value={item['value_hex']}" if "value_hex" in item else ""
            print(f"  {item['uuid'][4:8].upper()}: {','.join(item['properties'])}{value}")

        await client.start_notify(FD02, on_fd02)
        if challenge is None:
            challenge = bytes(await client.read_gatt_char(FD03))
        if len(challenge) < 2:
            raise RuntimeError("Dragy FD03 challenge was shorter than two bytes")
        response = bytes(
            [challenge[0], challenge[1], challenge[0] ^ challenge[1], challenge[0] & challenge[1]]
        )
        await client.write_gatt_char(FD03, response, response=True)
        await asyncio.sleep(0.25)

        if args.probe_command_path:
            command = build_ubx_frame(0x0A, 0x04)
            await client.write_gatt_char(FD01, command, response=True)
            commands_sent.append(
                {"purpose": "poll MON-VER", "target": "FD01", "raw_hex": command.hex()}
            )
            print("Sent UBX-MON-VER poll to FD01; expecting UBX 0A/04 on FD02")

        if args.single_gnss:
            query = build_cfg_valget(list(SIGNAL_KEYS))
            await client.write_gatt_char(FD01, query, response=True)
            commands_sent.append(
                {"purpose": "poll CFG-SIGNAL", "target": "FD01", "raw_hex": query.hex()}
            )
            await asyncio.sleep(0.25)

            command = build_cfg_valset_u1_many(
                {
                    CFG_SIGNAL_GPS_ENA: 1,
                    CFG_SIGNAL_GAL_ENA: 0,
                    CFG_SIGNAL_BDS_ENA: 0,
                    CFG_SIGNAL_GLO_ENA: 0,
                }
            )
            await client.write_gatt_char(FD01, command, response=True)
            commands_sent.append(
                {"purpose": "CFG-SIGNAL GPS-only", "target": "FD01", "raw_hex": command.hex()}
            )
            print("Selected RAM-only GPS mode; waiting for the GNSS subsystem restart")
            await asyncio.sleep(1.0)

        if args.set_rate:
            measurement_ms = 1000 // args.set_rate
            rate_commands = (
                ("CFG-RATE-MEAS", build_cfg_valset_u2(CFG_RATE_MEAS, measurement_ms)),
                ("CFG-RATE-NAV", build_cfg_valset_u2(CFG_RATE_NAV, 1)),
            )
            for purpose, command in rate_commands:
                await client.write_gatt_char(FD01, command, response=True)
                commands_sent.append(
                    {"purpose": purpose, "target": "FD01", "raw_hex": command.hex()}
                )
                await asyncio.sleep(0.05)
            print(
                f"Sent RAM-only navigation rate configuration: {args.set_rate} Hz "
                f"({measurement_ms} ms, NAV ratio 1)"
            )

        if args.disable_extra_nav:
            query = build_cfg_valget(list(MSGOUT_KEYS))
            await client.write_gatt_char(FD01, query, response=True)
            commands_sent.append(
                {"purpose": "poll CFG-MSGOUT ports", "target": "FD01", "raw_hex": query.hex()}
            )
            await asyncio.sleep(0.25)

            message_commands = tuple(
                (
                    f"CFG-MSGOUT-{MSGOUT_KEYS[key].replace(' ', '_')}=0",
                    build_cfg_valset_u1(key, 0),
                )
                for key in (
                    CFG_MSGOUT_UBX_NAV_DOP_I2C,
                    CFG_MSGOUT_UBX_NAV_DOP_UART1,
                    CFG_MSGOUT_UBX_NAV_DOP_SPI,
                    CFG_MSGOUT_UBX_NAV_SAT_I2C,
                    CFG_MSGOUT_UBX_NAV_SAT_UART1,
                    CFG_MSGOUT_UBX_NAV_SAT_SPI,
                )
            )
            for purpose, command in message_commands:
                await client.write_gatt_char(FD01, command, response=True)
                commands_sent.append(
                    {"purpose": purpose, "target": "FD01", "raw_hex": command.hex()}
                )
                await asyncio.sleep(0.05)
            print("Disabled NAV-DOP and NAV-SAT on I2C, UART1, and SPI in the RAM layer")

        print(f"Capturing all FD02 UBX messages for {args.duration:g} seconds...")
        await asyncio.sleep(args.duration)

    summary = build_summary(
        messages, notification_count, notification_lengths, args.duration, parser
    )
    return {
        "format_version": 1,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "label": args.label,
        "requested_duration_s": args.duration,
        "device": {"name": device.name or args.name, "address": device.address},
        "commands_sent": commands_sent,
        "characteristics": characteristics,
        "summary": summary,
        "messages": messages,
    }


def format_rate(value: Any) -> str:
    return f"{value:.2f} Hz" if isinstance(value, (int, float)) else "none"


def print_summary(capture_data: dict[str, Any]) -> None:
    summary = capture_data["summary"]
    nav = summary["nav_pvt"]
    hnr = summary["hnr_pvt"]
    print(f"UBX messages: {summary['ubx_message_counts']}")
    print(
        "NAV-PVT: "
        f"{nav['count']} frames, iTOW={format_rate(nav['itow_rate_hz'])}, "
        f"host={format_rate(nav['host_rate_hz'])}, deltas={nav['itow_delta_counts']}"
    )
    print(
        "HNR-PVT: "
        f"{hnr['count']} frames, iTOW={format_rate(hnr['itow_rate_hz'])}, "
        f"host={format_rate(hnr['host_rate_hz'])}"
    )
    print(
        f"FD02 notifications: {summary['fd02_notification_count']} "
        f"({summary['fd02_notification_rate_hz']:.2f} Hz), "
        f"lengths={summary['fd02_notification_lengths']}"
    )
    if summary["bad_ubx_checksums"] or summary["discarded_stream_bytes"]:
        print(
            f"Parser diagnostics: bad checksums={summary['bad_ubx_checksums']}, "
            f"discarded bytes={summary['discarded_stream_bytes']}"
        )
    if summary["msgout_ram_values"]:
        print(f"CFG-MSGOUT RAM values before disabling: {summary['msgout_ram_values']}")
    if summary["signal_ram_values"]:
        print(f"CFG-SIGNAL RAM values before single-GNSS mode: {summary['signal_ram_values']}")
    command_purposes = {item["purpose"] for item in capture_data.get("commands_sent", [])}
    if "poll MON-VER" in command_purposes:
        response_count = summary["ubx_message_counts"].get("0A/04", 0)
        print(
            "FD01 UBX command path: "
            + (f"confirmed ({response_count} MON-VER response)" if response_count else "no MON-VER response")
        )
    if any(purpose.startswith("CFG-") for purpose in command_purposes):
        ack_count = summary["ubx_message_counts"].get("05/01", 0)
        nak_count = summary["ubx_message_counts"].get("05/00", 0)
        print(f"CFG response messages: ACK={ack_count}, NAK={nak_count}")


def comparable_characteristics(capture_data: dict[str, Any]) -> dict[str, str]:
    return {
        item["uuid"]: item["value_hex"]
        for item in capture_data.get("characteristics", [])
        if "value_hex" in item and item["uuid"] != FD03
    }


def compare_captures(paths: list[Path]) -> None:
    captures = [(path, json.loads(path.read_text())) for path in paths]
    print("label\tNAV-PVT iTOW\thost rate\tmedian delta\tHNR-PVT\tUBX messages")
    for path, capture_data in captures:
        summary = capture_data["summary"]
        nav = summary["nav_pvt"]
        label = capture_data.get("label") or path.stem
        print(
            f"{label}\t{format_rate(nav['itow_rate_hz'])}\t"
            f"{format_rate(nav['host_rate_hz'])}\t{nav['itow_median_delta_ms']} ms\t"
            f"{summary['hnr_pvt']['count']}\t{summary['ubx_message_counts']}"
        )

    all_uuids = sorted(
        set().union(*(comparable_characteristics(data) for _, data in captures))
    )
    changed = []
    for uuid in all_uuids:
        values = [comparable_characteristics(data).get(uuid, "<unreadable>") for _, data in captures]
        if len(set(values)) > 1:
            changed.append((uuid, values))

    if changed:
        print("\nReadable characteristic differences (FD03 challenge excluded):")
        for uuid, values in changed:
            print(f"  {uuid}: {values}")
    else:
        print("\nNo readable characteristic value changed between captures.")


def main() -> None:
    args = parse_args()
    if args.compare:
        compare_captures(args.compare)
        return

    capture_data = asyncio.run(capture(args))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(capture_data, indent=2) + "\n")
    print_summary(capture_data)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
