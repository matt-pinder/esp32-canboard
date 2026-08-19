#!/usr/bin/env python3
"""Capture and decode the Dragy Pro's undocumented FD05 IMU stream."""

import argparse
import asyncio
import csv
import struct
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

FD00 = "0000fd00-0000-1000-8000-00805f9b34fb"
FD02 = "0000fd02-0000-1000-8000-00805f9b34fb"
FD03 = "0000fd03-0000-1000-8000-00805f9b34fb"
FD05 = "0000fd05-0000-1000-8000-00805f9b34fb"
ACCEL_COUNTS_PER_G = 4096.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0, help="capture duration in seconds")
    parser.add_argument("--output", type=Path, default=Path("dragy_imu.csv"), help="output CSV path")
    parser.add_argument("--name", default="DRGPR-7E084E", help="advertised Dragy name")
    return parser.parse_args()


async def capture(args: argparse.Namespace) -> list[tuple]:
    samples: list[tuple] = []

    def on_imu(_characteristic, data: bytearray) -> None:
        payload = bytes(data)
        # FD05 batches one or more fixed-size records in each notification.
        for offset in range(0, len(payload) - 15, 16):
            record = payload[offset : offset + 16]
            counter = int.from_bytes(record[0:3], "big")
            marker = record[3]
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = struct.unpack(">hhhhhh", record[4:])
            samples.append(
                (
                    time.time_ns(),
                    counter,
                    marker,
                    accel_x,
                    accel_y,
                    accel_z,
                    accel_x / ACCEL_COUNTS_PER_G,
                    accel_y / ACCEL_COUNTS_PER_G,
                    accel_z / ACCEL_COUNTS_PER_G,
                    gyro_x,
                    gyro_y,
                    gyro_z,
                )
            )

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
        raise RuntimeError(f"Dragy {args.name!r} was not found; ensure it is on and not connected elsewhere")

    print(f"Connecting to {device.name or args.name} ({device.address})")
    async with BleakClient(device, timeout=20.0) as client:
        # The normal telemetry subscription and handshake keep the device's BLE
        # session alive while FD05 is enabled.
        await client.start_notify(FD02, lambda _characteristic, _data: None)
        await client.start_notify(FD05, on_imu)
        challenge = bytes(await client.read_gatt_char(FD03))
        if len(challenge) < 2:
            raise RuntimeError("Dragy FD03 challenge was shorter than two bytes")
        response = bytes(
            [challenge[0], challenge[1], challenge[0] ^ challenge[1], challenge[0] & challenge[1]]
        )
        await client.write_gatt_char(FD03, response, response=True)
        print(f"Capturing FD05 for {args.duration:g} seconds...")
        await asyncio.sleep(args.duration)

    return samples


def write_csv(path: Path, samples: list[tuple]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(
            [
                "host_time_ns",
                "device_counter",
                "marker",
                "accel_x_raw",
                "accel_y_raw",
                "accel_z_raw",
                "accel_x_g",
                "accel_y_g",
                "accel_z_g",
                "gyro_x_raw",
                "gyro_y_raw",
                "gyro_z_raw",
            ]
        )
        writer.writerows(samples)


def main() -> None:
    args = parse_args()
    samples = asyncio.run(capture(args))
    write_csv(args.output, samples)
    if samples:
        counter_span = samples[-1][1] - samples[0][1]
        print(
            f"Wrote {len(samples)} samples to {args.output}; "
            f"device counter span={counter_span}, marker=0x{samples[0][2]:02X}"
        )
    else:
        print(f"No complete FD05 records received; wrote an empty capture to {args.output}")


if __name__ == "__main__":
    main()
