# MK60 passive CAN capture

This standalone ESP-IDF project turns an ESP32-WROOM and SN65HVD230 into a
500 kbit/s, listen-only USB logger. It never acknowledges or transmits CAN
frames. Output is newline-delimited JSON at 921600 baud.

## Wiring

| ESP32 | SN65HVD230 |
|---|---|
| 3V3 | VCC |
| GND | GND |
| GPIO5 | D (TXD) |
| GPIO4 | R (RXD) |

Connect CAN-H, CAN-L, and ground to the vehicle. Do not add a termination
resistor. With the vehicle unpowered, first verify approximately 60 ohms
between CAN-H and CAN-L.

## Build and capture

```sh
idf.py set-target esp32
idf.py build
idf.py -p /dev/your-port flash monitor | tee cluster-connected-1.jsonl
```

Record three ignition cycles with the cluster connected and three with only
the cluster unplugged. Keep the MSS54 and every other CAN module unchanged.
Include ignition-on startup, idle/brake-switch activity, and only where safe a
controlled low-speed wheel-speed sample. Stop the monitor between captures so
each file starts with a fresh boot timestamp.

Compare a pair with:

```sh
python3 compare_captures.py cluster-connected-1.jsonl cluster-disconnected-1.jsonl
```

The comparison lists IDs/rates, logger drops, and every data frame observed in
the 50 ms after a standard `0x610` RTR. Do not enable an emulator profile until
the response bytes and timing repeat across the captures.

The logger deliberately uses ESP-IDF's mature legacy TWAI receive queue. The
new ESP-IDF 6.0.2 node API asserts while querying status for a listen-only node
without a transmit queue on the original ESP32.
