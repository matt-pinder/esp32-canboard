# ESP32-CANBoard
* ESP32-S3 Dual Core SoC
* MCP2562T CAN Transceiver (up to 1Mbps)
* 10x 5v Tolerant Inputs - Pressure Sensors, NTCs, etc
* 2x 5v Outputs - Fused at 500mA (Thermal Reset)
* USB-C for programming, with JTAG support for debugging
* ESD Protection on both USB and CAN
* JAE Automotive Connector (PCB Socket: MX23A18NF1, Cable Plug: MX23A18SF1)
* Optional pull-up resistors via fused 5v rail for each input (TH 6.3mm)
* Optional 120ohm CAN terminating resistor
* Configuration via web interface over WiFi
* Optional per-channel median filtering with selectable strength (none/low/med/high) to reduce noise
* Small PCB Footprint - 40mm x 60mm

## Build requirement

This project targets ESP-IDF 6.0.2. Activate the 6.0.2 environment before running any `idf.py` command; `main/idf_component.yml` and `dependencies.lock` enforce that toolchain version.

## Device Configuration

The board keeps its WiFi access point available continuously. To reduce idle memory use, the web server starts when a client associates with the access point and stops when that client disconnects.

| SSID | WPA2 Key | Web UI |
|:---|:---|:---|
| ESP32-CanBoard | canconfig | http://192.168.4.1 |

The web UI allows you to:

- View and edit per-channel settings (name, sensor type, pull-up, **filter level** dropdown, pressure calibration).
- Configure required CAN parameters - Base ID and bus speed.
- Configure CAN and ESP-NOW output, including optional CAN bus relay over ESP-NOW.
- Configure up to 16 relay output rules using local sensor values, DBC-imported CAN signals, timers, and pulse lookup tables.
- Adjust pullup vref calculation voltage to allow for LDO regulator output/load.
- View current input voltages and calculated values in real time.
- Backup the entire configuration to a JSON file.
- Restore configuration from a previously exported JSON file.

![esp32-canboard-configuration](docs/esp32-canboard-configuration.png)

| Function | Description |
|:----|:----|
| Save Config | Save current UI settings to the dedicated `config` NVS partition. Changes are validated, persisted and applied immediately. |
| Backup | Download a JSON snapshot of the current configuration. The filename is prefixed with `esp32-canboard-config-` and suffixed with the client timestamp in `ddmmyy-hhmmss` format. |
| Restore | Select a previously exported JSON file. The UI validates it, backs up the previous valid NVS record, commits the replacement, and verifies it by reading it back. |
| Reboot Device | Reboots the device. |

**Notes:**
- Configuration is persisted in the dedicated `config` NVS partition. The web UI uses JSON export/import for human-readable backups.
- On boot, firmware automatically imports a valid legacy `/spiffs/config.bin` into NVS when one is still present and verifies the committed record. The legacy file is left untouched.
- Normal `idf.py flash` updates the application and SPIFFS web assets, but does not write the dedicated `config` partition. Before the first upgrade from a SPIFFS-stored configuration, export a JSON backup (or flash/boot the migration firmware without its SPIFFS target once); a normal project flash replaces the old shared SPIFFS image before firmware can import its config file.
- `erase-flash`, whole-chip images, or explicitly flashing address `0x200000` will still erase configuration; ordinary application/partition-table flashing will not.
- After restoring a new configuration via the web UI the changes are applied immediately.


## CAN Output

The device transmits input data as a set of five CAN frames starting at the configured base ID. All frames use DLC=8 and little-endian byte ordering.

| CAN ID | Name | Payload |
|:---|:---|:---|
| Base ID | analogVoltage_1 | Inputs 0..3 as four uint16 (LSB,MSB) — bytes 0..7 (values in mV) |
| Base ID + 1 | analogVoltage_2 | Inputs 4..7 as four uint16 (LSB,MSB) — bytes 0..7 (values in mV) |
| Base ID + 2 | analogVoltage_3 | Inputs 8..9 as two uint16 (bytes 0..3), dynamic0 (bytes 4..5), dynamic1 (bytes 6..7) |
| Base ID + 3 | dynamicSignals_1 | dynamic2, dynamic3, dynamic4, dynamic5 as four 2-byte values (bytes 0..7) |
| Base ID + 4 | dynamicSignals_2 | dynamic6, dynamic7, dynamic8, dynamic9 as four 2-byte values (bytes 0..7) |

Encoding rules for dynamic values (one per input):
| Channel | Type | Encoding |
|:---|:---|:---|
|analogVoltage|-|unsigned uint16 = voltage * 1000 (resolution 0.001 V)|
|dynamicSignal|Raw|unsigned uint16 = **0** use analogVoltage signal instead|
|dynamicSignal|Pressure|unsigned uint16 = pressure_kPa * 100 (resolution 0.01 kPa)|
|dynamicSignal|NTC|signed int16 = temperature_C * 1 (°C as integer)|

Example DBC for signal names and scaling: [dbc/esp32-canboard.dbc](dbc/esp32-canboard.dbc)

### E46 M3 MK60 cluster emulator (capture-first)

The firmware contains an opt-in MK60 request/response service for cars where
the original instrument cluster has been removed. It is disabled by default
and has no factory response payload: capture the car first with the standalone
listen-only logger in [`tools/mk60-can-capture`](tools/mk60-can-capture).

The `mk60_emulator` object is included in configuration backup/import JSON. A
profile contains the exact standard data frames observed after the MK60's
standard `0x610` RTR request. Only `0x610`, `0x613`, `0x615`, `0x316`, and
`0x329` are accepted, the first response must be `0x610`, and emulation can
only be enabled with a complete profile at 500 kbit/s.

```json
"mk60_emulator": {
  "enabled": false,
  "trigger_id": 1552,
  "trigger_dlc": 8,
  "responses": []
}
```

Populate `responses` from repeatable connected/disconnected captures. Each
entry has `id`, `dlc`, `delay_before_ms`, and an eight-element `data` byte
array. Do not copy example payloads from the internet or enable the feature
until the bytes and timing have been verified on the target MK60. A CAN
transmit error or bus-off condition latches the emulator off until its profile
is reapplied or the board restarts.

### CAN relay over ESP-NOW

When ESP-NOW and **Relay CAN bus** are enabled, externally received CAN frames are sent byte-for-byte to the configured ESP-NOW target as `twai_message_t` values. Physical CAN transmission of the board's own sensor frames may remain disabled; the TWAI controller and CAN speed setting remain active for receiving relay traffic. The CAN receive filter is enabled only while relay mode is active.

Relay traffic is best-effort and lower priority than the board's sensor and GPS output. A received frame remains pending while the ESP-NOW sender is occupied, without blocking ADC sampling or locally generated transmissions. A heavily loaded CAN bus can still produce traffic faster than the receive queue and ESP-NOW link can forward it.

ESP-NOW and the local configuration access point use Wi-Fi channel 1. The receiving ESP-NOW device must also operate on channel 1.

## Dragy GPS Output

When GPS is enabled, the board scans for the configured Dragy BLE MAC, connects to service `FD00`, performs the `FD03` challenge response, and decodes checksum-valid UBX NAV-PVT packets from `FD02`. Valid fixes are published at the GPS update rate; a connected no-fix stream publishes only the status frame, limited to 1 Hz. BLE discovery, decoding, and publishing run independently from ADC sampling and the existing sensor transmit task.

Dragy output uses six standard 11-bit CAN frames starting at the configured GPS base ID. All multi-byte values are little-endian. GPS fields retain their native UBX scaling and IMU fields contain unscaled signed raw counts.

| CAN ID | Bytes | Type | Value |
|:---|:---|:---|:---|
| GPS Base ID | 0 | uint8 | UBX fix type (`0` no fix, `2` 2D, `3` 3D, `4` GNSS + dead reckoning) |
| GPS Base ID | 1 | uint8 | UBX NAV-PVT flags; bit 0 is `gnssFixOK` |
| GPS Base ID | 2 | uint8 | Satellite count |
| GPS Base ID | 3 | uint8 | Dragy battery percent, or `0xFF` when unavailable |
| GPS Base ID | 4..7 | uint32 | Horizontal accuracy in mm |
| GPS Base ID + 1 | 0..3 | uint32 | Ground speed in mm/s |
| GPS Base ID + 1 | 4..7 | int32 | Heading of motion in degrees x 100,000 |
| GPS Base ID + 2 | 0..3 | int32 | Latitude in degrees x 10,000,000 |
| GPS Base ID + 2 | 4..7 | int32 | Longitude in degrees x 10,000,000 |
| GPS Base ID + 3 | 0..3 | int32 | Mean-sea-level altitude in mm |
| GPS Base ID + 3 | 4..7 | uint32 | GPS time of week in ms |
| GPS Base ID + 4 | 0..2 | uint24 | FD05 sample counter |
| GPS Base ID + 4 | 3 | uint8 | FD05 record marker (`0xE1`) |
| GPS Base ID + 4 | 4..5 | int16 | Raw accelerometer X |
| GPS Base ID + 4 | 6..7 | int16 | Raw accelerometer Y |
| GPS Base ID + 5 | 0..1 | int16 | Raw accelerometer Z |
| GPS Base ID + 5 | 2..3 | int16 | Raw gyroscope X |
| GPS Base ID + 5 | 4..5 | int16 | Raw gyroscope Y |
| GPS Base ID + 5 | 6..7 | int16 | Raw gyroscope Z |

The Dragy BLE handshake and stream format are based on the experimental [DragyDash ESP32 protocol notes](https://github.com/jremick/dragy-dash-esp32/blob/main/docs/DRAGY_PROTOCOL.md).

### Dragy Pro IMU capture (experimental)

Testing with a Dragy Pro DRG71 found an undocumented IMU notification stream on characteristic `FD05`. Notifications contain one or more 16-byte records:

| Bytes | Encoding | Observed value |
|:---|:---|:---|
| 0..2 | uint24, big-endian | Sample counter, increasing by 12 per sample |
| 3 | uint8 | Record marker (`0xE1`) |
| 4..9 | 3 x int16, big-endian | Accelerometer X/Y/Z |
| 10..15 | 3 x int16, big-endian | Gyroscope X/Y/Z |

[`tools/dragy_imu_capture.py`](tools/dragy_imu_capture.py) connects directly from a host using [Bleak](https://github.com/hbldh/bleak), performs the existing `FD03` challenge response, subscribes to `FD05`, and writes the decoded samples to CSV:

```sh
python3 -m pip install bleak
python3 tools/dragy_imu_capture.py --duration 20 --output dragy_imu.csv
```

The observed stream rate is approximately 14.5 Hz. The CSV's acceleration conversion currently uses a provisional scale of 4096 counts/g; gyroscope values remain raw until the scale and axis orientation have been calibrated. Disconnect the Dragy mobile app and ESP32 first because the Dragy may only accept one BLE central connection at a time.

When GPS is enabled in the ESP32 configuration, the firmware subscribes to `FD05` alongside `FD02` and publishes every queued IMU record on CAN IDs `GPS Base ID + 4` and `GPS Base ID + 5`. The full counter, marker, and six raw channels are retained so a CAN log can be aligned with a second Dragy's app output for calibration.

The example DBC includes all six Dragy messages at `0x650` through `0x655`, corresponding to the default GPS base ID of `0x650`. GPS fields are converted to metres, m/s, degrees, and seconds by the DBC, while IMU fields remain raw counts. Update those message IDs if a different GPS base ID is configured.

## Schematic
[View PDF](docs/esp32-canboard-schematic.pdf)

## Images
![esp32-canboard-iso](docs/esp32-canboard-iso.png)

![esp32-canboard-top](docs/esp32-canboard-top.png)

![esp32-canboard-bottom](docs/esp32-canboard-bottom.png)

## Connector Pinout
|Pin|Function|Additional Information|
|:---:|:---|:---|
|1|12v Supply||
|2|5v Sensor Supply|500ma Thermal Fuse|
|3|5v Sensor Supply|500ma Thermal Fuse|
|4|Input 6||
|5|Input 7||
|6|Input 8||
|7|Input 9||
|7|Input 10||
|9|CAN High||
|10|Ground||
|11|Ground||
|12|Ground||
|13|Input 1||
|14|Input 2||
|15|Input 3||
|16|Input 4||
|17|Input 5||
|18|CAN Low||
