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
* Small PCB Footprint - 40mm x 60mm

## Device Configuration

On each boot the board enables a WiFi access point and web configuration interface; this will automatically disable after 120 seconds if no client connects. The web UI allows you to:

- View and edit per-channel settings (name, sensor type, pull-up, filtering, pressure calibration).
- Configure requird CAN parameters - Base ID and bus speed.
- Backup the entire configuration to a JSON file.
- Restore configuration from a previously exported JSON file.

![esp32-canboard-configuration](docs/esp32-canboard-configuration.png)

Backup behavior:
- Click the `Backup` button in the web UI to download a JSON snapshot of the current configuration. The filename is prefixed with `esp32-canboard-config-` and suffixed with the client timestamp in `ddmmyy-hhmmss` format.

Restore behavior:
- Use the `Restore` button to select a previously exported JSON file. The UI will upload the JSON to the device and validate the payload. The existing configuration is backed up on the device before overwrite; if saving the imported file fails, the device will attempt to restore the previous configuration.

Notes:
- Configuration is persisted on SPIFFS at `/spiffs/config.bin` (binary) and the web UI uses JSON export/import for human-readable backups.
- After restoring a new configuration via the web UI you should reboot the device to apply changes.


## Schematic
[View PDF](docs/esp32-canboard-schematic.pdf)

## Images
![esp32-canboard-iso](docs/esp32-canboard-iso.png)

![esp32-canboard-top](docs/esp32-canboard-top.png)

![esp32-canboard-bottom](docs/esp32-canboard-bottom.png)

## Pinout
|Pin|Function|Additional Information|
|---|---|---|
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