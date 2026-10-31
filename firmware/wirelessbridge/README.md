# mLRS - Firmware - WirelessBridge #

This folder contains pre-compiled wireless-bridge firmware binaries.

mlrs-wireless-bridge-esp32c3.ino.bin
- date of creation: 26.10.2026
- sketch version: 11.10.2026

mlrs-wireless-bridge-esp8266.ino.bin
- date of creation: 26.10.2026
- sketch version: 11.10.2026

## Notes for Devs ##

The binaries are built with tools/run_make_bridge_firmwares.py, which needs arduino-cli with the cores below.
Use -t <name> to build only one target, -u to first install or update the cores.
Install arduino-cli: Windows, Linux see https://arduino.github.io/arduino-cli/latest/installation/, Mac brew install arduino-cli.

- esp32c3: ESP32-C3 Dev Module, Partition Scheme "No OTA (2MB APP/2MB SPIFFS)", MODULE_ESP32C3_ELRS_TX, core esp32:esp32 3.3.12
- esp8266: Generic ESP8266 Module, Flash Size 1MB, Flash Mode DOUT, MODULE_ESP82XX_ELRS_TX, core esp8266:esp8266 3.1.2
