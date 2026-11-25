# Receiver OTA Architecture

This document describes how mLRS updates ESP32, ESP8285 and STM32 receivers over the air. The receiver pulls the image, the Tx module relays, and a host tool on the PC answers the receiver's requests.

```
 Host (PC)                  Tx module                      Receiver (ESP32 / ESP8285 / STM32)
 tools/run_rx_ota.py ─USB── ota_relay_tx.h ───LoRa──────── ota_rx.h
 answers requests           bridge, no protocol state      asks, owns all timing
```

## Files

| File | Role |
|---|---|
| `Common/ota/ota_protocol.h` | App info, packet structs, commands |
| `Common/ota/ota_link.h` | LoRa link settings and DIO polling, shared by Tx and Rx |
| `CommonRx/ota_rx.h` | `tRxOta`: the request loop and storage for ESP32 / ESP8266 and the STM32 loader, the STM32 loader's state |
| `CommonRx/ota_loader.cpp` | The STM32 loader, in place of `mlrs-rx.cpp` |
| `CommonTx/ota_relay_tx.h` | `tTxOtaRelay`: the radio ↔ com bridge |
| `tools/run_rx_ota.py` | Host tool |

When OTA isn't built, both classes compile as empty dummies, so `mlrs-tx.cpp` and `mlrs-rx.cpp` need no `#ifdef`s. `USE_RX_OTA` is defined for:
- all transmitters;
- all ESP32 and ESP8266 receivers;
- STM32 receivers with `DEVICE_HAS_OTA_LOADER`, for now the Matek mR900-30(C), mR24-30(C) and mR900-22 (WLE5CC).

## Starting an update

1. The host opens the Tx's com port, checks that the CLI answers `v`, and sends `rxota`.
2. If the link is up, the Tx repeats `FRAME_CMD_RX_OTA_ENTER` in its command frames. One second later it starts the relay.
3. When the receiver gets the command, it leaves its main loop and calls `tRxOta::Run()`. On ESP this doesn't reboot. On STM32 the app hands over to the loader, see below.
4. The host switches to relay framing and waits for the receiver's first request.

## Radio link

The update uses LoRa with explicit header and the radio CRC on, on a fixed frequency per band, `ota_link_freq_reg()` in `ota_link.h`. These are the bind channels of `fhss.h`: 2447, 913.8, 863.275, 865.375, 433.36 and 436.4 MHz. Both sides derive the settings from the band and mode they were connected in. The settings are `tSxLoraConfiguration` entries in `ota_link.h`, applied with the drivers' own `SetLoraConfiguration()`.

| Band / mode | Settings |
|---|---|
| 2.4 GHz | SF5, BW800, CR LI 4/5 |
| Sub-GHz | SF5, BW500, CR 4/5 |
| Sub-GHz, 19 Hz 7x | SF7, BW500, CR 4/5. An SX127x can only be on the other side in this mode, and SF7 is its lowest SF with explicit header. |

LR20xx (LR2021) uses the same settings, with BW812 at 2.4 GHz. LR2021 Tx modules can relay, but no LR2021 receiver has OTA yet: none is ESP, and the STM32 ones have no loader.

The `session_id` (= `Config.FrameSyncWord`) is in every packet, so that only the paired Tx and Rx talk to each other.

Both sides **poll the DIO pin** and read the IRQ status over SPI only after DIO is set. The IRQ status itself is never polled. The STM32WL radio has no DIO pin, so there the pending bit of its disabled radio interrupt takes the place of the pin. `ota_link_sx_only()` does the same on both sides: the radio interrupts go off, and on dual-chip devices sx2 is held in reset with its front end off. The restart of the Tx controller, or the reboot of the receiver, undoes it. The setup enables the CRC-error IRQ on all chips, and the header-error IRQ on all but the SX127x, which has none, so a bad packet is never taken as good.

## Protocol: receiver pull

The protocol is strict request-response. The receiver asks and the host answers; the Tx only relays, and neither the Tx nor the host keeps any state.

```
rx -> host: cmd, session_id[2], payload
host -> rx: cmd | 0x80, session_id[2], payload
```

The payloads are packed structs in `ota_protocol.h`, little endian. The protocol version is 1.

| Cmd | Rx asks | Host answers |
|---|---|---|
| INFO | `tOtaInfoRequest`: version, space[4], flags, device_name_20[20] | `tOtaInfoResponse`: status, length[4], flags, image_length[4], crc[4] |
| DATA | `tOtaDataRequest`: offset[4], max_len | `tOtaDataResponse`: offset[4], data[n], with 1 ≤ n ≤ max_len |
| RESULT | status | – |

- **Which receiver:** the receiver is identified by its device name in plain text, as in the bind and setup frames. The image has it in `tOtaAppInfo` (magic, version, `DEVICE_NAME`), which the host tools find by its magic. The receiver sends the same string in INFO, read from `tOtaAppInfo`. Before starting, `run_rx_ota.py` also checks the name of the connected receiver, which the Tx knows from the setup data (`v` in the CLI).
- **INFO:** if the status isn't OK (wrong receiver or protocol version), the receiver leaves straight away.
- **DATA:** `max_len` is 248, the largest payload that fits a LoRa frame. The receiver advances its offset by whatever it gets, and ignores an answer for any other offset.
- **RESULT:** the receiver repeats it until it gets the answer, at most 5 times.

### Receiver state machine

```
INFO -> DATA (repeat until all received) -> RESULT -> DONE -> restart
```

| Rule | Value |
|---|---|
| No answer to a request | Repeat the same request after 300 ms |
| Retries per request | 5, then leave |
| No answer at all | 5 s, then leave |
| Waiting for the host to appear | 10 s (the relay and tool need a moment to start) |

On ESP, leaving always means a restart. If the update didn't complete, the receiver restarts on its previous firmware. On STM32 the loader decides what comes next, see below.

### Storage

| MCU | How the image is stored | Compression |
|---|---|---|
| ESP32 / C3 / S3 | `esp_ota_*` writes into the inactive app partition; the bootloader switches over | deflate (zlib), inflated with the ROM miniz |
| ESP8266 / ESP8285 | Updater writes into the free flash behind the app; eboot unpacks it over the app at the next reset | gzip, stored as received |
| STM32 | the loader writes the app to its place | none |

- **CRC check:** the receiver checks a CRC over everything transferred before it finalizes. For ESP8266 this is the only check, since eboot doesn't verify the stream. It is CRC-32/MPEG-2, as the platforms have it already: the ESP32 ROM (`esp_rom_crc32_be()`), the ESP8266 core (`crc32()`) and the STM32 CRC unit in its default setup.
- **Compression:** ESP receivers are always sent a compressed image. STM32 receivers advertise no compression and are sent the raw image.
- **Params are untouched:** they live in NVS, which the update never writes.

## STM32 loader

STM32 receivers can't hold a second app, so a loader in front of the app receives the update. The loader is built from the same sources as the app: the CubeMX `main.cpp`, HAL, the device hal, the sx driver, `ota_link.h` and the request loop of `ota_rx.h`. Only `CommonRx/ota_loader.cpp` takes the place of `mlrs-rx.cpp`. It is built first, into `ota_loader.bin`, by `tools/run_make_firmwares.py` when the `.ld` has `mlrs_ota_app`, and in CubeIDE by `CommonRx/ota_loader.mk`, which the project's `makefile.targets` includes into its makefile. In the app's build `ota_loader.cpp` `.incbin`s it into the `.ota_loader` section, which the `.ld` places at the flash start, so the app's `.elf` and `.hex` have both, also when flashed or debugged from CubeIDE. The host tool takes that `.hex` and sends the app part of it.

| G431, 128 kB | |
|---|---|
| `0x08000000` | loader, 14 kB (12.3 kB used) |
| `0x08003800` | loader state, one 2 kB page |
| `0x08004000` | app, up to the EE pages, 104 kB |
| `0x0801E000` | EE pages, powerup counter page, untouched |

The WLE5CC (256 kB, mR900-22) has the same layout, loader 12.7 kB used, app 224 kB, EE pages at `0x0803C000`.

The `.ld` has the `.ota_loader` section and one changed line, in the style of the ELRS bootloader targets: `DEFINED(mlrs_ota_app)` moves the app behind the loader, `DEFINED(mlrs_ota_loader)` limits the loader to 14 kB. An `ASSERT` fails the app's link if the loader's image is not in, as it would be with a stale CubeIDE `objects.list`. The loader sets `VTOR` and starts the app with the clocks and peripherals deinitialized by the HAL, so the app starts as after a reset.

### Loader state

The state page is a list of records (16 bytes, the type word is written last), which are only appended; the last one counts.

| Record | Written by | Meaning |
|---|---|---|
| LINK | app | please update; frequency, session id, band and mode for the link |
| DONE | loader | the request was served, with an update or without |

At boot the loader starts the app, unless:
- the last record is LINK: the app asked for an update;
- the app is invalid: its first double word holds the stack pointer, and is written last, after the crc check over the flash. A half-written app is never started.

In these cases it waits for an update, with the link from the last LINK record, until one comes. When the update loop returns, it writes DONE and starts the app if it is valid, else it keeps waiting. Without a LINK record it doesn't know the link; it blinks red, and only flashing by wire helps.

A new app which is valid is always started. There is no fallback into the loader for an app which runs but never connects, as that would also catch a receiver which is powered a few times without its Tx, e.g. on the bench.

### Measured

Matek mR900-30C (G431, SX1262) from the Nomad (115200 UART), 915 FCC 31 Hz, 77.7 kB raw: 21.6 s, 314 requests, no repeats.

- Update cut off half way, then reset: the loader waits, a second run without a connected receiver completes.

### Open

- A power loss right while a record is written can leave an ECC error in the state page, and reading it gives an NMI.
- Other STM32 families: only G4 and WL are set up. The state records have a word variant, but the app is written in double words (G4, L4, WL), and F1 needs its own crc, since its CRC unit has no initial value.

## Tx relay

```
host -> tx / tx -> host: 0xA5, len, packet[len], crc16     (len = 0 from the host ends the relay)
```

- **Forwarding:** whatever the receiver sends goes to the host, and whatever the host sends goes over the radio.
- **IRQ:** the DIO interrupts are off while the relay runs, and sx2 is held in reset, as on the receiver. The controller restart afterwards undoes it.
- **Ending:** the relay ends when the host sends an empty frame, or after 10 s of silence from the host.
- **Power:** the Tx sends at its lowest power level, as Tx and receiver usually sit next to each other.
- **RAM:** the relay's buffers are on the stack of `Run()`, so OTA costs no RAM outside of an update (R9M: 16 instead of 544 bytes).
- **Transport:** the relay talks to a `tSerialBase`, the com port.

## Host tool

`run_rx_ota.py <port> <firmware.bin>` does the following:
- reads the device name from the image;
- checks the CLI and the link before starting;
- compresses the image according to what the receiver advertises;
- answers requests until the RESULT arrives.

## Measured times

All runs used the compressed image (~216–224 kB for ESP32, 247 kB for ESP8285) at the minimum power level. Every receiver came back on its new firmware.

| Rx | Tx | Mode → OTA link | Time | Requests |
|---|---|---|---|---|
| XR4 (ESP32, LR1121) | Nomad (LR1121) | 2.4 GHz 50 Hz → SF5 | 47.6 s | 902 |
| XR4 | Nomad | 915 31 Hz → SF5 | 61.9 s | 902 |
| XR4 | Nomad | 915 19 Hz 7x → SF7 | 127.7 s | 901 |
| XR1 (ESP32-C3, LR1121) | Nomad | 2.4 GHz 50 Hz → SF5 | 48.7 s | 873 |
| XR1 | Nomad | 915 31 Hz → SF5 | 60.7 s | 872 |
| XR4 | BetaFPV Micro 1W (SX1281) | 2.4 GHz 50 Hz → SF5 | 46.9 s | 901 |
| XR1 | BetaFPV Micro 1W | 2.4 GHz 50 Hz → SF5 | 48.1 s | 872 |
| Generic 900 (ESP8285, SX1276) | Nomad | 915 19 Hz 7x → SF7 | 141.4 s | 997 |
| Generic 900 | Bandit 900 (SX1276) | 915 19 Hz 7x → SF7 | 141.3 s | 1004 |
| XR4 | Matek mR900-30C (G431, SX1262), native USB | 915 31 Hz → SF5 | 39.9 s | 901 |
| Matek mR900-22 (WLE5CC), 63.4 kB raw | Nomad | 915 31 Hz → SF5 | 18.2 s | 262 |

- **Repeated requests:** 0–7 per run.
- **Cross-chip pairs:** SX1281 ↔ LR1121 at 2.4 GHz and LR1121 ↔ SX1276 at SF7 work with explicit header.
- **LR1121 at SF7:** with the SX127x compatibility register left on from 19 Hz 7x mode, it causes no problem.
- **Not run on hardware** (judged low risk): SuperD (ESP32, SX1281) and Generic 2400 D PA (ESP8285, SX1280). LR2021 Tx modules as the relay are build-checked only.

## Findings and future work

- **Privacy must be off**, by design. The OTA link isn't encrypted, so the receiver ignores the enter command when Privacy is on.
- **ESP32, wire flash after an OTA.** The update goes into the other app slot and `otadata` points there. A wire flash which writes only the app at 0x10000 is then not booted, the OTA slot still is. PlatformIO's upload and the Web Flasher also write `boot_app0.bin` at 0xE000, which points `otadata` back at the first slot, so they are fine; plain `esptool write_flash 0x10000` is not.
- **CLI on some Tx modules.** On the BetaFPV and Bandit, the CLI is only on the USB-UART when the 5-way is held down at boot, or when the serial port setting leaves that UART free (`USE_COM_ON_SERIAL`).
