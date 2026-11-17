# mLRS Lua Script for Ethos (FrSky) Radios

A Lua utility for **Ethos transmitters** that provides a graphical configuration interface for **mLRS** devices over the **CRSF protocol**.

This script allows you to **view, edit, and save mLRS parameters** directly from your Ethos radio without connecting to a computer or using the mLRS CLI.


![Screenshot](https://raw.githubusercontent.com/robthomson/mlrs-ethos/main/.github/gfx/screenshot.jpg)

---

## ✨ Features

- 📡 **Direct mLRS integration** via CRSF (Crossfire).
- 🧭 **Automatic discovery** of both Tx module and receiver parameters and metadata.
- ⚙️ **Auto-reconnect** after module reboot or power cycle.
- 🧱 **Fully dynamic UI** — fields are built automatically from mLRS parameter descriptors.
- 🔗 **Binding** — start and stop bind mode; bind mode started from the module's button is shown as well.
- 🛠️ **Tools** — put the Tx module into its System Bootloader or Flash ESP mode, as on the EdgeTX Tools page.
- ✅ **Version check** — refuses Tx/Rx firmware older than v1.3.03 and warns when the Tx and Rx parameter layouts differ.

---

## 🕹️ Installation

1. Download and copy the folder "Ethos" and its content to your radio's SD card, placing it in

```SD:\scripts\mlrs```

2. Restart your radio.

3. In Ethos 1.6 you will find the mLRS menu in the system tools. Please note you will need to ensure that you have switched your protocol to CRSF with a baud rate of 400k.

4. In Ethos 1.7, mLRS will appear in the normal 'external rf module' settings. You will not need to use the system tool and in fact this is not even shown. Simply visit RF module settings; and click options.

5. Visit the tool and access all settings available.

---

## 💾 Saving Settings

- Each edit is sent to the module straight away, but is only kept until the module reloads its settings. The line above the **Save Params** button shows “Pending changes” until you save.
- Tap **Save Params** to make the changes permanent. The status changes from “Saving…” to “Saved”.
- Tap **Reload** to read all parameters from the module again. This discards unsaved changes, because the module reloads its stored settings.

---

## 🧠 Troubleshooting

- **Not all parameters appear?**  
Wait for the load to finish — the status line shows the progress, e.g. “Loading params… (12/45)”.  
If persistent, ensure your CRSF link is stable and firmware up to date.

- **No Receiver section?**  
Receiver parameters are only loaded while a receiver is connected. If you powered the receiver after opening the script, tap **Reload**.

- **“Tx version not supported” or “Rx version not supported”?**  
Update the firmware of that device to v1.3.03 or newer.

- **“param version older than …” next to the status?**  
The Tx and Rx firmware use different parameter layouts. Update the device the message names.

- **No mLRS detected?**  
Make sure the module is powered and connected via CRSF UART.

---

## 🧑‍💻 Development Notes

- Written entirely in **Lua** for the Ethos scripting API.
- Compatible with external mLRS Tx modules using CRSF telemetry (e.g. X18S, X20, etc.).
- All form fields are created dynamically from parameter frames.

---

## 📜 License

GPL-3.0 © 2025 Rob Thomson  
See [LICENSE](LICENSE) for details.

---

## 🧩 Credits

- **Rob Thomson** — development and Ethos integration  
- **mLRS developers** — for open protocol and reference implementations  
- **Ethos Team** — for providing a flexible Lua interface

---
