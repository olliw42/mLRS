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

- After editing values, tap **Save** at the bottom of the form to write the changes to the mLRS devices and make them permanent.
- The script will:
1. Send the `PARAM_STORE` command to mLRS.
2. Keep the progress loader open while the module reboots.
3. Automatically reconnect and reload the parameters.
- The loader text will change from “Writing…” → “Reconnecting…” → “Complete” once ready.

---

## 🧠 Troubleshooting

- **Not all parameters appear?**  
Wait a few seconds — some metadata arrives after the main list.  
If persistent, ensure your CRSF link is stable and firmware up to date.

- **Progress dialog flickers or closes early?**  
This version uses a *persistent loader* that stays visible through save → reboot → reconnect.  
If you still see flicker, verify that your Ethos version supports `form.openProgressDialog()` properly.

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
