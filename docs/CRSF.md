# mLRS Documentation: CRSF Telemetry and Yaapu Telemetry App #

([back to main page](../README.md))

This page describes how to set up a mLRS system for EdgeTx/OpenTx radios, such that you get the usual CRSF telemetry sensors and can use the Yaapu telemetry app.

Two things need to be done, (1) the mLRS Tx module needs to be put into "CRSF mode", and (2) the flight controller needs to be set up to send a MAVLink stream with the desired MAVLink messages.

In principle, there is no specific configuration of the mLRS receiver neccessary. It is however recommended to set the receiver into "mavlink mode", as described below.

Note: Any radio which supports CRSF should work. This should include many brands besides EdgeTx/OpenTx radios.

Note: An ArduPilot flight controller is assumed. For PX4 and iNav it needs to be tested and seen.


## mLRS Tx Module Setup

- Tx Ch Source = crsf
- Tx Ser Dest = serial or serial2 (not mbridge!)
- Tx Ser Link Mode = mavlink
- Tx Snd RadioStat = off (yes, off!)

Note: There are situations in which it can be usefull to enable "Tx Snd RadioStat", but you should do this only if you know what you are doing. You really should not need it for this setup.


## ArduPilot Setup

Configuration of a serial for MAVLink v2

- SERIALx_BAUD = 57 
- SERIALx_OPTIONS = 0
- SERIALx_PROTOCOL = 2

Configuration of MAVLink stream rates

- SRx_EXT_STATS = 1 (or 2 if you use 31 Hz or 50 Hz mode)
- SRx_EXTRA1 = 4
- SRx_EXTRA2 = 4
- SRx_EXTRA3 = 1 (or 2 if you use 31 Hz or 50 Hz mode)
- SRx_POSITION = 2
- SRx_RAW_SENS = 0 (for most of you this one is unimportant, keep it at 0 unless you really need it)

Configuration for CRSF receiver

Setting up ArduPilot for a CRSF receiver can be a bit tricky, as it can depend on the used board, and might need BRD_ALT_CONFIG to be set to a specific value. It is best to consult the ArduPilot wiki, or ask in the ArduPilot discussion channel.

For my Matek H743 board the configuration is:

- BRD_ALT_CONFIG = 1
- RC_PROTOCOLS = 536 or 512
- SERIAL7_BAUD = irrelevant
- SERIAL7_OPTIONS = 0
- SERIAL7_PROTOCOL = 23


## mLRS Rx Module Setup

These configurations are not strictly neccesary, but recommended.

- Rx Out Mode = crsf
- Rx Ser Link Mode = mavlink
- Rx Snd RadioStat = w txbuf


## OpenTx and Yaapu Telemetry App

The radio needs to be set up for CRSF. This however proceeds exactly as described in common tutorials.

In EdgeTx/OpenTx go as usual to MDL->TELEMETRY and start "Discover new sensors". You should then see plenty of sensors appearing.

Install the Yaapu app exactly as described in its wiki. Note: You need to install the dev version, the stable release version does not work. You can check if all is good by running the "Yaapu Debug CRSF" script in SYS->TOOLS.




