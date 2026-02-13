# mLRS - Release Notes #

## v1.3.08 (pre-release) (13.2.2026) ##

This version brings:

- support Flysky ProArt PA01 internal module, PR02 receiver
- support RadioMaster GX12 internal module, TX15 internal module, Nomad tx module, XR1, XR4
- new Web Flasher app (old Python desktop app deprecated)
- DroneCAN: official support now
- DroneCAN: further hardened
- DroneCAN: support for round-robin statistics in rc message for AP 4.7
- bandage to EdgeTx H7 sport issue (come on guys)
- dualband: crucial fix of transmit stage
- dualband: CLI and OLed improvements
- multiband for LR1121 devices (Nomad, GX12, TX15, XR1, XR4)
- MAVLink: SystemID configuration parameter 
- MAVLink: requests BATT_CAPACITY, to help Yaapu
- MSP: boot per MSP messageT, passthru flashing on MSP/INAV FCs (requires INAV 9.1)
- fix, show 100 LQ at first connection, avoid nasty radio telemetry lost message
- up to 8 power levels on high-power radios
- new, more comprehensible method for switching power levels via radio switch
- USB NAK flow control
- lua: lua script for Ethos/FrSky radios
- lua color: protocol change, much less load failures, reload rarely needed
- lua color: prepared for EdgeTx 3.0
- lua color: support for 320x240 screens (Flysky PA01)
- lua bw: device info page added
- wireless-bridge: support of BLE (in addition to TCP,UDP,UDPSTA,classic BT)
- wireless-bridge: possibility to set password for TCP,UDP,UDPSTA and ssid for UDPSTA via CLI
- devy: no nasty empy for dronecan/run_setup anymore
- devy: wireless bridge sketch significantly reworked to be more C++ and OOP
- devy: etx/otx lua script significantly reworked using tables, mainly to save locals
- devy: MavlinkX, stupid bit operations replaced by proper bit buffers, much faster and less flash

The two major additions in this release are certainly the new Web FLasher app created by @JLP and the Ethos Lua script created by @rob.thomson. How cool is that. Given they are relatively fresh, minor issues may occur; it would be much appreciated if you would report these to us.

This pre-release version is regarded as a release candidate preparing for the next major release v1.4. That is, we will let it run for some weeks to see if and what issues come up, but will not add new features. So, you are welcome to test this version critically, as always.

## v1.3.06 (pre-release) (9.4.2025) ##

This version brings:

- prepared for EdgeTx 2.11 (needed a change in CRSF behavior)
- full support of RadioMaster Bandit, Bandit Micro, "big" Ranger and BetaFPV 1W Micro ELRS Tx modules
    - allows JrBay pin5 use, no limit to only SiK anymore
    - support of wireless bridge functionality (using the the "backpack")
- support of internal ELRS Tx modules for RadioMaster TX16S, TX12, MT12, Zorro, Pocket, Boxer and Jumper T20, T15, T14, T-Pro S radios
    - including support of wireless bridge functionality (using the the "backpack")
- more robust connection (no "low LQ" anymore when in very close range with high power devices)
- corrected connection/reconnection behavior for dual-band systems
- much improved flashing procedures for ESP wireless bridge modules on Tx modules (external and internal)
- 19 Hz mode distinguished into "19 Hz" for sx128x and sx126x devices, and "19 Hz 7x" for sx127x devices
- 25 mW power setting on 900 MHz hardware (usefull for EU)
- rich link stats logging on ArduPilot F7/H7 flight controllers using an ArduPilot Lua script (thx to twistedwings)
- ESP8266 wireless bridge, UDP improved
- bug: rf power switch wasn't recognized correctly at power up (thx to CG Photo)
- bug: high mavlink traffic in tx->rx direction led to message losses (thx to festlv)
- bug: rssi rollover in very close range/high power (thx to b14ckyy)
- bug: rf sensitivity was wrongly reported in Lua script
- some more little bugs
- mLRSFlasher desktop app for easier flashing, supports many flash options, especially also updating of the receiver via AP passthrough

The upcoming EdgeTx 2.11 changed its CRSF behavior, which this mLRS version adapts to. Otherwise RC control would not work. Hence, all users of EdgeTx 2.11 must update their mLRS firmware.

A main addition in this pre-release is the massively widened support of ELRS Tx modules. The external modules can now be fully used, and for quite a number of radios the internal modules are supported as well.

A signifcant change has been made also in how the receivers connect to the Tx modules. The majority of users should not notice any difference, except that the "low LQ" issue when running at high power and connecting in very close range" should be gone now. The changes are very relevant to dual-band users however, where the previous connection method was indeed flawed, especially in far range. That is, dual-band users are strongly adviced to upgrade their mLRS firmware.

The docs are not yet fully reflecting all the changes, and this will need a bit of time. We appologize for this and any inconveniences this may cause.

## v1.3.04 (pre-release) (11.12.2024) ##

This version brings:

- RF power changeable via switch, new parameters "Tx/Rx Power Sw Ch"
- parameter "Tx Cli Lineend" removed, is now always CRLF
- HC04: BT name has been changed, contains now a 5 digit number
- wireless-bridge.ino: esp device name has changed, contains now a 4 digit number
- Lua script color version, handles 320 color displays (like on Jumper T15)
- Lua script bw version, much improved to allow editing of all parameters
- sx1262: rssi math mimics math for sx128x, sx1276
- MSP: arming flags & flight mode improvements
- MSP: link statistics and link info per MSP, shows in OSD, usable with INAV 8.0
- MSP: co-operation with MSP GCSes such as MWP, usable with INAV 8.0
- Matek mR900-30, receiver and tx kit: lower power settings down to 50 mW
- Matek mR24-30, receiver and tx kit: more precise power in lower power settings
- Matek mR900-22: use DC-DC regulator (little bit less power consumption)
- WioE5 Mini board: add system boot handling
- RadioMaster Bandit (micro and "big") and BetaFpv 1W Micro ELRS tx modules supported, but ONLY for SIK
- iFlight ELRS receivers unsupported (unreliable operation)
- better protection against "low LQ" connection issue when Tx module and receiver are high power and in very close proximity
- setup layout versioning, Lua script (color and bw versions) updated to handle setup layout versioning 

In addition, it is a great pleasure to announce two great additions to the docs, the "Quick Start Guide" and "Troubleshooting". Lots of insight and work by @brad112358 went into this.

The major feature added in this release is probably the extensively improved support of MSP, especially for INAV 8. Huge THX go here to @b14ckyy for his pushing, coordination, and relentless testing, and to @MrD-RC for doing the INAV side of the things. As an ArduPilot user, it is actually impressive how quickly this could get realized. The MSP/INAV page in the docs has been updated, and we hope you'll find using INAV 8 and mLRS in conjunction unprecedentedly feature rich and enjoyable. 

Beginning with this release, we will have a tighter version control. We should have done this since ever but didn't for laziness and for it not "hurting" too much so far. Please upgrade your Lua script to the latest.

This release took way longer than anticipated. This is because out of nowhere a critical bug emerged, and it took us 2 months of stressful work to get this sorted. It really required the very close working together of the mLRS dev team, and as lead maintainer I'm proud about "us" here. THX guys, mLRS would be dead if not for you. The good side of the story is that this release is the best stress tested since two years, and we hope you'll find it to be rock solid.

One feature which we hoped would come with this release but doesn't is official DroneCAN support. ArduPilot is unfortunately not yet fully ripe for it, so we need to see how that unfolds further.

## v1.3.02 (pre-release) (9.9.2024) ##

This version brings:

- defaut CLI line endings changed to CRLF (admitedly a small change but removes nastiness)
- improved parameter/mission download for upcoming ArduPilot v4.6 (the good thing, it's auto-detected, mLRS just does it for you)
- dedicated support for MSP, especially for INAV
- few little bugs fixed (all non-critical)

In addition, it is great pleasure to announce the mLRS Web Flasher (https://mlrs.xyz/flash/), which is brought to us by @mustard.tiger. 

It is also mentioned that there is experimental DroneCAN support available in the dev-dronecan branch (https://github.com/olliw42/mLRS/tree/dev-dronecan). Testers are highly welcome.

## v1.3.00 (official release) (19.7.2024) ##

List of main features as compared to release v1.2:

- logo (yes, mLRS has now a logo :scream: )(thx to @mhotar)
- ARQ (retransmission) method added, massively improves link behavior (only rx->tx so far)
- tx MAVLink component, for configuration via MAVLink parameters in GCSes
- support of MatekSys mLRS devices
- tx sending RADIO_STATUS set per default (instead of off)
- support for receiver flashing via ArduPilot passthru
- receiver-side buzzer support removed
- support for dual band, allows operating simultaneously in e.g. the 2.4 GHz and 868/915 MHz bands
- support for ELRS receivers (both based on ESP8285 and ESP32 chip sets)
- tx channel order set to AETR per default
- receivers with no OUT port (like most ELRS receivers) send RC_OVERRIDE per default
- mavlinkX set per default (instead of mavlink)
- account for EByte's mess with changing the E22 hardware (tcxo vs xtal)
- a good number of little bugs removed (all uncritical, just annoying)



