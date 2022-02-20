//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// TODO
//*******************************************************
/*

ISSUES:
- why is otx not getting the crsf tx & rx link stats telemetry?
- what do the crsf values exactly mean? should rssi be in -db or +db??
- convert crsf power

- LQ 98 in receiver

- rssi -1

- do we want also antenna field for transmit antenna?
  in some way yes, since we have it for receive antenna, but in some way no, is it useful in any way??
  we have the spare bit, so just add it, can be remove later anytime

- do we really want to use RX with timeout in transmitter? couldn't we just use no timeout?

- rc Data: how to handle over-sized values? simply clip to +.100% /currently done), rescale to allow +-120% ?


TODO:
- retransmissions, frame loss, resend last one time to make it more robust
- diversity for transmit

- can we pl check the real spi clock speed !?!?!!?

- effect of USE_DCDC? where to place it??

- sync & configure at first connect
- RX side sbus/sbus-fast/sbus-inverted ppm?  (diversity possible on ArduPilot?)
- RX side rssi output
- RX side failsafe behaviors
- OTA
- configuration settings
- bind phrase
- transform mavlink data into more robust & slim format, introduce cmpr_msg
- USB on TX side


Longterm TODO:

- option to select serial or serial with mavlink parsing, allow e.g. radio_status only in latter mode

- rate management by radio_status txbuf
  note: rts/cts is not a substitute as this wouldn't help in router situations
  idea: use crsf, add new frame type

- support high_latency2

- get a clean HAL concept

- get a clear mbridge and in concept, currently quite ugly as uart.h conflicts

- when we eventually do OTA update of the rx, the rx firmware must be universal in the sense that it supports all possible
  rx boards. Gladly, rx modules are relatively simple. But: Craft a concept which hopefully will last for a long time.
  It won't work for different hardware platforms!?
  Ultimately I think it should be all G4 (and maybe ESP32)

- idea: switch to a longer-range mode when not connected, would also allow rc data to be transmitted further out
*/




