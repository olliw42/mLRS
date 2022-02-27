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
- LQ 98 in receiver

- do we want also antenna field for transmit antenna?
  in some way yes, since we have it for receive antenna, but in some way no, is it useful in any way??
  we have the spare bit, so just add it, can be remove later anytime

- rc Data: how to handle over-sized values? simply clip to +-100% (as currently done), rescale to allow +-120% ?


The 3 MAIN topics TODO:

1) Link
- retransmissions, resend last one time to make it more robust
- diversity for transmitting (should also work for omni + directional)
- reconnect by choosing frequency based on rssi map

2) Parameters, usability
- parameters instead of compile defines wherever possible
- parameter synchronization between tx and rx upon first connection
- some way for users to adjust parameters (cli? lua?)
- bind phase
- firmware update: via connection to USB on tx module, for receiver ota-passthrough

3) Mavlink
- parser on both ends
- router on Tx side
- mavlinkX (transform mavlink data into more robust & slim format, introduce cmpr_msg)
- rate management

TODO:
- can we pl check the real spi clock speed !?!?!!?

- effect of USE_DCDC? where to place it??

- RX side sbus/sbus-fast/sbus-inverted ppm?  (diversity possible on ArduPilot?)
- RX side rssi output
- RX side failsafe behaviors
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

- long range mode with rx only, switch to a longer-range mode when not connected, would also allow rc data to be transmitted further out
*/




