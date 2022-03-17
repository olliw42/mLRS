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
- mavlink parser should probably be reset when packets are missed
- align mavlink messages only with respect to header, i.e., such that header is never split up
- led blink to signal serial traffic


The 3 MAIN topics TODO:

1) Link
- AFC
- retransmissions, resend last one time to make it more robust
- diversity for transmitting (should also work for omni + directional)
- reconnect by choosing frequency based on rssi map

2) Parameters, usability
- parameters instead of compile defines wherever possible
- parameter synchronization between tx and rx upon first connection
- some way for users to adjust parameters (cli? lua? mavlink?)
- bind phase
- firmware update: via connection to USB on tx module, for receiver ota-passthrough

3) Mavlink
- parser on both ends
- router on Tx side
- mavlinkX (transform mavlink data into more robust & slim format, introduce cmpr_msg)

TODO:
- can we pl check the real spi clock speed !?!?!!?

- effect of USE_DCDC? where to place it??

- USB on TX side

Longterm TODO:
- support high_latency2

- get a clean HAL concept

- get a clear mbridge and in concept, currently quite ugly as uart.h conflicts

- when we eventually do OTA update of the rx, the rx firmware must be universal in the sense that it supports all possible
  rx boards. Gladly, rx modules are relatively simple. But: Craft a concept which hopefully will last for a long time.
  It won't work for different hardware platforms!?
  Ultimately I think it should be all G4 (and maybe ESP32)

- long range mode with rx only, switch to a longer-range mode when not connected, would also allow rc data to be transmitted further out
*/




