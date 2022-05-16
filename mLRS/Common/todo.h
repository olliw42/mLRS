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
- work out what to do if FrequencyBand_allowed_mask, Mode_allowed_mask are different for Rx and Tx (which should not happen)?

- ensure that it works with asymmetric connection, i.e., rx connects to tx, but not tx connects to rx !!!!!!!!!

- sx Init implicitly uses Config, would be nicer to have this disentangled

- setup per model
- make rx name editable
- setup: which are effective only on restart? which on-the-fly?
- entering bind by power cycles

- EVERY tx module needs a means to set the parameters, via SWD?

- restart: we do not want to go through waiting for sx and testing their presence
- for dual sx avoid that both sx.Init wait 300ms
- allow a missing 2nd sx for diversity boards

- mavlink parser should probably be reset when packets are missed
- align mavlink messages only with respect to header, i.e., such that header is never split up
- led blink to signal serial traffic

- crsf baro alt item, can we add more of our own?

The 3 MAIN topics TODO:

1) Link
- AFC
- retransmissions, resend last one time to make it more robust
- diversity for transmitting (should also work for omni + directional)
- reconnect by choosing frequency based on rssi map

2) Parameters, usability
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

- when we eventually do OTA update of the rx, the rx firmware must be universal in the sense that it supports all possible
  rx boards. Gladly, rx modules are relatively simple. But: Craft a concept which hopefully will last for a long time.
  It won't work for different hardware platforms!?
  Ultimately I think it should be all G4 (and maybe ESP32)

- long range mode with rx only, switch to a longer-range mode when not connected, would also allow rc data to be transmitted further out
*/




