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
- Tx, SetupMetaData.rx_available:
  it very occasionally can happen that
  !connected() && (transmit_frame_type == TRANSMIT_FRAME_TYPE_CMD) && (frame->status.frame_type != FRAME_TYPE_TX_RX_CMD)
  is true, and that we may not have gotten fresh SetupMetaData when switching to CONNECT_STATE_CONNECTED
  * general problem: no serial send to rx unless connected to rx
  * how to avoid that race condition at the low level (currently we are ok if connect_occured_once)
  * probably should introduce .rx_available and .rx_valid !?!

- we should take into account in rssi scaling a LNA as well as the db min for a board & setting

- make rx name editable
- setup: which are effective only on restart? which on-the-fly?
- work out what to do if FrequencyBand_allowed_mask, Mode_allowed_mask are different for Rx and Tx (which should not happen)?
  This holds for common parameters. It should work such, that the Tx only provides options also allowed by the Rx.
- work out how to go with setup layout changes

- EVERY tx module needs a means to set the parameters, via SWD?

- restart: we do not want to go through waiting for sx and testing their presence
- for dual sx avoid that both sx.Init wait 300ms
- allow a missing 2nd sx for diversity boards

- mavlink parser should probably be reset when packets are missed
- led blink to signal serial traffic

- crsf baro alt item, can we add more of our own?

- rx R9MM R9MX, why buzzer behaves different for R9MX?

- don't allow bind, save, reload, param changes, etc, when vehicle is armed/flying
  => this requires the tx&rx to know if the vehicle is in this state
  that's possible for when mavlink mode is used, but else?
  => two bits, one to indicate available, one to indicate armed/disarmed
  do it on rx side by parsing mavlink, most robust
- don't allow bind, save, reload, param changes, etc, when connection is too weak, e.g. rssi too low

- buzzer: work out what to do to account for active-low and actiove-high, so that buzzer isn't permanently "on".

- factory reset


The 3 MAIN topics TODO:

1) Link
- AFC
- retransmissions, resend last one time to make it more robust
- reconnect by choosing frequency based on rssi map

2) Parameters, usability
- firmware update: via connection to USB on tx module, for receiver ota-passthrough

3) Mavlink
- parser on both ends
- router on Tx side
- mavlinkX (transform mavlink data into more robust & slim format, introduce cmpr_msg)

4) Auxiliary features
- relay setup, how could this be done?
- 2.4 GHz high-interference mode bw 400kHz, at least for experiment to evaluate
- beacon mode


TODO:
- can we pl check the real spi clock speed !?!?!!?

- effect of USE_DCDC? where to place it??


Longterm TODO:
- support high_latency2

- get a clean HAL concept

- when we eventually do OTA update of the rx, the rx firmware must be universal in the sense that it supports all possible
  rx boards. Gladly, rx modules are relatively simple. But: Craft a concept which hopefully will last for a long time.
  It won't work for different hardware platforms!?
  Ultimately I think it should be all G4 (and maybe ESP32)
  alternatively: the receiver is not automatically updated together with the tx, but simply through the tx module
  yes, that's in view of the growing targets the only way to go, and it is sufficiently user friendly

- long range mode with rx only, switch to a longer-range mode when not connected
  would also allow rc data to be transmitted further out
  the problem is how could we then ever reconnect?
*/




