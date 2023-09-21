//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Theory of Operation
//*******************************************************
// Documents the operation and functioning of certain mechanisms.
/*




==============================
Relay Mode
==============================

Two pairs of tx module and receiver are daisy chained. It may be more than two pairs, but let's stick
to only two for the sake of the discussion.

For notation:

radio <-CRSF-> secondary <-air-> secondary <-CRSF,serial-> main      <-air-> main     <-CRSF,serial-> flight
               tx module         receiver                  tx module         receiver                 controller

The relay mode is enabled by setting Rx_Out_Mode = "crsf tx" in the secondary receiver. This has a chain
of consequences in the operation of the secondary receiver and the two tx modules.

Secondary receiver:

Relay mode is enabled by Rx_Out_Mode = "crsf tx".

- OUT port is configured to use tx inverted, and 400k baud
- rc data is send with address CRSF_ADDRESS_RECEIVER (instead of CRSF_ADDRESS_BROADCAST)
- rc data is adjusted by the channel order, but no other modifications are done (like failsafe, rssi, etc)
- no other CRSF frames are send (like link statistics)
- injection of mavlink frames into the outgoing stream (serial tx) is disabled
  (no flow control via RADIO_STATUS, no RC_CHANNELS_OVERRIDE, etc)
- mavlink mavftp fakery is disabled

Main Tx Module:

It determines its relay mode from the address of the received CRSF frames. When it receives a CRSF frame
with address CRSF_ADDRESS_BROADCAST it assumes business as usual. If it receives a CRSF frame with address
CRSF_ADDRESS_RECEIVER it however determines that it is the main tx module in a relay setup.

- function crsf.IsRelayMain() returns true
- all emission of CRSF frames is disabled. This implies that the IN pin is never becoming Tx and remains all
  time in Rx configuration. This is to ensure that there are no electrical shorts with the receiver's Tx output.
- any received CRSF frame except of the rc date (CRSF_FRAME_ID_CHANNELS) are ignored
- handling of telemetry mavlink messages is disabled, i.e., the mavlink stream is not processed and CRSF telemetry
  data are not extracted (reduced cpu load)
- the mavlink message MLRS_MAIN_RADIO_STATS is injected into the outgoing mavlink stream, with 10 Hz rate
  (it has a size of 24 bytes, so this additional data increases data rate by 10*24 = 240 bytes/sec)

Secondary Tx Module:

It determines its relay mode as follows: When it receives MLRS_MAIN_RADIO_STATS mavlink messages it determines
that it is the secondary tx module in a relay module, otherwise it assumes business as usual.

- function crsf.IsRelaySecondary() returns true
- the CRSF link statistic frames (CRSF_FRAME_ID_LINK_STATISTICS, CRSF_FRAME_ID_LINK_STATISTICS_TX,
  CRSF_FRAME_ID_LINK_STATISTICS_RX) send the info obtained from the MLRS_MAIN_RADIO_STATS mavlink
  messages (instead of that of the secondary tx module and receiver pair).
- if enabled, the RADIO_STATUS mavlink message is emitted with sysid and compid of 51, 69 (instead of
  51, 68), to distinguish them.

Inner Tx Modules:

In a chain of three or more pairs, the inner tx modules will be both in state crsf.IsRelayMain() == true
and crsf.IsRelaySecondary() == true. This can be used to identify them as inner tx modules.

- emission of MLRS_MAIN_RADIO_STATS messages is disabled. This ensures that the secondary tx module connected to
  the radio receives the correct link statistics, namely that of the main tx module.























*/


