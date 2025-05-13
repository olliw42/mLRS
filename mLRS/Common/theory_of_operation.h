//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Theory of Operation
//*******************************************************
// Notes on the operation and functioning of certain features and mechanisms.
/*


============================================================
  Frequency index in Tx OTA packets
  fhss_index, fhss_index_band
============================================================

For the Tx->Rx direction, the frequency on which a ota frame is send needs to be transmitted alongside with that
frame.

This is done by the fhss_index, fhss_index_band fields in the header of the Tx frame, see tTxFrameStatus. These
re-purpose hitherto unused bits in the Tx frame header.


There are two reasons for this:

1. Single band systems (including diversity systems)
----------------------------------------------------

In close range and strong power it can happen that the receiver successfully receives a frame from the Tx module
even though it is not adjusted to the correct frequency. The fhss information in the frame allows the receiver to
check if it has received that frame while being on that frequency, or not. In the latter case it wants to discard
that frame. This avoids that the receiver synchronizes incorrectly with the transmitter and connects to it even
though it is not correctly synchronized to the fhss sequence.
This error mode typically shows up as a connection with a low LQ. This can be dangerous because the user may think
to have a good connection and starts flying but in fact the connection is bad (and will stay bad when flying).
This check needs to be only done in the synchronization phase (i.e. when the receiver wants to connect) since once
it is correctly synchronized it will stay synchronized (or disconnects).
In the single band case the mechanism really just helps with avoiding false connection in closer range/high power.

2. Dual band systems
--------------------

For dual band systems the mechanism is crucial for establishing a proper connection, also and especially in far
range when the receiver has disconnected and tries to reconnect.

Let's imagine the vehicle is at far distance, such that at this distance only the frames of one of the bands are
received. The receiver will see these frames, and synchronize its hoping sequence to these frames. If the hopping
sequence lengths on both bands would be the same, the receiver could determine from receiving the frames on one
band what the transmitter's frequency on the other band is, and all would be good. However, this is not the situation.
Especially the 915 band's fhss sequence length is incommensurate to those of all other bands, i.e., its 25 hops
cannot easily be divided by the other fhss sequence lengths. Therefore, the result can be that the receiver connects
to the packets on one band, but cannot receive the packets on the other bands even when it gets closer because it is
always looking at the wrong frequency. The advantage of dual band would be lost and the system would essentially fall
back to single band operation.

This can be avoided by this mechanism: When the receiver receives a packet on one band, it sets the fhss of the
other band to the frequency which the transmitter is currently using. This information the receiver gets from the
fhss info transmitted with each frame.

Ideally, in each frame the frequency for both bands would be transmitted. This however would have required a change
to the frame header, and break backwards compatibility. Hence, the two frequencies are send alternately, which is
what the variable fhss_index_band is for (can be 0/1). Gladly, the used bits were set to UINT8_MAX in older version,
which allows us to check if it's a "old" or "new" frame, i.e., a frame carrying valid fhss information or not.



==============================
Relay Mode
==============================

Two pairs of tx module and receiver are daisy chained. It may be more than two pairs, but let's stick
to only two for the sake of the discussion.

For notation:

radio <-CRSF-> secondary <- air -> secondary <-CRSF,serial-> main      <- air -> main     <-CRSF,serial-> flight
               tx module           receiver                  tx module           receiver                 controller

The relay mode is enabled by setting Rx_Out_Mode = "crsf tx" in the secondary receiver. This has a chain
of consequences in the operation of the secondary receiver and the two tx modules.

Secondary Receiver:

Relay mode is enabled by Rx_Out_Mode = "crsf tx".

- OUT port is configured to use tx inverted, and 400k baud
- rc data is send with address CRSF_ADDRESS_RECEIVER (instead of CRSF_ADDRESS_FLIGHT_CONTROLLER)
- rc data is adjusted by the channel order, but no other modifications are done (like failsafe, rssi, etc)
- no other CRSF frames are send (like link statistics)
- injection of mavlink frames into the outgoing stream (serial tx) is disabled
  (no flow control via RADIO_STATUS, no RC_CHANNELS_OVERRIDE, etc)
- mavlink mavftp fakery is disabled

Main Tx Module:

It determines its relay mode from the address of the received CRSF frames. When it receives a CRSF frame
with address CRSF_ADDRESS_FLIGHT_CONTROLLER (or CRSF_ADDRESS_BROADCAST) it assumes business as usual. If
it receives a CRSF frame with address CRSF_ADDRESS_RECEIVER it however determines that it is the main tx module
in a relay setup.

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


