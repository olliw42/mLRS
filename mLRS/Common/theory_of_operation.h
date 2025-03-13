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

For the Tx->Rx direction, the frequency on which a ota frame is send needs to be transmitted alongside with that packet.

This is done by the fhss_index, fhss_index_band fields in the header of the Tx frame, see tTxFrameStatus. These
re-purpose hitherto unused bits in the Tx frame header.


There are two reasons for this:

1. Single band systems (including diversity systems)
----------------------------------------------------

In close range and strong power it can happen that the receiver successfully receives a frame from the Tx module
even though it is not adjusted to the correct frequency. The fhss information in the frame allows the receiver to
check if it has received that frame while being on that frequency, or not. In the latter case it wants to discard
that frame. This avoids that the receiver synchronizes incorrectly with and connects to the transmitter even though
it is not correctly synchronized to the fhss sequence.
This error mode typically shows up as a connection with a low LQ. This can be dangerous because the user may think
to have a good connection but in fact the connection is bad (and will stay bad when flying).
This check needs to be only done in the synchronization phase (i.e. when the receiver wants to connect) since once
it is correctly synchronized it will stay synchronized (or disconnects).
In the single band case the mechanism really just helps with avoiding wrong connection in closer range/high power.

2. Dual band systems
--------------------

Here the mechanism is crucial for establishing a proper connection, also in far range when the receiver has
disconnected and tries to reconnect.

Let's imagine the vehicle is at far distance, such that at this distance only the frames of one of the bands are
received. The receiver will see these frames, and synchronize its hoping sequence to these frames. If the hopping
sequences on both bands would be the same, the receiver could determine from receiving the frames on one band what
the transmitter's frequency on the other band is, and all would be good. However, this is not the situation.
Especially the 915 band's fhss sequence length is incommensurate to those of all other bands, i.e., its 25 hops
can't easily be divided by the other fhss sequence lengths. Therefore, the result can be that the receiver connects
to the packets on one band, but cannot receiver the packets on the other bands. The advantage of dual band would
be lost and the system would fall back to single band.
This can be avoided by this mechanism: When the receiver receives a packet on one band, it sets the fhss of the
other band to the frequency which the transmitter is currently using. This information the receiver gets from
fhss info transmitted with each frame.

Ideally, in each frame the frequency for both bands would be transmitted. This however would have required a change
to the frame header, and break backwards compatibility. Hence, the two frequencies are send alternately, which is
what the variable fhss_index_band is for. Gladly, the used bits were set to UINT8_MAX in older version, which
allows us to check if it's a "old" or "new" frame, i.e., a frame carying valid fhss information or not.





*/


