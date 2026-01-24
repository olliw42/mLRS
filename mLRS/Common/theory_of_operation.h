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
  Flags for various operation modes
  Config.ReceiveUseAntenna1, Config.ReceiveUseAntenna2
  Config.TransmitUseAntenna1, Config.TransmitUseAntenna2
  Config.IsDualBand
  will be abbreviated as: Ra1, Ra2, Ta1, Ta2, isDB
============================================================

hardware    |  Ra1 |  Ra2 | Ta1 | Ta2 | isDB | operation mode, and comments
-------------------------------------------------------------------------------
SX1         |   x      -     x     -     -   | band 1, diversity = a1 (forced)
-------------------------------------------------------------------------------
SX1,SX2     |   x      -     x     -     -   | band 1, diversity = a1
            |   -      x     -     x     -   | band 1, diversity = a2
            |   x      x     x     x     -   | band 1, diversity = enabled
                 <- -> (either or)
            |   x      -     x     x     -   | band 1, diversity = r.enabled ta1
            |   -      x     x     x     -   | band 1, diversity = r.enabled ta2
-------------------------------------------------------------------------------
SX1,SX'2    |   x      -     x     -     -   | band 1, diversity = a1 (forced)
            |   -      x     -     x     -   | band 2, diversity = a2 (forced)
            |   x      x     x     x     x   | dual band, band 1 & band 2, diversity = enabled (forced)
-------------------------------------------------------------------------------
LR1         |   x      -     x     -     -   | band 1, diversity = a1
-------------------------------------------------------------------------------
LR1,LR'2    |   x      -     x     -     -   | band 1, diversity = a1
            |   -      x     -     x     -   | band 1, diversity = a2
            |   x      x     x     x     -   | band 1, diversity = enabled
                 <- -> (either or)
            |   x      -     x     x     -   | band 1, diversity = r.enabled ta1
            |   -      x     x     x     -   | band 1, diversity = r.enabled ta2
            |   x      x     x     x     x   | dual band, band 1 & band 2, diversity = enabled (forced)


Note: The cases diversity and dual band are thus distinguished by the isDB flag.

For single band fhss1 and fhss2 need to be set up identically.



============================================================
  Frequency index in Tx OTA packets
  fhss_index, fhss_index_band
============================================================

For the Tx->Rx direction, the frequency on which a ota frame is send needs to be transmitted to the receiver
alongside with that frame.

This is done by the fhss_index, fhss_index_band fields in the header of the Tx frame, see tTxFrameStatus. These
re-purpose hitherto unused bits in the Tx frame header.

Note: The implementation in the receiver code is a bit dirty. The receiver wants to see both bands, even if it
is only a single band device. The tx module must in this case send identical fhss indices for both band slots,
since otherwise the receiver would jump to wrong frequencies. It would connect, but send at wrong frequencies,
and the tx module would not connect.

There are two reasons for the need of this mechanism:

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
band what the transmitter's frequency is on the other band, and all would be good. However, this is not the situation.
Especially the 915 band's fhss sequence length is incommensurate to those of all other bands, i.e., its 25 hops
cannot easily be divided by the other fhss sequence lengths. Therefore, the result can be that the receiver connects
to the packets on one band, but cannot receive the packets on the other bands even when it gets closer because it is
always looking at the wrong frequency. The advantage of dual band would be lost and the system would essentially fall
back to single band operation.

This can be avoided by this mechanism: When the receiver receives a packet on one band, it sets the fhss of the
other band to the frequency which the transmitter is currently using. The receiver gets this information from the
fhss info transmitted with each frame.

Ideally, in each frame the frequency for both bands would be transmitted. This however would have required a change
to the frame header, and break backwards compatibility. Hence, the two frequencies are send alternately, which is
what the variable fhss_index_band is for (can be 0/1). Gladly, the used bits were set to UINT8_MAX in older version,
which allows us to check if it's a "old" or "new" frame, i.e., a frame carrying valid fhss information or not.





*/


