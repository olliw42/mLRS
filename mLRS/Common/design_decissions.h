//*******************************************************
// MLRS project
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// bla bla
//*******************************************************
#ifndef BLABLA_H
#define BLABLA_H
#pragma once

//-------------------------------------------------------
// this is used as notepad to write down some things
//-------------------------------------------------------
/*

TODOs:
- sync & configure at first connect
- OTA
- frame loss, resend last one time to make it more robust
- FHSS
- LQ: we have seq_no, connect
- OTX & LQ: stats feed back to otx
- RADIO_STATUS
- crc
- DOI isr handling
- RX side sbus/sbus-fast/sbus-inverted ppm?  (diversity possible on ArduPilot?)
- TX side sport, to get rc channel data
- TX side jrbay bidirectional mavlink
- configuration settings
- bind phrase
- transform mavlink data into more robust & slim format, introduce cmpr_msg
- USB on TX side
- RX side rssi output
- RX side failsafe behaviors
- RX side antenna diversity ????? really? I guess I rather like a atto duo


-------------------------------------------------------
desired configurations

Tx:

a) stand-alone telemetry unit (like SiK, and so on)

usb/uart, uart with RTS/CTS
usb for config, upgrade
possibly uart BT module

b) standard JR bay module

rc channels via JR-Pin1
uart for mavlink
uart for BT module
possibly mimic telemetry via JR-Pin5
usb for config, upgrade
possibly LED for config

c) mBridge JR bay

rc channels & mavlink via Pin5
uart for mavlink
uart for BT module
usb for config, upgrade
possibly LED for config

Rx:

always the same
SBus output
Rx/Tx serial, possibly with RTS/CTS
Rssi output


-------------------------------------------------------
some experimental results

25 ms
rx = 16,1 ms  tx = 7.80 ms
20 ms
rx = 11.1 ms, tx = 7.80 ms
estimated time over air is 7.8128 ms

a 2nd timing check for pll:

treceive_done - treceive    = ca 11.6 ms
ttransmit - treceive        = ca 11.7 ms
ttransmit_done - treceive   = ca 19.9 ms
tplltick - treceive         = ca 11.6 ms
tdopostreceive - treceive   = ca 12.6 ms with 100 shift
tpllupdate - treceive       = ca 16.6 ms with 500 shift


experimental finding
on receiver:
amptx   amprx     rssi on transmitter   rx_rssi on transmitter
high    low       -22                   -18
high    high      -22                   -54     => bad reception of receiver
low     low       -83                   -19     => bad transmission of receiver
low     high      -83                   -54     => both

on transmitter:
amptx   amprx     rssi on transmitter   rx_rssi on transmitter
high    low       -21                   -17
high    high      -63                   -18
low     low       -22                   -78
low     high      -63                   -78


-------------------------------------------------------
2.4 GHz band

various sources
2.3995 - 2.4845
2.401 - 2.483 (- 2.473 in USA)
2.400 - 2.4835
2.4 - 2.48

ELRS 2400.4 - 0.4 ... 2479.4 + 0.4  = 2400.0 ... 2479.8

https://www.bundesnetzagentur.de Allgemeinzuteilungen/MobilfunkDectWlanCBFunk/2013_10_WLAN_2,4GHz_pdf:
2400,0 – 2483,5

so use 2406.0 ... 2473.0  in 1MHz steps


-------------------------------------------------------
Modules

Ebyte E28-2G4M27S
27 dBm / 501 mW

E28-2G4T12S   -> TTL interface ????
12.5 dBm / 18 mW

E28-2G4M12S
12.5 dBm / 18 mW

E28-2G4M20S
20 dBm / 100 mW


-------------------------------------------------------
Amps

SE2431L (siyi FM30):
3.6V, Pin = 3 dBm => Pout = 24.0 dBm (251mW)
3.3V, Pin = 0 dBm => Pout = 22.5 dBm (178mW)
signal gain typical 22 dBm (19-25 dBm)
Pin max = 6 dBm

=> do not feed it with more that 6 dBm !!
0 dBm should be a good choice

-10 dBm = 0.1 mW in -> ca 12 dBm / 16 mW out
0 dBm = 1 mW in -> ca 22 dBm / 158 mW out


SX1280 max output power = 12.5 dBm = 17.783 W
SiGe SE2431L
max output power = 24 dBm , 22.5 dBm at 3.3V = 177.8 mW
gain is 22 dBm
max input power is 6 dBm
=> do not use more than 6 dBm
   1 dBm should be way OK


-------------------------------------------------------
ImmersionRc Ghost
-------------------------------------------------------
2406 - 2479 MHz

Solid250    -105dBm   250Hz
Race500     -105dBm   500Hz
Race250     -105dBm   250Hz
Pure Race   -106dBm   250Hz  or 300Hz ??    11.5 km
Race        -106dBm   166Hz
Normal      -112dBm    55Hz                 23.0 km
Long Range  -117dBm    15Hz                 40.0 km

222.22 Hz

11 / 1600 = -117 dBm = 3.88 kbps, 6 bytes = 29.3 ms => NO
9 / 800   = -117 dBm = 6.34 kbps, 26 bytes = 29.8 ms => 3.9kbps  THIS IS IT ??
8 / 400   = -116 dBm = 5.64 kbps, 23 bytes = 28.8 ms
8 / 200   = -118 dBm = 2.82 kbps, 4 bytes = 29.3 ms => NO

7 / 800   = -112 dBm = 19.74 kbps, 31 bytes = 9.00 ms => 15.5kbps
the other bandwidths are worse

Tx module:
stm32f303cc  48p  72MHz  256k  40k
SE2431L ?? really? or SE2622L

Atto:
stm32f301k8  72MHz  64K  16K


-------------------------------------------------------
Siyi FM30
-------------------------------------------------------

Tx module:
stm32f103c8  48p  72MHz  64k  16k
SE2431L

Rx module:
stm32f373cc  48p  72MHz  256k  32k
SE2431L


-------------------------------------------------------
one needs to decide between small frames at higher rate or fat frames at lower rate
with some retransmission capability slim frames would have an advantage
maybe they would do generally
but to start with, let's go with fat frames
it should not be very difficult to go to slim frames later

a benefit of slim frames would be faster reconnection after connection loss
this could be potentially extremely crucial !!!
50 Hz = 20 ms & 64 frequencies => up to 1.3 secs to connect !!
200 Hz = 5 ms & 64 frequencies => up to 0.32 secs to connect
250 Hz = 4 ms & 64 frequencies => up to 0.26 secs to connect
we could reduce the number of frequencies for the connection state, e.g. just 16
50 Hz = 20 ms & 16 frequencies => up to 0.32 secs to connect

slim frames my also allow us to detected more quickly if the connection is lost
=> reconnection in case of connection loss might be much faster
DO not confuse with micro failsafes

experimentally I find:
on rx many packets are received, a good fraction of them pass crc1 test,
ergo, it seems that in long packets it is more likely that the early bytes are ok than the latter
similarly, on tx many quite more packets are received than do pass valid test
=> I think smaller packets have the benefit that more of those received will be valid

is it possible to improve that by using the frequency error correction ???


what LORA settings could be useful?

it appears that BW 800kHz provides the best tradeoff between high data rate and high receive sensitivity
so let's go with 800kHz

the highest rate setting would then be

5 / 800   = -105 dBm = 56.39 kbps, 105 bytes = 9.97 ms => 52.5kbps   this could be siyi FM30, doesn't it ??


so, the frame should have the channels, the payload, and additional info and crcs

channels idea 1:
12 ch @ 11 bits = 132 bits
 6 ch @  7 bits =  42 bits
 2 ch @  1 bits =   2 bits
                 -----------
                  176 bits = 22 bytes

channels idea 2:
 4 ch @ 11 bits =  44 bits
10 ch @  8 bits =  80 bits
 4 ch @  1 bits =   4 bits
                 -----------
                  128 bits = 16 bytes

channels idea 3:
 6 ch @ 11 bits =  66 bits
 8 ch @  7 bits =  56 bits
 2 ch @  2 bits =   4 bits
 2 ch @  1 bits =   2 bits
                 -----------
                  128 bits = 16 bytes


auxiliary:
- sequence no (few bits are sufficient to check for lost packets), e.g. 4 = 16 frames = 0.8 sec
- receive confirmation flag (to resend if missed), 1 bit
- packet type, e.g. 4 bits
- rssi (doesn't have to be 8 bit, 7 or 6 would be also ok)
- LQ (doesn't have to be 8 bit, 7 or 6 would be also ok)
- sync word
- payload len (doesn't have to be 8 bit, 7 bit should be OK)

example 1:

1:    sync byte 1
2:    sync byte 2
3:    status (seq. no, ack flag, packet type)
4:    rssi
5:    LQ
6:    payload len
7:-
|     6 bytes of RC data  (4 ch @ 11 bits & 4 ch @ bit)
12:-
13:   crc1
14:-
|     10 bytes of remaining RC data (10 ch @ 8 bits)
23:-
24:   crc2
25:-
|     64 bytes of payload
88:-
89:   crc3
90:   crc4

=> 90 bytes/frame = 7.81 ms air time
=> 3200 B/s @ 50 Hz

1:    sync byte 1
2:    sync byte 2
3:    status (seq. no, ack flag, packet type)
4:    rssi
5:    LQ
6:    payload len
7:-
|     82 bytes of payload
88:-
89:   crc3
90:   crc4

=> 4100 B/s @ 50 Hz

longer downlink frame?
e.g.

1:    sync byte 1
2:    sync byte 2
3:    status (seq. no, ack flag, packet type)
4:    rssi
5:    LQ
6:    payload len
7:-
|     100 bytes of payload
106:-
107:   crc3
108:   crc4

=> 108 bytes/frame = 9.23 ms air time
=> 5000 B/s @ 50 Hz


=< let's start with 90 bytes/frame


*/
#endif // BLABLA_H



