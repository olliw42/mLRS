//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
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
failsafe modes
1) no out signal
2) always out signal, but with values set to pre-defined failsafe values
3) always out signal, but with values set to defaults (such as thr = 800, r,p,y = center, ...)
4) always out signal, but with values which makes the vehicle to hover


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

slim frames will however not allow sufficiently large serial data rates, because of the overhead

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

the "nice" thing of idea 2 is that it fits a crc1 idea


auxiliary:
- sync word
- sequence no (few bits are sufficient to check for lost packets), e.g. 4 = 16 frames = 0.8 sec
- receive confirmation flag (to resend if missed), 1 bit
- packet type, e.g. 4 bits
- rssi (doesn't have to be 8 bit, 7 or 6 would be also ok)
- LQ (doesn't have to be 8 bit, 7 or 6 would be also ok)
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
13:   crc1a
14:   crc1b
16:-
|     10 bytes of remaining RC data (10 ch @ 8 bits)
24:-
25:-
|     64 bytes of payload
88:-
89:   crca
90:   crcb

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
89:   crca
90:   crcb

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
107:   crca
108:   crcb

=> 108 bytes/frame = 9.23 ms air time
=> 5000 B/s @ 50 Hz


=< let's start with 90 bytes/frame

13.01.2022
the crc1 technique seems to really give an advantage
so for rx->tx we want two LQ's to distinguish
hence frame stats changed, and packets are now 91 bytes

-------------------------------------------------------
channel ideas, which fit the 6 bytes + crc1 + 10 bytes structure

6 bytes section is always
 4 ch @ 11 bits =  44 bits
 4 ch @  1 bits =   4 bits

10 bytes section:

A)
10 ch @  8 bits =  80 bits

B1)
4 ch @  11 bits =  44 bits
6 ch @   6 bits =  36 bits

B2)
4 ch @  11 bits =  44 bits
4 ch @   7 bits =  28 bits
2 ch @   4 bits =   8 bits

B3)
4 ch @  11 bits =  44 bits
4 ch @   8 bits =  32 bits
2 ch @   2 bits =   4 bits

C1)
2 ch @  11 bits =  22 bits
8 ch @   7 bits =  56 bits
                   => 2 bits left over

C2)
2 ch @  11 bits =  22 bits
6 ch @   8 bits =  48 bits
2 ch @   5 bits =  10 bits

D1)
4 ch @  11 bits =  44 bits
4 ch @   9 bits =  36 bits half-rate, use a bit from 6 byte section to indicate which it is


-------------------------------------------------------
Crossfire
https://www.g3gg0.de/wordpress/fpv/fpv-analysis-of-tbs-crossfire/
thx to V-22

tx->rx  23 bytes payload
8 x 10 bit channels = 10 bytes
9 bytes telemetry
2 bytes crc

rx->tx  13 bytes payload
9 bytes telemetry
1 byte crc

150 Hz = 6.66 ms
returned packet after 2.6 ms

=> 9 bytes @ 150 Hz = 1350 bytes/sec max

HOW does that fit into LoRa?
sx1262, SF5, 500kHz, CR4/5, 12 bytes preamble, no crc:
23 bytes -> 4.56 ms
13 bytes -> 3.28 ms
---
= 7.84 ms
=> 127 Hz max => no way it fits into 150 Hz !!!!
indeed, g3gg0 link says it uses FSK

cannot be done with sx1276, since it only has SF6 min

for configuring ArduPilot to use CRSF rc input, see e.g.
http://www.mateksys.com/?portfolio=h743-slim#tab-id-5
it's for the MatekH743, but it nicely makes clear how things work
also see https://ardupilot.org/copter/docs/common-tbs-rc.html#common-tbs-rc
e.g set RSSI_TYPE = 3


-------------------------------------------------------
Qczek LRS
https://qczek.beyondrc.com/qczek-lrs-433mhz-1w-lora-rc-link/qczek-lrs-technical-specyfication/

LORA SX1278 433MHz
250 kHz SF7 CR4/8  -121 dBm  24ms   hmm I calculate 30 ms ??
500 kHz race mode

fhss with 8 frequencies

tx->rx:
20 bytes
9 x 11 bit channels
4 bytes telemetry
16bit crc

rx->tx
11 bytes telemetry


-------------------------------------------------------
long range mode, sx1276
go from 50 Hz to 20 Hz

50 Hz:
800kHz, SF5, LI4/5 -105dBm  91 bytes payload => 7.9 ms  => 2.1 ms headroom

800kHz, SF7, LI4/5 -112dBm  91 bytes payload => 23.5 ms
=> 20 Hz with 1.5 ms headroom  but we could shave some bytes payload to get to 1/3 serial byte rate
                               -> 88 bytes payload => 22.7 ms => 2.1 ms headroom
=> 19 Hz with 2.8 ms headroom
=> 17 Hz with 5.9 ms headroom

sx1276
500kHz, SF6, CR4/5  -112dBm  91 payload => 21.6 ms  @ 865MHz and 433 MHz
=> 20 Hz with 2.8 ms headroom


50 Hz & 91 bytes payload
=> byte rates  tx->rx: 50 * 64 = 3200 bytes/sec
               rx->tx: 50 * 82 = 4100 bytes/sec

20 Hz & 91 bytes payload
=> byte rates  tx->rx: 20 * 64 = 1280 bytes/sec
               rx->tx: 20 * 82 = 1640 bytes/sec


full channels & telemetry only mode ??


-------------------------------------------------------
sx1280, sx1276, sx1262

sx1280:  800kHz, SF5, LI4/5, 12: -105 dBm, 91 bytes payload =>  7.9 ms    -> 20 ms (50 Hz)
sx1280:  800kHz, SF6, LI4/5, 12: -108 dBm, 91 bytes payload => 13.4 ms    -> 32 ms (31 Hz)
sx1280:  800kHz, SF7, LI4/5, 12: -112 dBm, 91 bytes payload => 23.5 ms    -> 53 ms (19 Hz)

sx1276:  500kHz, SF6, CR4/5, 12: -112 dBm, 91 bytes payload => 22.3 ms    -> 53 ms (19 Hz)

sx1262:  500kHz, SF5, CR4/5, 12: -111 dBm, 91 bytes payload => 13.2 ms    -> 32 ms (31 Hz)
sx1262:  500kHz, SF6, CR4/5, 12: -112 dBm, 91 bytes payload => 22.6 ms    -> 53 ms (19 Hz)


-------------------------------------------------------
FrSky
https://openrcforums.com/forum/viewtopic.php?t=7080
says 140- 160 bytes/sec max in single direction
     90 bytes/sec max in dual direction
https://www.frsky-rc.com/wp-content/uploads/2017/07/Manual/FRSKY%20TELEMETRY%20PROTOCOL.PDF
120 bytes/2 max
what's the truth??


-------------------------------------------------------
ArduPilot

rssi goes from 0 - 255

crsf rssi handling:
AP converts it as follows into its own rssi value
LINK_STATISTICS:
50 - 120 -> 255 - 0
https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp#L483-L510
Note: in 4.2 it also will handle LINK_STATISTICS_RX and LINK_STATISTICS_TX,
and each will overwrite any previous rssi, so be careful with what one really wants to do


-------------------------------------------------------
RC ranges

mLRS:
11 bit
1 ... 1024 ... 2047 for range +-120%

so:   1 ... 172 ... 1024 .. 1876 ... 2047
    -120%  -100%     0%    +100%    +120%

100% = 852 span
120% = 1023 span

sbus/crsf:
11 bit
+-100% = 173 ... 992 .. 1811

so:   9 ... 173 ... 992 .. 1811 ... 1965
    -120%  -100%    0%    +100%    +120%

100% = 819 span
120% = 983 span

OpenTx produces 173 ... 992 ... 1811 for -100% ... 100%

=> mlrs = (sbus - 992) * 1023 / 983 * 1024
let's use
   mlrs = (sbus - 992) * 2047 / 1966 * 1024


old 0..2047 = +-100% scaling:
 rc data in TxFrame is centered such to cover 0..2047, 0..255, 0..1
 ardupilot: sbus 200 -> 1000us, sbus 1800 -> 2000us
 txFrame:   ch0-ch3:    0 .. 1024 .. 2047, 11 bits
            ch4-ch13:   0 .. 128 .. 255, 8 bits
            ch14-ch17:  0..1, 1 bit

 let's convert the full range to +-100% or 200..1000..1800:
 sbus =  ch * 1600 / 2048 + 200 for 11 bits
         ch * 1600 / 256 + 200 for 8 bits

 we also could convert to OpenTx range +-100% = 988us ... 2012 us
 sbus =  ch * 1600 / 2048 + 200 for 11 bits
         ch * 1600 / 256 + 200 for 8 bits

 Ardupilot:
 translates sbus values 200..1000..1800 into pwm values 1000..1500..2000 us
 => pwm = sbus * 500 / 800 + 875
    sbus = (pwm - 875) * 800 / 500 = pwm * 800 / 500 - 1400
 thus, if we scale sbus to 200..1800 we get 1000..2000 us on ardupilot

 when selecting SBUS external module and connecting JRpin1 to ArduPilot, we get
 983 ... 1495 .. 2006

 opentx: sbus value = ch value * 8 / 10 + 992, where ch value = -1024...1023
 => sbus values = 173..992..1811


-------------------------------------------------------
SX1280 power
the sx power is calculated as
  sx_power = LIMIT(SX1280_POWER_m18_DBM, power - POWER_GAIN_DBM + 18, POWER_SX1280_MAX_DBM)

 example 1: no PA
  POWER_GAIN_DBM = 0
  POWER_SX1280_MAX_DBM = SX1280_POWER_12p5_DBM
  => power = -18 ... 13
  => sx_power = 0 ... 31 = SX1280_POWER_m18_DBM ... SX1280_POWER_12p5_DBM

 example 2: E28 PA 27 dBm gain
  POWER_GAIN_DBM = 27
  POWER_SX1280_MAX_DBM = SX1280_POWER_0_DBM
  => power = 9 ... 27
  => sx_power = 0 ... 18 = SX1280_POWER_m18_DBM ... SX1280_POWER_0_DBM

 example 2: siyi PA 22 dBm gain
  POWER_GAIN_DBM = 22
  POWER_SX1280_MAX_DBM = SX1280_POWER_3_DBM
  => power = 4 ... 25
  => sx_power = 0 ... 21 = SX1280_POWER_m18_DBM ... SX1280_POWER_3_DBM


-------------------------------------------------------
stuff

https://interrupt.memfault.com/blog/cortex-m-fault-debug


-------------------------------------------------------
some experimental results
-------------------------------------------------------

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


timing measures 11.02.2022
rx diy-f103, times in 10us, on tx side RX tmo is 10
CLOCK_SHIFT_10US 75 = 750us, 9MHz spi, diversity 1

: 25952, 02010; 00007, 00016, t 00028, 00067, 00857, r 00857, 00893, 01914,
: 27956, 02004; 00007, 00016, t 00028, 00067, 00857, r 00857, 00893, 01908,

: 41841, 02007; 00007, 00016, t 00029, 00067, 00857, r 00857, 00877, 01910, d 00163,
: 43848, 02007; 00007, 00016, t 00029, 00070, 00860, r 00860, 00879, 01911, d 00167,
: 45856, 02008; 00007, 00016, t 00029, 00075, 00864, r 00864, 00884, 01912, d 00171,
: 47861, 02005; 00007, 00016, t 00029, 00068, 00857, r 00857, 00877, 01908, d 00164,
dopostreceive is t0 point
02008 = 20208 us -> cycle time
00007 = 70 us  -> time to set TX
00016 = 160 us -> time to finish  -> could make sense to have a 2nd time trigger
t
00029 = 290 us -> tx entered
00070 = 700 us -> tx2 finished  => ca 400 us to get data, transfer frame to SX, and start transmitting
00860 = 8600 us -> txdone  => 7.9 ms for frame, consistent with toa 7.892 ms
r
00860 = 8600 us -> rx entered
00879 = 8790 us -> rx2 finished  => 200 us to start receiving
01911 = 19110 us -> rxdone  => 10.2 ms until it has received frame   => THIS IS TIGHT, just 0.9 ms ROOM!
d
00167 = 1670 us -> = tx2 - rxdone, time between rxdone and tx2

CLOCK_SHIFT_10US 75 = 750us, 2p25MHz spi, diversity 1
=> doesn't work, no connection !! => TX locks up, green on, red blink = SX1280_IRQ_RX_DONE in wrong state !!!
without uart debug it is just at the edge, tx locks up somewhat later

=> if time from end of receive to begin of transmit becomes too large, tx locks up, makes sense
=> time between rxdone and tx2 needs to be short enough

CLOCK_SHIFT_10US 75 = 750us, 4p5MHz spi, diversity 1
: 45780, 02007; 00007, 00016, t 00029, 00075, 00865, r 00865, 00886, 01903, d 00179,
: 47788, 02008; 00007, 00016, t 00030, 00081, 00871, r 00871, 00891, 01903, d 00185,
CLOCK_SHIFT_10US 75 = 750us, 9MHz spi, diversity 1
: 24677, 02007; 00007, 00016, t 00029, 00071, 00860, r 00861, 00880, 01911, d 00168,
: 26683, 02006; 00007, 00016, t 00029, 00069, 00858, r 00858, 00878, 01910, d 00165,
CLOCK_SHIFT_10US 75 = 750us, 18MHz spi, diversity 1
: 08261, 02007; 00007, 00016, t 00029, 00063, 00852, r 00852, 00871, 01914, d 00155,
: 10267, 02006; 00007, 00016, t 00029, 00065, 00854, r 00854, 00873, 01914, d 00158,

=> time between rxdone and tx2 gets shorter with larger SPI rate

CLOCK_SHIFT_10US 50 = 500us, 2p25MHz spi, diversity 1
: 59169, 02004; 00007, 00016, t 00030, 00093, 00884, r 00884, 00907, 01908, d 00188,
: 61175, 02006; 00007, 00016, t 00029, 00093, 00884, r 00884, 00906, 01911, d 00189,
CLOCK_SHIFT_10US 50 = 500us, 4p5MHz spi, diversity 1
: 63101, 02007; 00007, 00016, t 00029, 00086, 00876, r 00876, 00897, 01928, d 00165,
: 65106, 02005; 00007, 00016, t 00029, 00078, 00868, r 00868, 00889, 01925, d 00157,
CLOCK_SHIFT_10US 50 = 500us, 9MHz spi, diversity 1
: 20744, 02006; 00007, 00016, t 00029, 00075, 00864, r 00864, 00884, 01935, d 00146,
: 22749, 02005; 00007, 00016, t 00029, 00067, 00857, r 00857, 00877, 01934, d 00138,
CLOCK_SHIFT_10US 50 = 500us, 18MHz spi, diversity 1
: 43739, 02010; 00007, 00016, t 00029, 00073, 00862, r 00862, 00881, 01943, d 00140,
: 45743, 02004; 00007, 00016, t 00030, 00064, 00852, r 00853, 00872, 01937, d 00131,
=> ca. 500us gain for tx2-rxdone with higher SPI

=> time between rxdone and tx2 gets shorter with shorter CLOCK_SHIFT

CLOCK_SHIFT_10US 75 = 750us, 2p25MHz spi, diversity 1
tx side RX tmo is 10 => tx locks up, SX1280_IRQ_RX_DONE in wrong state
tx side RX tmo is 11 => works
CLOCK_SHIFT_10US 100 = 1000us, 2p25MHz spi, diversity 1
tx side RX tmo is 11 => works
CLOCK_SHIFT_10US 150 = 1500us, 2p25MHz spi, diversity 1
tx side RX tmo is 11 => works
CLOCK_SHIFT_10US 200 = 2000us, 2p25MHz spi, diversity 1
tx side RX tmo is 11 => no tx lock up, rec connects but tx does not connect

=> settle on CLOCK_SHIFT_10US 100 (as before), 9MHz spi, and tx side RX tmo 11

=> it could be useful to add 2nd doPostPostReceive or other means to split doPostReceives

with it:
& tx side RX tmo 11
CLOCK_SHIFT_10US 100 = 1000us, 9MHz spi, diversity 1
: 15695, 02006; 00007, 00007, t 00020, 00057, 00846, r 00846, 00866, 01885, d 00178,
: 17705, 02010; 00007, 00007, t 00020, 00061, 00851, r 00851, 00871, 01889, d 00182,
: 19709, 02004; 00007, 00007, t 00020, 00057, 00846, r 00846, 00866, 01883, d 00178,
& tx side RX tmo 10
CLOCK_SHIFT_10US 75 = 750us, 2p25MHz spi, diversity 1
: 33745, 02007; 00007, 00007, t 00019, 00056, 00846, r 00846, 00865, 01911, d 00152,
: 35754, 02009; 00007, 00007, t 00020, 00063, 00852, r 00853, 00872, 01913, d 00159,
: 37764, 02010; 00007, 00007, t 00020, 00067, 00856, r 00856, 00876, 01914, d 00163,
=> works!
CLOCK_SHIFT_10US 100 = 1000us, 2p25MHz spi, diversity 1
=> tx does not lock up, receiver connects, but tx does not connect

=> it's a good idea, timing also looks a bit more regular => do it!

our headroom is time_in_receive - toa = (19.1-8.7) ms - 7.9 ms = 2.5 ms


timing measures 23.02.2022 2.4GHz 19Hz mode
rx diy-f103, times in 10us, on tx side RX tmo is 10
CLOCK_SHIFT_10US 100 = 1000us, 9MHz spi, diversity 1

: 32795, 05321; 00009, t 00016, 00064, 02424, r 02424, 02445, 05199, d 00185,
: 38110, 05315; 00005, t 00021, 00070, 02429, r 02429, 02451, 05194, d 00192,
: 43429, 05319; 00004, t 00016, 00064, 02424, r 02424, 02446, 05198, d 00185,
: 48748, 05319; 00007, t 00016, 00064, 02423, r 02424, 02445, 05198, d 00185,
: 54065, 05317; 00004, t 00019, 00067, 02427, r 02427, 02448, 05196, d 00188,
dopostreceive is t0 point
05321 = 53210 us -> cycle time
00009 = 90 us  -> time to set TX
t
00016 = 160 us -> tx entered
00064 = 640 us -> tx2 finished  => ca 400-500 us to get data, transfer frame to SX, and start transmitting
02424 = 24240 us -> txdone  => 24.2 ms for frame, consistent with toa 23.527ms  one should subtract 0.64 ms as t0 is not beginning of frame tx
r
02424 = 24240 us -> rx entered
02445 = 24450 us -> rx2 finished  => 200 us to start receiving
05199 = 51990 us -> rxdone  => 27.5 ms until it has received frame   => 1.0 ms, this is our CLOCK_SHIFT
d
00185 = 1850 us -> = tx2 - rxdone, time between tx2 and rxdone, matches ca. CLOCK_SHIFT + (tx2-tx)

our headroom is time_in_receive - toa = (52.0-24.5) ms - 23.6 ms = 3.9 ms

=> 53 ms is plenty, we could use 52 ms ...

*/
#endif // BLABLA_H



