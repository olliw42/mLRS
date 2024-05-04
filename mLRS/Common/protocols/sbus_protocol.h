//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// SBUS Protocol
//
// SBus frame: 0x0F, 22 bytes channel data, flags byte, 0x00
// 100000 bps, 8E2
// send every 14 ms (normal) or 7 ms (high speed), 10 ms or 20 ms
//
// 11 bit, 173 ... 992 .. 1811 for +-100%
// so: 9 ... 173 ... 992 .. 1811 ... 1965  for -120%  -100%    0%    +100%    +120%
// 100% = 819 span
// 120% = 983 span
// OpenTx produces 173 ... 992 ... 1811 for -100% ... 100%
//*******************************************************
#ifndef SBUS_PROTOCOL_H
#define SBUS_PROTOCOL_H
#pragma once


#ifndef PACKED
  #define SBUS_PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) // that's for __GNUC__
#else
  #define SBUS_PACKED  PACKED
#endif


#define SBUS_CHANNELPACKET_SIZE   22 // 22 bytes channels
#define SBUS_FRAME_SIZE           25 // stx + 22 bytes channels + flag + end


typedef enum {
    SBUS_STX              = 0x0F,
    SBUS_END_STX          = 0x00,
} SBUS_STX_ENUM;


typedef enum {
    SBUS_FLAG_CH17        = 0x01,
    SBUS_FLAG_CH18        = 0x02,
    SBUS_FLAG_FRAME_LOST  = 0x04,
    SBUS_FLAG_FAILSAFE    = 0x08,
} SBUS_FLAG_ENUM;


//-- Channel frame

typedef union
{
    uint8_t c[SBUS_CHANNELPACKET_SIZE];
    SBUS_PACKED(
    struct {
        uint16_t ch0  : 11; // 11 bits per channel * 16 channels = 22 bytes
        uint16_t ch1  : 11;
        uint16_t ch2  : 11;
        uint16_t ch3  : 11;
        uint16_t ch4  : 11;
        uint16_t ch5  : 11;
        uint16_t ch6  : 11;
        uint16_t ch7  : 11;
        uint16_t ch8  : 11;
        uint16_t ch9  : 11;
        uint16_t ch10 : 11;
        uint16_t ch11 : 11;
        uint16_t ch12 : 11;
        uint16_t ch13 : 11;
        uint16_t ch14 : 11;
        uint16_t ch15 : 11;
    });
} tSBusChannelBuffer;


SBUS_PACKED(
typedef struct {
    uint8_t stx;
    tSBusChannelBuffer ch;
    uint8_t flags;
    uint8_t end_stx;
}) tSBusFrame;


#endif // SBUS_PROTOCOL_H
