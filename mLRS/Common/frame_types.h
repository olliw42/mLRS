//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// FRAME TYPES & DEFS
//*******************************************************
#ifndef FRAME_TYPES_H
#define FRAME_TYPES_H
#pragma once


#include <inttypes.h>


#ifndef PACKED
#  define PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) //that's for __GNUC__
#endif


//-------------------------------------------------------
// frames types only
//-------------------------------------------------------

typedef struct
{
  uint8_t seq_no;
  uint8_t ack;
  int8_t rssi;
  int8_t snr;
  uint8_t LQ;
} tFrameStats;


typedef enum {
  FRAME_TYPE_TX = 0x00,
  FRAME_TYPE_RX = 0x01,
} FRAME_TYPE_ENUM;


#define FRAME_HEADER_LEN        6
#define FRAME_TX_RCDATA1_LEN    6
#define FRAME_TX_RCDATA2_LEN    10
#define FRAME_TX_PAYLOAD_LEN    64 // 82 - 10-6(rcdata) - 2(crc) = 64
#define FRAME_RX_PAYLOAD_LEN    82


PACKED(
typedef struct
{
  uint32_t seq_no:3;
  uint32_t ack:1;
  uint32_t frame_type:4;
  uint32_t rssi_u8:7;
  uint32_t LQ:7;
  uint32_t payload_len:7;
  uint32_t spare:3;
}) tStatus;


//-- Tx Frames ----------

#define RC_DATE_LEN  18


typedef struct
{
  uint16_t ch[RC_DATE_LEN]; // 0 .. 1024 .. 2047, 11 bits
} tRcData;


PACKED(
typedef struct
{
  uint16_t ch0  : 11; // 0 .. 1024 .. 2047, 11 bits
  uint16_t ch1  : 11;
  uint16_t ch2  : 11;
  uint16_t ch3  : 11;
  uint16_t ch14 :  1; // 0..1, 1 bit
  uint16_t ch15 :  1;
  uint16_t ch16 :  1;
  uint16_t ch17 :  1;
}) tRcData1;


PACKED(
typedef struct
{
  uint8_t ch[FRAME_TX_RCDATA2_LEN]; // 0 .. 128 .. 255, 8 bits
}) tRcData2;


PACKED(
typedef struct
{
  uint16_t sync_word; // 2 bytes
  tStatus status; // 4 bytes
  tRcData1 rc1; // 6 bytes
  uint8_t crc1;
  tRcData2 rc2; // 10 bytes
  uint8_t crc2;
  uint8_t payload[64]; // = FRAME_TX_PAYLOAD_LEN
  uint16_t crc;
}) tTxFrame;


//-- Rx Frames ----------

PACKED(
typedef struct
{
  uint16_t sync_word; // 2 bytes
  tStatus status; // 4 bytes
  uint8_t payload[82]; // = FRAME_RX_PAYLOAD_LEN
  uint16_t crc;
}) tRxFrame;



#endif // FRAME_TYPES_H
