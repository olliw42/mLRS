//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// M Bridge Protocol
//********************************************************
#ifndef MBRIDGE_PROTOCOL_H
#define MMBRIDGE_PROTOCOL_H
#pragma once

#include <inttypes.h>


#ifndef PACKED
  #define MBRDIGE_PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) // that's for __GNUC__
#else
  #define MBRDIGE_PACKED  PACKED
#endif


// R: radio
// M: jr module
//
// format:
// radio->module:  stx1 stx2 len/cmd payload
// module->radio:  cmd payload
//
// radio is the master, module can transmit only after it got a radio frame
//
// r->m: the len/cmd byte is dual use: <= max payload size = serial data, else = command
// m->r: no len, will determine len based on timing
// m->r: no stx, will sync only based on timing
// has no crc, this should be a most reliable connection, so superfluous

#define MBRIDGE_STX1                          'O'
#define MBRIDGE_STX2                          'W'

#define MBRIDGE_R2M_SERIAL_PAYLOAD_LEN_MAX    24 // up to 26 bytes payload when received from transmitter
#define MBRIDGE_M2R_SERIAL_PAYLOAD_LEN_MAX    24 // up to 26 bytes payload when send from module to transmitter

#define MBRIDGE_CHANNELPACKET_SIZE            23 // 23 bytes payload, only received from transmitter

#define MBRIDGE_R2M_COMMAND_PAYLOAD_LEN_MAX   24 // 26 bytes payload
#define MBRIDGE_M2R_COMMAND_PAYLOAD_LEN_MAX   24 // 26 bytes payload

#define MBRIDGE_R2M_COMMAND_FRAME_LEN_MAX     25 // cmd byte + 26 bytes payload
#define MBRIDGE_M2R_COMMAND_FRAME_LEN_MAX     25 // cmd byte + 26 bytes payload


typedef enum {
  MBRIDGE_CHANNELPACKET_STX   = 0xFF, // marker which indicates a channel packet
  MBRIDGE_COMMANDPACKET_STX   = 0xA0, // 0b101x marker which indicates a command packet
  MBRIDGE_COMMANDPACKET_MASK  = 0xE0, // 0b111x
} MBRIDGE_PACKET_STX_ENUM;


typedef enum {
  MBRIDGE_TYPE_NONE = 0,
  MBRIDGE_TYPE_SERIALPACKET,
  MBRIDGE_TYPE_CHANNELPACKET,
  MBRIDGE_TYPE_COMMANDPACKET,
} MBRIDGE_TYPE_ENUM;;


typedef enum {
  MBRIDGE_CMD_TX_LINK_STATS = 0x02,
} MBRIDGE_CMD_ENUM;

#define MBRIDGE_CMD_TX_LINK_STATS_LEN         22


uint8_t mbridge_cmd_payload_len(uint8_t cmd)
{
  switch (cmd) {
  case MBRIDGE_CMD_TX_LINK_STATS: return MBRIDGE_CMD_TX_LINK_STATS_LEN;
  }
  return 0;
}


// do not confuse with sbus, it is similar to sbus packet format, but not sbus values
typedef union {
  uint8_t c[MBRIDGE_CHANNELPACKET_SIZE]; // 176 + 8 = 184 bits = 23 bytes
  MBRDIGE_PACKED(
  struct {
      uint16_t ch0  : 11; // 14 channels a 11 bits per channel = 154 bits, 0 .. 1024 .. 2047
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
      uint8_t ch16 : 1; // 2 channels a 1 bits per channel = 2 bits, 0..1
      uint8_t ch17 : 1;
  });
} tMBridgeChannelBuffer;


MBRDIGE_PACKED(
typedef struct
{
  // transmitter side of things

  uint8_t LQ; // = LQ_valid_received; // number of valid packets received on transmitter side
  int8_t rssi_instantaneous;
  int8_t rssi2_instantaneous;
  int8_t snr_instantaneous; // invalid = INT8_MAX

  int8_t rssi_filtered;
  int8_t rssi2_filtered;
  int8_t snr_filtered;

  // receiver side of things

  uint8_t receiver_LQ; // = receiver_LQ_crc1_received; // number of rc data packets received on receiver side
  uint8_t receiver_LQ_serial; // = receiver_LQ_valid_received; // number of completely valid packets received on receiver side
  int8_t receiver_rssi_instantaneous;
  int8_t receiver_rssi2_instantaneous;
  int8_t receiver_snr_instantaneous; // invalid = INT8_MAX

  int8_t receiver_rssi_filtered;
  int8_t receiver_rssi2_filtered;
  int8_t receiver_snr_filtered;

  // both

  // 0: antenna1, 1: antenna2, instantaneous value
  uint8_t transmitter_receive_antenna : 1;
  uint8_t transmitter_transmit_antenna : 1;
  uint8_t receiver_receive_antenna : 1;
  uint8_t receiver_transmit_antenna : 1;
  uint8_t spare : 4;

  // further stats acquired on transmitter side

  // number of transmitted packets per sec is always 50  => max transmit data rate = 50 * 64
  // so we only need to count transmitted packets with new serial data => current max transmit data rate = Nnew * 64
  // 50 - Nnew is the number of retransmissions
  // in addition we count the bytes transmitted
  uint8_t LQ_fresh_serial_packets_transmitted;
  uint8_t bytes_per_sec_transmitted;

  // number of received serial data packets per sec is LQ_valid_received => max receive data rate = N * 82
  // we further count received packets with new serial data => current max transmit data rate = Nnew * 64
  // N - Nnew is the number of retransmissions
  // in addition we count the bytes received
  uint8_t LQ_valid_received;  // number of completely valid packets received per sec
  uint8_t LQ_fresh_serial_packets_received;
  uint8_t bytes_per_sec_received;

  uint8_t LQ_received; // number of packets received per sec, not practically relevant
}) tMBridgeLinkStats; // 22 bytes



#endif // MBRIDGE_PROTOCOL_H

