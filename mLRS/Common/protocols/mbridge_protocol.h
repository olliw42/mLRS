//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// M Bridge Protocol
//*******************************************************
#ifndef MBRIDGE_PROTOCOL_H
#define MMBRIDGE_PROTOCOL_H
#pragma once

#include <inttypes.h>


#ifndef PACKED
  #define MBRIDGE_PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) // that's for __GNUC__
#else
  #define MBRIDGE_PACKED  PACKED
#endif


// R: radio
// M: jr module
//
// format:
// radio->module:  stx1 stx2 len/cmd payload
// module->radio:  0/cmd payload
//
// radio is the master, module can transmit only after it got a radio frame
//
// r->m: the len/cmd byte is dual use: <= max payload size = serial data, else = command
// m->r: no len, will determine len based on timing
// m->r: no stx, will sync only based on timing
// has no crc, this should be a most reliable connection, so superfluous

#define MBRIDGE_STX1                          'O'
#define MBRIDGE_STX2                          'W'

#define MBRIDGE_R2M_SERIAL_PAYLOAD_LEN_MAX    24 // up to 24 bytes payload when received from transmitter
#define MBRIDGE_M2R_SERIAL_PAYLOAD_LEN_MAX    24 // up to 24 bytes payload when send from module to transmitter

#define MBRIDGE_CHANNELPACKET_SIZE            23 // 23 bytes payload, only received from transmitter

#define MBRIDGE_R2M_COMMAND_PAYLOAD_LEN_MAX   24 // 24 bytes payload
#define MBRIDGE_M2R_COMMAND_PAYLOAD_LEN_MAX   24 // 24 bytes payload

#define MBRIDGE_R2M_COMMAND_FRAME_LEN_MAX     25 // cmd byte + 24 bytes payload
#define MBRIDGE_M2R_COMMAND_FRAME_LEN_MAX     25 // cmd byte + 24 bytes payload


typedef enum {
    MBRIDGE_CHANNELPACKET_STX   = 0xFF, // marker which indicates a channel packet
    MBRIDGE_COMMANDPACKET_STX   = 0xA0, // 0b101x marker which indicates a command packet
    MBRIDGE_COMMANDPACKET_MASK  = 0xE0, // 0b111x => 5 bit = 32 commands max
} MBRIDGE_PACKET_STX_ENUM;


typedef enum {
   // zero cannot be used
    MBRIDGE_CMD_TX_LINK_STATS         = 2,
    MBRIDGE_CMD_REQUEST_INFO          = 3, // len = 0
    MBRIDGE_CMD_DEVICE_ITEM_TX        = 4,
    MBRIDGE_CMD_DEVICE_ITEM_RX        = 5,
    MBRIDGE_CMD_PARAM_REQUEST_LIST    = 6, // len = 0
    MBRIDGE_CMD_PARAM_ITEM            = 7,
    MBRIDGE_CMD_PARAM_ITEM2           = 8,
    MBRIDGE_CMD_PARAM_ITEM3           = 9,
    MBRIDGE_CMD_REQUEST_CMD           = 10,
    MBRIDGE_CMD_INFO                  = 11,
    MBRIDGE_CMD_PARAM_SET             = 12,
    MBRIDGE_CMD_PARAM_STORE           = 13, // len = 0
    MBRIDGE_CMD_BIND_START            = 14, // len = 0
    MBRIDGE_CMD_BIND_STOP             = 15, // len = 0
    MBRIDGE_CMD_MODELID_SET           = 16,
    MBRIDGE_CMD_SYSTEM_BOOTLOADER     = 17, // len = 0
} MBRIDGE_CMD_ENUM;


#define MBRIDGE_CMD_TX_LINK_STATS_LEN         22
#define MBRIDGE_CMD_DEVICE_ITEM_LEN           24
#define MBRIDGE_CMD_PARAM_ITEM_LEN            24
#define MBRIDGE_CMD_REQUEST_CMD_LEN           18
#define MBRIDGE_CMD_INFO_LEN                  24
#define MBRIDGE_CMD_PARAM_SET_LEN             7
#define MBRIDGE_CMD_MODELID_SET_LEN           3


uint8_t mbridge_cmd_payload_len(uint8_t cmd)
{
    switch (cmd) {
    case MBRIDGE_CMD_TX_LINK_STATS: return MBRIDGE_CMD_TX_LINK_STATS_LEN;
    case MBRIDGE_CMD_REQUEST_INFO: return 0;
    case MBRIDGE_CMD_DEVICE_ITEM_TX: return MBRIDGE_CMD_DEVICE_ITEM_LEN;
    case MBRIDGE_CMD_DEVICE_ITEM_RX: return MBRIDGE_CMD_DEVICE_ITEM_LEN;
    case MBRIDGE_CMD_PARAM_REQUEST_LIST: return 0;
    case MBRIDGE_CMD_PARAM_ITEM: return MBRIDGE_CMD_PARAM_ITEM_LEN;
    case MBRIDGE_CMD_PARAM_ITEM2: return MBRIDGE_CMD_PARAM_ITEM_LEN;
    case MBRIDGE_CMD_PARAM_ITEM3: return MBRIDGE_CMD_PARAM_ITEM_LEN;
    case MBRIDGE_CMD_REQUEST_CMD: return MBRIDGE_CMD_REQUEST_CMD_LEN;
    case MBRIDGE_CMD_INFO: return MBRIDGE_CMD_INFO_LEN;
    case MBRIDGE_CMD_PARAM_SET: return MBRIDGE_CMD_PARAM_SET_LEN; break;
    case MBRIDGE_CMD_PARAM_STORE: return 0;
    case MBRIDGE_CMD_BIND_START: return 0;
    case MBRIDGE_CMD_BIND_STOP: return 0;
    case MBRIDGE_CMD_MODELID_SET: return MBRIDGE_CMD_MODELID_SET_LEN; break;
    case MBRIDGE_CMD_SYSTEM_BOOTLOADER: return 0;
    }
    return 0;
}


//-- MBridge Channel Packet

// do not confuse with sbus, it is similar to sbus packet format, but not sbus values
typedef union {
  uint8_t c[MBRIDGE_CHANNELPACKET_SIZE]; // 176 + 8 = 184 bits = 23 bytes
  MBRIDGE_PACKED(
  struct {
      uint16_t ch0  : 11; // 14 channels a 11 bits per channel = 154 bits, 1 .. 1024 .. 2047 for +-120%
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


//-- MBridge LinkStats Command

MBRIDGE_PACKED(
typedef struct
{
    // transmitter side of things

    uint8_t LQ; // = LQ_valid_received; // number of valid packets received on transmitter side
    int8_t rssi1_instantaneous;
    int8_t rssi2_instantaneous;
    int8_t snr_instantaneous;

    int8_t rssi1_filtered;
    int8_t rssi2_filtered;
    int8_t snr_filtered;

    // receiver side of things

    uint8_t receiver_LQ; // = receiver_LQ_crc1_received; // number of rc data packets received on receiver side
    uint8_t receiver_LQ_serial; // = receiver_LQ_valid_received; // number of completely valid packets received on receiver side
    int8_t receiver_rssi_instantaneous;

    int8_t receiver_rssi_filtered;

    // both

    // 0: antenna1, 1: antenna2, instantaneous value
    uint8_t receive_antenna : 1;
    uint8_t transmit_antenna : 1;
    uint8_t receiver_receive_antenna : 1;
    uint8_t receiver_transmit_antenna : 1;
    uint8_t _diversity : 1; // pretty useless, so deprecate
    uint8_t _receiver_diversity : 1; // pretty useless, so deprecate
    uint8_t rx1_valid : 1;
    uint8_t rx2_valid : 1;

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

    uint8_t fhss_curr_i;
    uint8_t fhss_cnt;

    uint8_t vehicle_state : 2; // 0 = disarmed, 1 = armed 2 = flying, 3 = invalid/unknown
    uint8_t spare : 6;

    uint8_t link_state_connected : 1;
    uint8_t link_state_binding : 1;
    uint8_t spare2 : 6;
}) tMBridgeLinkStats; // 22 bytes


//-- MBridge More Commands

MBRIDGE_PACKED(
typedef struct
{
    uint8_t cmd_requested;
    MBRIDGE_PACKED(union {
        uint8_t cmd_request_data[17];
        MBRIDGE_PACKED(struct {
            uint8_t index;
            char name_16[16];
        }) param_item;
    });
}) tMBridgeRequestCmd; // 18 bytes


MBRIDGE_PACKED(
typedef struct
{
    int16_t receiver_sensitivity;
    uint8_t spare1;
    int8_t tx_actual_power_dbm;
    int8_t rx_actual_power_dbm;
    uint8_t rx_available : 1;
    uint8_t tx_actual_rdiversity : 2;
    uint8_t rx_actual_rdiversity : 2;
    uint8_t spare2 : 3;
    uint8_t tx_config_id;
    uint8_t tx_actual_tdiversity : 2;
    uint8_t rx_actual_tdiversity : 2;
    uint8_t spare3 : 4;
    uint8_t spare[16];
}) tMBridgeInfo; // 24 bytes


//-- MBridge DeviceItem Commands

MBRIDGE_PACKED(
typedef struct
{
    uint16_t firmware_version_u16;
    uint16_t setup_layout;
    char device_name_20[20];
}) tMBridgeDeviceItem; // 24 bytes


//-- MBridge ParamItem Commands

typedef enum {
    MBRIDGE_PARAM_TYPE_UINT8 = 0,
    MBRIDGE_PARAM_TYPE_INT8,
    MBRIDGE_PARAM_TYPE_UINT16,
    MBRIDGE_PARAM_TYPE_INT16,
    MBRIDGE_PARAM_TYPE_LIST,
    MBRIDGE_PARAM_TYPE_STR6,
} MBRIDGE_PARAM_TYPE_ENUM;


MBRIDGE_PACKED(
typedef struct
{
    uint8_t index;
    uint8_t type;
    char name_16[16];
    MBRIDGE_PACKED(union {
        tParamValue value;
        char str6_6[6];
    });
}) tMBridgeParamItem; // 24 bytes


MBRIDGE_PACKED(
typedef struct
{
    uint8_t index;
    MBRIDGE_PACKED(union {
        MBRIDGE_PACKED(struct {
            tParamValue min;
            tParamValue max;
            tParamValue dflt;
            char unit_6[6];
        });
        MBRIDGE_PACKED(struct {
            uint16_t allowed_mask;
            char options_21[21];
        });
    });
}) tMBridgeParamItem2; // 24 bytes


MBRIDGE_PACKED(
typedef struct
{
    uint8_t index;
    MBRIDGE_PACKED(union {
        char options2_23[23];
    });
}) tMBridgeParamItem3; // 24 bytes


MBRIDGE_PACKED(
typedef struct
{
    uint8_t index;
    MBRIDGE_PACKED(union {
        tParamValue value;
        char str6_6[6];
    });
}) tMBridgeParamSet; // 7 bytes


//-- check some sizes

STATIC_ASSERT(sizeof(tMBridgeChannelBuffer) == MBRIDGE_CHANNELPACKET_SIZE, "tMBridgeChannelBuffer len missmatch")
STATIC_ASSERT(sizeof(tMBridgeLinkStats) == MBRIDGE_CMD_TX_LINK_STATS_LEN, "tMBridgeLinkStats len missmatch")
STATIC_ASSERT(sizeof(tMBridgeRequestCmd) == MBRIDGE_CMD_REQUEST_CMD_LEN, "tMBridgeRequestCmd len missmatch")
STATIC_ASSERT(sizeof(tMBridgeInfo) == MBRIDGE_CMD_INFO_LEN, "tMBridgeInfo len missmatch")
STATIC_ASSERT(sizeof(tMBridgeDeviceItem) == MBRIDGE_CMD_DEVICE_ITEM_LEN, "tMBridgeDeviceItem len missmatch")
STATIC_ASSERT(sizeof(tMBridgeParamItem) == MBRIDGE_CMD_PARAM_ITEM_LEN, "tMBridgeParamItem len missmatch")
STATIC_ASSERT(sizeof(tMBridgeParamItem2) == MBRIDGE_CMD_PARAM_ITEM_LEN, "tMBridgeParamItem2 len missmatch")
STATIC_ASSERT(sizeof(tMBridgeParamItem3) == MBRIDGE_CMD_PARAM_ITEM_LEN, "tMBridgeParamItem3 len missmatch")
STATIC_ASSERT(sizeof(tMBridgeParamSet) == MBRIDGE_CMD_PARAM_SET_LEN, "tMBridgeParamSet len missmatch")


#endif // MBRIDGE_PROTOCOL_H

