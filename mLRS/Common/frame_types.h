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

typedef enum {
    FRAME_TYPE_TX = 0x00,
    FRAME_TYPE_RX = 0x01,
    FRAME_TYPE_TX_RX_CMD = 0x02, // these commands use the normal Tx/Rx frames, with repurposed payload however
} FRAME_TYPE_ENUM;


//-------------------------------------------------------
// normal Tx,Rx frames
//-------------------------------------------------------

typedef struct
{
    uint8_t seq_no;
    uint8_t ack;
    int8_t rssi;
    uint8_t LQ; // that's the LQ we want to report to the world
    uint8_t LQ_serial_data;
    uint8_t antenna;
    uint8_t transmit_antenna;
} tFrameStats;


#define FRAME_TX_RX_HEADER_LEN  7
#define FRAME_TX_RCDATA1_LEN    6
#define FRAME_TX_RCDATA2_LEN    10
#define FRAME_TX_PAYLOAD_LEN    64 // 82 - 10-6(rcdata) - 2(crc) = 64
#define FRAME_RX_PAYLOAD_LEN    82


PACKED(
typedef struct
{
    uint8_t seq_no : 3;
    uint8_t ack : 1;
    uint8_t frame_type : 4;
    uint32_t antenna : 1;
    uint32_t rssi_u7 : 7;
    uint32_t LQ : 7; // only Rx->Tx frame, not Tx->Rx
    uint32_t LQ_serial_data : 7;
    uint32_t transmit_antenna : 1;
    uint32_t spare : 2;
    uint32_t payload_len : 7;
}) tFrameStatus;


//-- Tx Frame ----------

PACKED(
typedef struct
{
    uint16_t ch0  : 11; // 0 .. 1024 .. 2047, 11 bits
    uint16_t ch1  : 11;
    uint16_t ch2  : 11;
    uint16_t ch3  : 11;
    uint16_t ch12 :  2; // 0 .. 1 .. 2, 2 bits, 3-way
    uint16_t ch13 :  2;
}) tFrameRcData1; // 6 bytes


PACKED(
typedef struct
{
    uint16_t ch4  : 11; // 0 .. 1024 .. 2047, 11 bits
    uint16_t ch5  : 11;
    uint16_t ch6  : 11;
    uint16_t ch7  : 11;
    uint16_t ch14 :  2; // 0 .. 1 .. 2, 2 bits, 3-way
    uint16_t ch15 :  2;
    uint8_t ch8;        // 0 .. 128 .. 255, 8 bits
    uint8_t ch9;        // 0 .. 128 .. 255, 8 bits
    uint8_t ch10;       // 0 .. 128 .. 255, 8 bits
    uint8_t ch11;       // 0 .. 128 .. 255, 8 bits
}) tFrameRcData2; // 10 bytes


PACKED(
typedef struct
{
    uint16_t sync_word; // 2 bytes
    tFrameStatus status; // 5 bytes
    tFrameRcData1 rc1; // 6 bytes
    uint16_t crc1;
    tFrameRcData2 rc2; // 10 bytes
    uint8_t payload[64]; // = FRAME_TX_PAYLOAD_LEN
    uint16_t crc;
}) tTxFrame; // 91 bytes


//-- Rx Frame ----------

PACKED(
typedef struct
{
    uint16_t sync_word; // 2 bytes
    tFrameStatus status; // 5 bytes
    uint8_t payload[82]; // = FRAME_RX_PAYLOAD_LEN
    uint16_t crc;
}) tRxFrame; // 91 bytes


//-------------------------------------------------------
// Bind frames
// are send on bind frequency in 19 Hz mode
//-------------------------------------------------------

PACKED(
typedef struct
{
    uint64_t bind_signature; // 8 bytes // different for Tx and Rx
    uint8_t seq_no : 3;
    uint8_t ack : 1;
    uint8_t frame_type : 4; // 1 byte // not used currently

    uint8_t connected : 1;
    uint8_t spare : 7;

    char BindPhrase_6[6];
    uint8_t FrequencyBand_XXX : 4; // TODO
    uint8_t Mode : 4;
    uint8_t Ortho : 4;

    uint8_t spare1 : 4;
    uint8_t spare2[71];

    uint16_t crc; // 2 bytes
}) tTxBindFrame; // 91 bytes


PACKED(
typedef struct
{
    uint64_t bind_signature; // 8 bytes // different for Tx and Rx
    uint8_t seq_no : 3;
    uint8_t ack : 1;
    uint8_t frame_type : 4; // 1 byte // not used currently

    uint8_t connected : 1;
    uint8_t spare : 7;

    uint32_t firmware_version;
    char device_name_20[20];

    uint8_t spare2[55];

    uint16_t crc; // 2bytes
}) tRxBindFrame; // 91 bytes


//-------------------------------------------------------
// Cmd & SetupData frames
// these are just normal Tx and Rx frames, with special payloads though
//-------------------------------------------------------

typedef enum {
    FRAME_CMD_NONE = 0,
    FRAME_CMD_RX_REBOOT,
    FRAME_CMD_RX_BIND,

    // these commands use the normal Tx/Rx frames, with re-purposed payload however
    FRAME_CMD_GET_RX_SETUPDATA = 32,    // tx -> rx, ask for parameters & metadata -> response with RX_SETUPDATA
    FRAME_CMD_RX_SETUPDATA,             // rx -> tx, return parameters & metadata
    FRAME_CMD_SET_RX_PARAMS,            // tx -> rx, set parameters -> response with RX_ACK
    FRAME_CMD_STORE_RX_PARAMS,          // tx -> rx, ask to store parameters -> response with RX_SETUPDATA
    FRAME_CMD_GET_RX_SETUPDATA_WRELOAD,
} FRAME_CMD_ENUM;


// Rx Parameter structure
// limit to 4 bits, so max 16 options per parameter; is also the limit of allowed_mask_ptr in tSetupParameterItem
PACKED(
typedef struct
{
    uint8_t Power : 4;
    uint8_t Diversity : 4;
    uint8_t ChannelOrder : 4;
    uint8_t OutMode : 4;
    uint8_t OutRssiChannelMode : 4;
    uint8_t FailsafeMode : 4;
    uint8_t SerialBaudrate : 4;
    uint8_t SerialLinkMode : 4;
    uint8_t SendRadioStatus : 4;
    uint8_t Buzzer : 4;
    uint8_t SendRcChannels : 4;
    uint8_t __RadioStatusMethod : 4; // deprecated
    uint8_t OutLqChannelMode : 4;

    uint8_t spare : 4;
    uint8_t spare2[4];

    int8_t FailsafeOutChannelValues_Ch1_Ch12[12]; // -120 .. +120
    uint8_t FailsafeOutChannelValue_Ch13 : 2;
    uint8_t FailsafeOutChannelValue_Ch14 : 2;
    uint8_t FailsafeOutChannelValue_Ch15 : 2;
    uint8_t FailsafeOutChannelValue_Ch16 : 2;
}) tCmdFrameRxParameters; // 24 bytes


// send from Rx as response to GET_RX_SETUPDATA
PACKED(
typedef struct
{
    uint8_t cmd;
    uint8_t spare;

    // rx setup meta data 1
    uint16_t firmware_version_u16;
    uint16_t setup_layout;
    char device_name_20[20];
    int8_t actual_power_dbm;
    uint8_t actual_diversity;

    // rx parameter values
    // BindPhrase, FrequencyBand, Mode must be equal to Tx, otherwise Rx wouldn't connect, so don't have to be send
    tCmdFrameRxParameters RxParams; // 24 bytes

    // rx setup meta data 2, parameter metadata
    uint16_t FrequencyBand_allowed_mask_XXX; // TODO
    uint8_t Mode_allowed_mask_XXX; // TODO
    uint8_t Ortho_allowed_mask_XXX; // TODO
    uint8_t spare2[2];

    int16_t Power_list[8];
    uint8_t Diversity_allowed_mask;
    uint8_t OutMode_allowed_mask;
    uint8_t Buzzer_allowed_mask;

    uint8_t spare3[5];
}) tRxCmdFrameRxSetupData; // 82 bytes


// send from Tx to do SET_RX_PARAMS
PACKED(
typedef struct
{
    uint8_t cmd;
    uint8_t spare;

    // rx parameter values
    char BindPhrase_6[6];
    uint8_t FrequencyBand : 4;
    uint8_t Mode : 4;
    uint8_t Ortho : 4;

    uint8_t spare1 : 4;
    uint8_t spare2[2];

    tCmdFrameRxParameters RxParams; // 24 bytes

    uint8_t spare3[28];
}) tTxCmdFrameRxParams; // 64 bytes


// for type casting to get the header
PACKED(
typedef struct
{
    uint8_t cmd;
}) tCmdFrameHeader; // 1 byte



#endif // FRAME_TYPES_H
