//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Setup and configuration types
//*******************************************************
#ifndef SETUP_TYPES_H
#define SETUP_TYPES_H
#pragma once


#include <stdint.h>


//-------------------------------------------------------
// Setup Enums
//-------------------------------------------------------

typedef enum {
    SETUP_FREQUENCY_BAND_2P4_GHZ = 0,
    SETUP_FREQUENCY_BAND_915_MHZ_FCC,
    SETUP_FREQUENCY_BAND_868_MHZ,
    SETUP_FREQUENCY_BAND_NUM,
} SETUP_FREQUENCY_BAND_ENUM;


typedef enum {
    MODE_50HZ = 0,
    MODE_31HZ,
    MODE_19HZ,
    MODE_NUM,
} MODE_ENUM;


typedef enum {
    DIVERSITY_DEFAULT = 0,
    DIVERSITY_ANTENNA1, // antenna 1 if diversity available
    DIVERSITY_ANTENNA2, // antenna 2 if diversity available
    DIVERSITY_NUM,
} DIVERSITY_ENUM;


typedef enum {
    SERIAL_BAUDRATE_9600 = 0,
    SERIAL_BAUDRATE_19200,
    SERIAL_BAUDRATE_38400,
    SERIAL_BAUDRATE_57600,
    SERIAL_BAUDRATE_115200,
    SERIAL_BAUDRATE_NUM,
} SERIAL_BAUDRATE_ENUM;


typedef enum {
    CHANNEL_ORDER_AETR = 0, // most common
    CHANNEL_ORDER_TAER, // spektrum/graupner/jr
    CHANNEL_ORDER_ETAR,
    CHANNEL_ORDER_NUM,
} CHANNEL_ORDER_ENUM;


typedef enum {
    SERIAL_LINK_MODE_TRANSPARENT = 0,
    SERIAL_LINK_MODE_MAVLINK,
    SERIAL_LINK_MODE_NUM,
} SERIAL_LINK_MODE_ENUM;


typedef enum {
    SEND_RADIO_STATUS_OFF = 0,
    SEND_RADIO_STATUS_ON,
    SEND_RADIO_STATUS_ON_W_TXBUF,
    SEND_RADIO_STATUS_NUM,
} SEND_RADIO_STATUS_ENUM;


//-- Tx

typedef enum {
    SERIAL_DESTINATION_SERIAL_PORT = 0,
    SERIAL_DESTINATION_MBRDIGE,
    SERIAL_DESTINATION_NUM,
} TX_SERIAL_DESTINATION_ENUM;


typedef enum {
    CHANNEL_SOURCE_NONE = 0,
    CHANNEL_SOURCE_MBRIDGE, // JR pin5
    CHANNEL_SOURCE_INPORT, // In port
    CHANNEL_SOURCE_CRSF, // JR pin5
    CHANNEL_SOURCE_NUM,
} TX_CHANNELS_SOURCE_ENUM;


typedef enum {
    IN_CONFIG_SBUS = 0,
    IN_CONFIG_SBUS_INVERTED,
    IN_CONFIG_NUM,
} TX_IN_CONFIG_ENUM;


//-- Rx

typedef enum {
    OUT_CONFIG_SBUS = 0,
    OUT_CONFIG_CRSF,
    OUT_CONFIG_SBUS_INVERTED,
    OUT_CONFIG_NUM,
} RX_OUT_CONFIG_ENUM;


typedef enum {
    OUT_RSSI_CHANNEL_OFF = 0,
    OUT_RSSI_CHANNEL_CH5,
    OUT_RSSI_CHANNEL_CH6,
    OUT_RSSI_CHANNEL_CH7,
    OUT_RSSI_CHANNEL_CH8,
    OUT_RSSI_CHANNEL_CH9,
    OUT_RSSI_CHANNEL_CH10,
    OUT_RSSI_CHANNEL_CH11,
    OUT_RSSI_CHANNEL_CH12,
    OUT_RSSI_CHANNEL_NUM,
} RX_OUT_RSSI_CHANNEL_ENUM;


typedef enum {
    FAILSAFE_MODE_NO_SIGNAL = 0,
    FAILSAFE_MODE_LOW_THROTTLE,
    FAILSAFE_MODE_AS_CONFIGURED,
    FAILSAFE_MODE_LOW_THROTTLE_ELSE_CENTER,
    FAILSAFE_MODE_CH1CH4_CENTER,
    FAILSAFE_MODE_NUM,
} RX_FAILSAFE_MODE_ENUM;


//-------------------------------------------------------
// Config Enums
//-------------------------------------------------------
// numbers must match config arrays in sx drivers !!

typedef enum {
    SX128x_LORA_CONFIG_BW800_SF5_CRLI4_5 = 0,
    SX128x_LORA_CONFIG_BW800_SF6_CRLI4_5,
    SX128x_LORA_CONFIG_BW800_SF7_CRLI4_5,
    SX128x_LORA_CONFIG_NUM,
} SX128x_LORA_CONFIG_ENUM;


typedef enum {
    SX127x_LORA_CONFIG_BW500_SF6_CR4_5 = 0,
    SX127x_LORA_CONFIG_NUM,
} SX127x_LORA_CONFIG_ENUM;


typedef enum {
    SX126x_LORA_CONFIG_BW500_SF5_CR4_5 = 0,
    SX126x_LORA_CONFIG_BW500_SF6_CR4_5,
    SX126x_LORA_CONFIG_NUM,
} SX126x_LORA_CONFIG_ENUM;


//-------------------------------------------------------
// Setup and Config Types
//-------------------------------------------------------

typedef struct
{
    uint8_t Power;
    uint8_t Diversity;
    uint8_t ChannelsSource;
    uint8_t ChannelOrder;
    uint8_t InMode;
    uint8_t SerialDestination;
    uint8_t SerialBaudrate;
    uint8_t SerialLinkMode;
    uint8_t SendRadioStatus;

    uint8_t spare[11];
} tTxSetup;


typedef struct
{
    uint8_t Power;
    uint8_t Diversity;
    uint8_t ChannelOrder;
    uint8_t OutMode;
    uint8_t OutRssiChannelMode;
    uint8_t FailsafeMode;
    uint8_t SerialBaudrate;
    uint8_t SerialLinkMode;
    uint8_t SendRadioStatus;

    uint8_t spare[11];

    int8_t FailsafeOutChannelValues_Ch1_Ch12[12]; // -120 .. +120
    uint8_t FailsafeOutChannelValues_Ch13_Ch16[4]; // 0,1,2 = -120, 0, +120
} tRxSetup;


#define SETUP_MARKER_STR      "SetupStartMarker"
#define SETUP_MARKEREND_STR   "!end!"

// user setable parameter values, stored in EEPROM
typedef struct
{
    char MarkerStr[16];
    uint32_t Version;
    uint16_t Layout;

    // parameters common to both Tx and Rx
    // cannot be changed on the fly, loss of connection will happen, need restart/reconnect
    char BindPhrase[6+1];
    uint8_t FrequencyBand;
    uint8_t Mode;

    uint8_t spare[7];

    // parameters specific to Rx, can be changed on the fly
    tRxSetup Rx;

    // parameters specific to Tx, can be changed on the fly
    tTxSetup Tx;

    char MarkerEnd[8];
} tSetup;


typedef struct
{
    uint16_t FrequencyBand_allowed_mask;
    uint16_t Mode_allowed_mask;

    char Tx_Power_optstr[32+1];
    uint16_t Tx_Diversity_allowed_mask;
    uint16_t Tx_ChannelsSource_allowed_mask;
    uint16_t Tx_InMode_allowed_mask;
    uint16_t Tx_SerialDestination_allowed_mask;

    char Rx_Power_optstr[32+1];
    uint16_t Rx_Diversity_allowed_mask;
    uint16_t Rx_OutMode_allowed_mask;

    bool rx_available;

    uint32_t rx_firmware_version;
    uint16_t rx_setup_layout;
    char rx_device_name[20+1];
    int8_t rx_actual_power_dbm;
    uint8_t rx_actual_diversity;
} tSetupMetaData;


// global configuration values, not stored in EEPROM
// can be/are derived from setup parameters, from defines, or otherwise
typedef struct
{
    uint8_t Mode;

    uint8_t LoraConfigIndex;
    uint8_t lora_send_frame_tmo;

    uint16_t FrameSyncWord;
    uint16_t FhssNum;
    uint32_t FhssSeed;

    uint16_t LQAveragingPeriod;

    int8_t Power_dbm;
    uint32_t SerialBaudrate;

    uint16_t frame_rate_ms;
    uint16_t frame_rate_hz;
    uint16_t connect_tmo_systicks;
    uint16_t connect_listen_hop_cnt;

    bool UseAntenna1;
    bool UseAntenna2;

    bool UseMbridge;
    bool UseCrsf;
} tGlobalConfig;


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#define SEND_FRAME_TMO            Config.lora_send_frame_tmo

#define CONNECT_TMO_SYSTICKS      Config.connect_tmo_systicks
#define CONNECT_LISTEN_HOP_CNT    Config.connect_listen_hop_cnt

#ifdef DEVICE_HAS_DIVERSITY
#define IF_ANTENNA1(x)            if (Config.UseAntenna1) { x; }
#define IF_ANTENNA2(x)            if (Config.UseAntenna2) { x; }
#define USE_ANTENNA1              (Config.UseAntenna1)
#define USE_ANTENNA2              (Config.UseAntenna2)
#else
#define IF_ANTENNA1(x)            x;
#define IF_ANTENNA2(x)
#define USE_ANTENNA1              true
#define USE_ANTENNA2              false
#endif

#ifdef DEVICE_HAS_JRPIN5
#define IF_MBRIDGE(x)             if (Config.UseMbridge) { x; }
#define IF_CRSF(x)                if (Config.UseCrsf) { x; }
#else
#define IF_MBRIDGE(x)
#define IF_CRSF(x)
#endif


#endif // SETUP_TYPES_H
