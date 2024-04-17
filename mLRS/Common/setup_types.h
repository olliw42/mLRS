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
#include "common_conf.h"
#include "hal/device_conf.h"


//-------------------------------------------------------
// Setup Enums
// ATTENTION: only extend/append, never change sequence
//-------------------------------------------------------

//-- common to Tx & Rx

typedef enum {
    SETUP_FREQUENCY_BAND_2P4_GHZ = 0,
    SETUP_FREQUENCY_BAND_915_MHZ_FCC,
    SETUP_FREQUENCY_BAND_868_MHZ,
    SETUP_FREQUENCY_BAND_433_MHZ,
    SETUP_FREQUENCY_BAND_70_CM_HAM,
    SETUP_FREQUENCY_BAND_866_MHZ_IN,
    SETUP_FREQUENCY_BAND_NUM,
} SETUP_FREQUENCY_BAND_ENUM;


typedef enum {
    MODE_50HZ = 0,
    MODE_31HZ,
    MODE_19HZ,
    MODE_FLRC_111HZ,
    MODE_FSK_50HZ,
    MODE_NUM,
} MODE_ENUM;


typedef enum {
    ORTHO_NONE = 0,
    ORTHO_1_3,
    ORTHO_2_3,
    ORTHO_3_3,
    ORTHO_NUM,
} ORTHO_ENUM;


typedef enum {
    EXCEPT_NONE = 0,
    EXCEPT_2P4_GHZ_WIFIBAND_1,
    EXCEPT_2P4_GHZ_WIFIBAND_6,
    EXCEPT_2P4_GHZ_WIFIBAND_11,
    EXCEPT_2P4_GHZ_WIFIBAND_13,
    EXCEPT_NUM,
} EXCEPT_ENUM;


typedef enum {
    DIVERSITY_DEFAULT = 0, // diversity enabled, both receive and transmit
    DIVERSITY_ANTENNA1, // antenna 1
    DIVERSITY_ANTENNA2, // antenna 2
    DIVERSITY_R_ENABLED_T_ANTENNA1, // receive diversity enabled, transmit antenna 1
    DIVERSITY_R_ENABLED_T_ANTENNA2, // receive diversity enabled, transmit antenna 2
    DIVERSITY_NUM,
} DIVERSITY_ENUM;


typedef enum {
    SERIAL_BAUDRATE_9600 = 0,
    SERIAL_BAUDRATE_19200,
    SERIAL_BAUDRATE_38400,
    SERIAL_BAUDRATE_57600,
    SERIAL_BAUDRATE_115200,
    SERIAL_BAUDRATE_230400,
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
#ifdef USE_FEATURE_MAVLINKX
    SERIAL_LINK_MODE_MAVLINK_X,
#endif
    SERIAL_LINK_MODE_NUM,
} SERIAL_LINK_MODE_ENUM;


#ifndef USE_FEATURE_MAVLINKX
  #define SERIAL_LINK_MODE_IS_MAVLINK(x)  ((x) == SERIAL_LINK_MODE_MAVLINK)
#else
  #define SERIAL_LINK_MODE_IS_MAVLINK(x)  ((x) == SERIAL_LINK_MODE_MAVLINK || (x) == SERIAL_LINK_MODE_MAVLINK_X)
#endif


typedef enum {
    TX_SEND_RADIO_STATUS_OFF = 0,
    TX_SEND_RADIO_STATUS_1HZ,
    TX_SEND_RADIO_STATUS_NUM,
} TX_SEND_RADIO_STATUS_ENUM;

typedef enum {
    RX_SEND_RADIO_STATUS_OFF = 0,
    RX_SEND_RADIO_STATUS_METHOD_ARDUPILOT_1,
    RX_SEND_RADIO_STATUS_METHOD_PX4,
    RX_SEND_RADIO_STATUS_NUM,
} RX_SEND_RADIO_STATUS_ENUM;


//-- Tx only

typedef enum {
    SERIAL_DESTINATION_SERIAL = 0,
    SERIAL_DESTINATION_SERIAL2,
    SERIAL_DESTINATION_MBRDIGE,
    SERIAL_DESTINATION_NUM,
} TX_SERIAL_DESTINATION_ENUM;
typedef enum {
    L0329_SERIAL_DESTINATION_SERIAL = 0,
    L0329_SERIAL_DESTINATION_MBRDIGE,
    L0329_SERIAL_DESTINATION_SERIAL2,
    L0329_SERIAL_DESTINATION_NUM,
} L0329_TX_SERIAL_DESTINATION_ENUM;


typedef enum {
    CHANNEL_SOURCE_NONE = 0,
    CHANNEL_SOURCE_CRSF, // JR pin5
    CHANNEL_SOURCE_INPORT, // In port
    CHANNEL_SOURCE_MBRIDGE, // JR pin5
    CHANNEL_SOURCE_NUM,
} TX_CHANNELS_SOURCE_ENUM;
typedef enum {
    L0329_CHANNEL_SOURCE_NONE = 0,
    L0329_CHANNEL_SOURCE_MBRIDGE,
    L0329_CHANNEL_SOURCE_INPORT,
    L0329_CHANNEL_SOURCE_CRSF,
    L0329_CHANNEL_SOURCE_NUM,
} L0329_TX_CHANNELS_SOURCE_ENUM;


typedef enum {
    IN_CONFIG_SBUS = 0,
    IN_CONFIG_SBUS_INVERTED,
    IN_CONFIG_NUM,
} TX_IN_CONFIG_ENUM;


typedef enum {
    CLI_LINE_END_CR = 0,
    CLI_LINE_END_LF,
    CLI_LINE_END_CRLF,
    CLI_LINE_END_NUM,
} TX_CLI_LINE_END_ENUM;


typedef enum {
    BUZZER_OFF = 0,
    BUZZER_LOST_PACKETS,
    BUZZER_RX_LQ,
    BUZZER_NUM,
} TX_BUZZER_END_ENUM;


//-- Rx only

typedef enum {
    OUT_CONFIG_SBUS = 0,
    OUT_CONFIG_CRSF,
    OUT_CONFIG_SBUS_INVERTED,
    OUT_CONFIG_CRSF_TX_JRPIN5,
    OUT_CONFIG_NUM,
} RX_OUT_CONFIG_ENUM;


typedef enum {
    OUT_RSSI_LQ_CHANNEL_OFF = 0,
    OUT_RSSI_LQ_CHANNEL_CH5,
    OUT_RSSI_LQ_CHANNEL_CH6,
    OUT_RSSI_LQ_CHANNEL_CH7,
    OUT_RSSI_LQ_CHANNEL_CH8,
    OUT_RSSI_LQ_CHANNEL_CH9,
    OUT_RSSI_LQ_CHANNEL_CH10,
    OUT_RSSI_LQ_CHANNEL_CH11,
    OUT_RSSI_LQ_CHANNEL_CH12,
    OUT_RSSI_LQ_CHANNEL_CH13,
    OUT_RSSI_LQ_CHANNEL_CH14,
    OUT_RSSI_LQ_CHANNEL_CH15,
    OUT_RSSI_LQ_CHANNEL_CH16,
    OUT_RSSI_LQ_CHANNEL_NUM,
} RX_OUT_RSSI_LQ_CHANNEL_ENUM;


typedef enum {
    FAILSAFE_MODE_NO_SIGNAL = 0,
    FAILSAFE_MODE_LOW_THROTTLE,
    FAILSAFE_MODE_AS_CONFIGURED,
    FAILSAFE_MODE_LOW_THROTTLE_ELSE_CENTER,
    FAILSAFE_MODE_CH1CH4_CENTER,
    FAILSAFE_MODE_NUM,
} RX_FAILSAFE_MODE_ENUM;


typedef enum {
    SEND_RC_CHANNELS_OFF = 0,
    SEND_RC_CHANNELS_RCCHANNELSOVERRIDE,
    SEND_RC_CHANNELS_RADIORCCHANNELS,
    SEND_RC_CHANNELS_NUM,
} RX_SEND_RCCHANNELS_ENUM;


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
// ATTENTION: only extend/append, never change sequence
//-------------------------------------------------------

typedef struct
{
    char BindPhrase[6+1];
    uint8_t FrequencyBand;
    uint8_t Mode;
    uint8_t Ortho;

    uint8_t spare[6];
} tCommonSetup; // 16 bytes


typedef struct
{
    uint8_t Power;
    uint8_t Diversity;
    uint8_t ChannelsSource;
    uint8_t ChannelOrder;
    uint8_t InMode;
    uint8_t SerialDestination;
    uint8_t SerialBaudrate;
    uint8_t __SerialLinkMode; // deprecated, substituted by Rx.SerialLinkMode
    uint8_t SendRadioStatus;
    uint8_t Buzzer;
    uint8_t CliLineEnd;

    uint8_t spare[9];
} tTxSetup; // 20 bytes


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
    uint8_t Buzzer;
    uint8_t SendRcChannels;
    uint8_t __RadioStatusMethod; // deprecated
    uint8_t OutLqChannelMode;

    uint8_t spare[7];

    int8_t FailsafeOutChannelValues_Ch1_Ch12[12]; // -120 .. +120
    uint8_t FailsafeOutChannelValues_Ch13_Ch16[4]; // 0,1,2 = -120, 0, +120
} tRxSetup; // 36 bytes


#define SETUP_MARKER_STR      "SetupStartMarker"
#define SETUP_MARKEREND_STR   "!end!"

#define SETUP_CONFIG_LEN      10 // not more, so it's only one char


// user setable parameter values, stored in EEPROM
typedef struct
{
    char MarkerStr[16];
    uint32_t Version;
    uint16_t Layout;

    // parameters common to both Tx and Rx
    // deprecated
    char __BindPhrase[6+1];
    uint8_t __FrequencyBand;
    uint8_t __Mode;

    uint8_t _ConfigId; // strange name to avoid mistake

    uint8_t spare[6];

    // parameters specific to Rx, can be changed on the fly
    // for transmitters this is populated upon first connection, see SetupMetaData.rx_available mechanism
    tRxSetup Rx;

    // parameters specific to Tx, can be changed on the fly
    // not used by receivers
    tTxSetup Tx[SETUP_CONFIG_LEN];

    // parameters common to both Tx and Rx
    // cannot be changed on the fly, loss of connection will happen, needs restart/reconnect
    tCommonSetup Common[SETUP_CONFIG_LEN];

    char MarkerEnd[8];
} tSetup;


typedef struct
{
    uint16_t FrequencyBand_allowed_mask;
    uint16_t Mode_allowed_mask;
    uint16_t Ortho_allowed_mask;

    char Tx_Power_optstr[44+1];
    uint16_t Tx_Diversity_allowed_mask;
    uint16_t Tx_ChannelsSource_allowed_mask;
    uint16_t Tx_InMode_allowed_mask;
    uint16_t Tx_SerialDestination_allowed_mask;
    uint16_t Tx_Buzzer_allowed_mask;

    char Rx_Power_optstr[44+1];
    uint16_t Rx_Diversity_allowed_mask;
    uint16_t Rx_OutMode_allowed_mask;
    uint16_t Rx_Buzzer_allowed_mask;

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
    uint8_t LoraConfigIndex;
    uint32_t FlrcSyncWord;
    int8_t Power_dbm;
    uint8_t FrequencyBand;
    // helper
    bool is_lora;
    bool modeIsLora(void) { return is_lora; }
} tSxGlobalConfig;


typedef struct 
{
    uint8_t Num;
    uint32_t Seed;
    uint8_t FrequencyBand;
    uint8_t Ortho;
    uint8_t Except;
    uint16_t FrequencyBand_allowed_mask; // copy of SetupMetaData for sx1, is modified for sx2
} tFhssGlobalConfig;


typedef struct
{
    uint8_t ConfigId; // we take a copy at startup to avoid confusion

    uint8_t FrequencyBand;
    uint8_t Mode;

    tSxGlobalConfig Sx;
    tSxGlobalConfig Sx2;
    uint8_t send_frame_tmo_ms;

    uint16_t FrameSyncWord;

    tFhssGlobalConfig Fhss;
    tFhssGlobalConfig Fhss2;

    uint16_t LQAveragingPeriod;

    uint32_t SerialBaudrate;

    uint16_t frame_rate_ms;
    uint16_t frame_rate_hz;
    uint16_t connect_tmo_systicks;
    uint16_t connect_listen_hop_cnt;

    uint8_t Diversity; // snapshot of Setup's Diversity at startup
    bool ReceiveUseAntenna1;
    bool ReceiveUseAntenna2;
    bool TransmitUseAntenna1;
    bool TransmitUseAntenna2;

    bool UseMbridge;
    bool UseCrsf;
    bool UseIn;
} tGlobalConfig;


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#define SEND_FRAME_TMO_MS         Config.send_frame_tmo_ms

#define CONNECT_TMO_SYSTICKS      Config.connect_tmo_systicks
#define CONNECT_LISTEN_HOP_CNT    Config.connect_listen_hop_cnt


#endif // SETUP_TYPES_H
