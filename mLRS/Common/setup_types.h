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
  MODE_50HZ = 0,
  MODE_19HZ,
  MODE_NUM,
} MODE_ENUM;


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
  CHANNEL_ORDER_AETR = 0, // most common
  CHANNEL_ORDER_TAER, // spektrum/graupner/jr
  CHANNEL_ORDER_ETAR,
  CHANNEL_ORDER_NUM,
} CHANNEL_ORDER_ENUM;


typedef enum {
  SERIAL_LINK_MODE_TRANSPARENT = 0,
  SERIAL_LINK_MODE_MAVLINK,
  SERIAL_LINK_MODE_NUM,
} SERIAL_LINK_MODE_TRANSPARENT_ENUM;


typedef enum {
  SEND_RADIO_STATUS_OFF = 0,
  SEND_RADIO_STATUS_ON,
  SEND_RADIO_STATUS_NUM,
} SEND_RADIO_STATUS_ENUM;


typedef enum {
  DIVERSITY_DEFAULT = 0,
  DIVERSITY_ANTENNA1, // antenna 1 if diversity available
  DIVERSITY_ANTENNA2, // antenna 2 if diversity available
  DIVERSITY_NUM,
} DIVERSITY_ENUM;


typedef enum {
  IN_CONFIG_SBUS = 0,
  IN_CONFIG_SBUS_INVERTED,
  IN_CONFIG_NUM,
} TX_IN_CONFIG_ENUM;


typedef enum {
  OUT_CONFIG_SBUS = 0,
  OUT_CONFIG_CRSF,
  OUT_CONFIG_NUM,
} RX_OUT_CONFIG_ENUM;


typedef enum {
  FAILSAFE_MODE_NO_SIGNAL = 0,
  FAILSAFE_MODE_CH1CH4_CENTER_SIGNAL,
  FAILSAFE_MODE_NUM,
} RX_FAILSAFE_MODE_ENUM;


//-------------------------------------------------------
// Config Enums
//-------------------------------------------------------
// numbers must match config arrays in sx drivers !!

typedef enum {
    SX128x_LORA_CONFIG_BW800_SF5_CRLI4_5 = 0,
    SX128x_LORA_CONFIG_BW800_SF7_CRLI4_5,
    SX128x_LORA_CONFIG_NUM,
} SX128x_LORA_CONFIG_ENUM;


typedef enum {
    SX127x_LORA_CONFIG_BW500_SF6_CR4_5 = 0,
    SX127x_LORA_CONFIG_NUM,
} SX127x_LORA_CONFIG_ENUM;


typedef enum {
    SX126x_LORA_CONFIG_BW500_SF6_CR4_5 = 0,
    SX126x_LORA_CONFIG_NUM,
} SX126x_LORA_CONFIG_ENUM;


//-------------------------------------------------------
// Setup and Config Types
//-------------------------------------------------------

typedef struct
{
  uint16_t Power;
  uint16_t Diversity;
  uint16_t ChannelsSource;
  uint16_t ChannelOrder;
  uint16_t InMode;
  uint16_t SerialDestination;
  uint16_t SerialBaudrate_bytespersec; // baudrate / 10
  uint16_t SerialLinkMode;
  uint16_t SendRadioStatus;
} tTxSetup;


typedef struct
{
  uint16_t Power;
  uint16_t Diversity;
  uint16_t ChannelOrder;
  uint16_t OutMode;
  uint16_t FailsafeMode;
  uint16_t SerialBaudrate_bytespersec; // baudrate / 10
  uint16_t SerialLinkMode;
  uint16_t SendRadioStatus;
} tRxSetup;


// user setable parameter values, stored in EEPROM
typedef struct
{
  // parameters common to both Tx and Rx
  // cannot be changed on the fly, loss of connection will happen, need restart/reconnect
  uint32_t BindDblWord;
  uint16_t Mode;

  // parameters specific to Rx, can be changed on the fly
  tRxSetup Rx;

  // parameters specific to Tx, can be changed on the fly
  tTxSetup Tx;
} tSetup;


// global configuration values, not stored in EEPROM
// can be derived from setup parameters, from defines, or otherwise
typedef struct
{
  uint8_t LoraConfigIndex;
  uint8_t lora_send_frame_tmo;

  uint16_t FrameSyncWord;
  uint16_t FhssNum;
  uint32_t FhssSeed;

  uint16_t LQAveragingPeriod;

  uint8_t Power;

  uint16_t frame_rate_ms;
  uint16_t frame_rate_hz;
  uint16_t connect_tmo_systicks;
  uint16_t connect_listen_hop_cnt;

  bool UseAntenna1;
  bool UseAntenna2;
} tGlobalConfig;


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#define SEND_FRAME_TMO          Config.lora_send_frame_tmo

#define CONNECT_TMO_SYSTICKS    Config.connect_tmo_systicks
#define CONNECT_LISTEN_HOP_CNT  Config.connect_listen_hop_cnt

#ifdef DEVICE_HAS_DIVERSITY
#define IF_ANTENNA1(x)          if (Config.UseAntenna1) { x; }
#define IF_ANTENNA2(x)          if (Config.UseAntenna2) { x; }
#define USE_ANTENNA1            (Config.UseAntenna1)
#define USE_ANTENNA2            (Config.UseAntenna2)
#else
#define IF_ANTENNA1(x)          x;
#define IF_ANTENNA2(x)
#define USE_ANTENNA1            true
#define USE_ANTENNA2            false
#endif


#endif // SETUP_TYPES_H
