//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// CRSF Protocol
// The CRSF protocol details have been thankfully released by TBS
// So, this info is openly available.
// See e.g.
// betaflight:
// https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf_protocol.h
// https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c
// ardupilot:
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RCProtocol/AP_RCProtocol_CRSF.h
//
// CRSF frame format:
// address len type payload crc
// len is the length including type, payload, crc
// crc includes type, payload
//
// baudrate:
// is reported inconsistently across the various resources
// Ardupilot: seems to use 416666 for receiver -> autopilot
// OpenTx: seems to use 400000 for radio -> tx module
//********************************************************
#ifndef CRSF_PROTOCOL_H
#define CRSF_PROTOCOL_H
#pragma once


#ifndef PACKED
  #define CRSF_PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) // that's for __GNUC__
#else
  #define CRSF_PACKED  PACKED
#endif


typedef enum {
  CRSF_ADDRESS_BROADCAST            = 0x00,
  CRSF_ADDRESS_FLIGHT_CONTROLLER    = 0xC8,
  CRSF_ADDRESS_RADIO                = 0xEA, // radio module -> radio transmitter
  CRSF_ADDRESS_RECEIVER             = 0xEC,
  CRSF_ADDRESS_MODULE               = 0xEE, // radio transmitter -> radio module

  CRSF_SYNC_BYTE                    = 0xC8,
} CRSF_ADDRESS_ENUM;


typedef enum {
  CRSF_FRAME_ID_GPS                 = 0x02,
  CRSF_FRAME_ID_VARIO               = 0x07,
  CRSF_FRAME_ID_BATTERY             = 0x08,
  CRSF_FRAME_ID_LINK_STATISTICS     = 0x14,
  CRSF_FRAME_ID_CHANNELS            = 0x16,
  CRSF_FRAME_ID_LINK_STATISTICS_RX  = 0x1C,
  CRSF_FRAME_ID_LINK_STATISTICS_TX  = 0x1D,
  CRSF_FRAME_ID_ATTITUDE            = 0x1E,
  CRSF_FRAME_ID_FLIGHT_MODE         = 0x21,
  CRSF_FRAME_ID_PING_DEVICES        = 0x28,
  CRSF_FRAME_ID_DEVICE_INFO         = 0x29,
  CRSF_FRAME_ID_REQUEST_SETTINGS    = 0x2A,
  CRSF_FRAME_ID_COMMAND             = 0x32,
  CRSF_FRAME_ID_RADIO               = 0x3A,
} CRSF_FRAME_ID_ENUM;


typedef enum {
  CRSF_POWER_0_mW = 0,
  CRSF_POWER_10_mW,
  CRSF_POWER_25_mW,
  CRSF_POWER_100_mW,
  CRSF_POWER_500_mW,
  CRSF_POWER_1000_mW,
  CRSF_POWER_2000_mW,
  CRSF_POWER_250_mW,
  CRSF_POWER_50_mW
} CRSF_POWER_ENUM;


#define CRSF_CHANNELPACKET_SIZE  22

// do not confuse with sbus, it is similar to sbus packet format, but not sbus values
typedef union {
  uint8_t c[CRSF_CHANNELPACKET_SIZE];
  CRSF_PACKED(
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
} tCrsfChannelBuffer;


/* 0x14 Link Statistics

  uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
  uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
  uint8_t Uplink Package success rate / Link quality ( % )
  int8_t Uplink SNR ( db )
  uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
  uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
  uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 250mW )
  uint8_t Downlink RSSI ( dBm * -1 )
  uint8_t Downlink package success rate / Link quality ( % )
  int8_t Downlink SNR ( db )
*/
CRSF_PACKED(
typedef struct
{
  uint8_t uplink_rssi1;
  uint8_t uplink_rssi2;
  uint8_t uplink_LQ;
  int8_t uplink_snr;
  uint8_t active_antenna;
  uint8_t mode;
  uint8_t uplink_transmit_power;
  uint8_t downlink_rssi;
  uint8_t downlink_LQ;
  int8_t downlink_snr;
}) tCrsfLinkStatistics;

#define CRSF_LINK_STATISTICS_LEN  10


/* 0x1D Link Statistics TX

  uint8_t Uplink RSSI ( dBm * -1 )
  uint8_t Uplink RSSI ( % )
  uint8_t Uplink Package success rate / Link quality ( % )
  int8_t Uplink SNR ( db )
  uint8_t Downlink RF Power ( db )
  uint8_t Uplink FPS ( FPS / 10 )
*/
CRSF_PACKED(
typedef struct
{
  uint8_t uplink_rssi;
  uint8_t uplink_rssi_percent;
  uint8_t uplink_LQ;
  int8_t uplink_snr;
  uint8_t downlink_transmit_power;
  uint8_t uplink_fps;
}) tCrsfLinkStatisticsTx;

#define CRSF_LINK_STATISTICS_TX_LEN  6


CRSF_PACKED(
typedef struct
{
  uint8_t downlink_rssi;
  uint8_t downlink_rssi_percent;
  uint8_t downlink_LQ;
  int8_t downlink_snr;
  uint8_t uplink_transmit_power;
}) tCrsfLinkStatisticsRx;

#define CRSF_LINK_STATISTICS_RX_LEN  5



#endif // CRSF_PROTOCOL_H
