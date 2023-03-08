//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// CRSF Protocol
//
// The CRSF protocol details have been thankfully released by TBS
// So, this info is openly available.
// See e.g.
// BetaFlight:
// https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf_protocol.h
// https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c
// ArduPilot:
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RCProtocol/AP_RCProtocol_CRSF.h
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RCTelemetry/AP_CRSF_Telem.h
// ExpressLRS:
// https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/CrsfProtocol/crsf_protocol.h
// https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol

// CRSF frame format:
//   address
//   len
//   type/frame_id
//   payload
//   crc8
// len is the length including type, payload, crc8
// crc8 includes type, payload
// maximal frame length is 64 bytes (ELRS says 66 bytes!) correct???
// maximal payload length is thus 60 bytes (ELRS says 62 bytes!)
//
// baudrate:
// is reported inconsistently across the various resources
// Ardupilot: seems to use 416666 for receiver -> autopilot
// OpenTx: seems to use 400000 for radio -> tx module
//
// depending on type/frame_id, the payload can have additional sub-struture
// type/frame_id = CRSF_FRAME_ID_COMMAND (0x32):
//   destination address
//   source address
//   sub command identifier (0x10)
//   command
//   data
//   command_crc8
//
//*******************************************************
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
    CRSF_ADDRESS_TRANSMITTER_MODULE   = 0xEE, // radio transmitter -> radio module

    CRSF_OPENTX_SYNC                  = 0xC8, // is this an otx specific thing? called CRSF_SYNC_BYTE in CleanFlight
} CRSF_ADDRESS_ENUM;


typedef enum {
    CRSF_FRAME_ID_GPS                 = 0x02,
    CRSF_FRAME_ID_VARIO               = 0x07,
    CRSF_FRAME_ID_BATTERY             = 0x08,
    CRSF_FRAME_ID_BARO_ALTITUDE       = 0x09,
    CRSF_FRAME_ID_LINK_STATISTICS     = 0x14,
    CRSF_FRAME_ID_CHANNELS            = 0x16,
    CRSF_FRAME_ID_LINK_STATISTICS_RX  = 0x1C,
    CRSF_FRAME_ID_LINK_STATISTICS_TX  = 0x1D,
    CRSF_FRAME_ID_ATTITUDE            = 0x1E,
    CRSF_FRAME_ID_FLIGHT_MODE         = 0x21,
    CRSF_FRAME_ID_PING_DEVICES        = 0x28,
    CRSF_FRAME_ID_DEVICE_INFO         = 0x29,
    CRSF_FRAME_ID_REQUEST_SETTINGS    = 0x2A,
    CRSF_FRAME_ID_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAME_ID_PARAMETER_READ      = 0x2C,
    CRSF_FRAME_ID_PARAMETER_WRITE     = 0x2D,
    CRSF_FRAME_ID_COMMAND             = 0x32,
    CRSF_FRAME_ID_RADIO               = 0x3A,

    CRSF_FRAME_ID_KISS_REQUEST        = 0x78,
    CRSF_FRAME_ID_KISS_RESPONSE       = 0x79,
    CRSF_FRAME_ID_MSP_REQUSET         = 0x7A,
    CRSF_FRAME_ID_MSP_RESPONSE        = 0x7B,
    CRSF_FRAME_ID_MSP_WRITE           = 0x7C,

    CRSF_FRAME_ID_AP_CUSTOM_TELEM_LEGACY = 0x7F,
    CRSF_FRAME_ID_AP_CUSTOM_TELEM     = 0x80,

    CRSF_FRAME_ID_MBRIDGE_TO_MODULE   = 0x81, // radio -> tx module
    CRSF_FRAME_ID_MBRIDGE_TO_RADIO    = 0x82, // tx module -> radio
} CRSF_FRAME_ID_ENUM;


typedef enum {
    CRSF_COMMAND_ID                   = 0x10,
} CRSF_COMMAND_ID_ENUM;


typedef enum {
    CRSF_COMMAND_MODEL_SELECT_ID      = 0x05,
} CRSF_COMMAND_ENUM;


// SubType IDs for CRSF_FRAME_ID_AP_CUSTOM_TELEM
typedef enum {
    CRSF_AP_CUSTOM_TELEM_TYPE_SINGLE_PACKET_PASSTHROUGH = 0xF0,
    CRSF_AP_CUSTOM_TELEM_TYPE_STATUS_TEXT = 0xF1,
    CRSF_AP_CUSTOM_TELEM_TYPE_MULTI_PACKET_PASSTHROUGH = 0xF2,
}CRSF_AP_CUSTOM_TELEM_TYPE_ENUM;


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


typedef enum {
    CRSF_RFMODE_4_HZ = 0,
    CRSF_RFMODE_50_HZ,
    CRSF_RFMODE_150_HZ,
    CRSF_RFMODE_250_HZ,
} CRSF_RFMODE_ENUM;


//-- Frame header

// final crc8 included in payload or cmd_data, hence these fields are one byte longer
CRSF_PACKED(
typedef struct
{
    uint8_t address;
    uint8_t len;
    uint8_t frame_id;
    CRSF_PACKED(union {
        uint8_t payload[64 - 4 + 1]; // +1 for crc
        CRSF_PACKED(struct {
            uint8_t cmd_dest_adress;
            uint8_t cmd_src_adress;
            uint8_t cmd_id;
            uint8_t cmd;
            uint8_t cmd_data[64 - 4 - 4 + 1]; // +1 for crc
        });
    });
}) tCrsfFrameHeader;


//-- Channel frame

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


//-- Link statistics frames

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
    uint8_t uplink_rssi1;               // OpenTX -> "1RSS"
    uint8_t uplink_rssi2;               // OpenTX -> "2RSS"
    uint8_t uplink_LQ;                  // OpenTx -> "RQly"
    int8_t uplink_snr;                  // OpenTx -> "RSNR"
    uint8_t active_antenna;             // OpenTx -> "ANT"
    uint8_t mode;                       // OpenTx -> "RFMD"
    uint8_t uplink_transmit_power;      // OpenTx -> "TPw2" ?? uplink but "T" ??
    uint8_t downlink_rssi;              // OpenTx -> "TRSS"
    uint8_t downlink_LQ;                // OpenTx -> "TQly"
    int8_t downlink_snr;                // OpenTx -> "TSNR"
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
    uint8_t uplink_rssi_percent;        // OpenTx -> "TRSP"
    uint8_t uplink_LQ;
    int8_t uplink_snr;
    uint8_t downlink_transmit_power;    // OpenTx -> "RPWR"
    uint8_t uplink_fps;                 // OpenTx -> "TFPS"
}) tCrsfLinkStatisticsTx;

#define CRSF_LINK_STATISTICS_TX_LEN  6


CRSF_PACKED(
typedef struct
{
    uint8_t downlink_rssi;
    uint8_t downlink_rssi_percent;      // OpenTx -> "RRSP"
    uint8_t downlink_LQ;
    int8_t downlink_snr;
    uint8_t uplink_transmit_power;      // OpenTx -> "TPWR"
}) tCrsfLinkStatisticsRx;

#define CRSF_LINK_STATISTICS_RX_LEN  5


//-- Telemetry data frames

CRSF_PACKED(
typedef struct
{
    int32_t latitude; // degree / 1e7                 // OpenTx -> "GPS"
    int32_t longitude; // degree / 1e7                // OpenTx -> "GPS"
    uint16_t groundspeed; // km/h / 100               // OpenTx -> "GSpd"
    uint16_t gps_heading; // degree / 100             // OpenTx -> "Hdg"
    uint16_t altitude; // meter - 1000m offset        // OpenTx -> "GAlt"
    uint8_t satellites;                               // OpenTx -> "Sats"
}) tCrsfGps;

#define CRSF_GPS_LEN  15


CRSF_PACKED(
typedef struct
{
    int16_t climb_rate; // units ??? m/s / 100 indirectly concluded from otx    // OpenTx -> "VSpd"
}) tCrsfVario;

#define CRSF_VARIO_LEN  2


CRSF_PACKED(
typedef struct
{
    uint16_t voltage; // mV * 100                     // OpenTx -> "RxBt"
    uint16_t current; // mA * 100                     // OpenTx -> "Curr"
    uint8_t capacity[3]; // mAh                       // openTx -> "Capa"
    uint8_t remaining; // percent                     // OpenTx -> "Bat%"
}) tCrsfBattery;

#define CRSF_BATTERY_LEN  8


CRSF_PACKED(
typedef struct
{
    int16_t pitch; // rad * 1e4                       // OpenTx -> "Ptch"
    int16_t roll; // rad * 1e4                        // OpenTx -> "Roll"
    int16_t yaw; // rad * 1e4                         // OpenTx -> "Yaw"
}) tCrsfAttitude;

#define CRSF_ATTITUDE_LEN  6


CRSF_PACKED(
typedef struct
{
    char flight_mode[16]; // null-terminated string   // OpenTx -> "FM"
}) tCrsfFlightMode;

#define CRSF_FLIGHTMODE_LEN  16


CRSF_PACKED(
typedef struct
{
    uint16_t altitude; // units ??? message ???       // OpenTx -> "Alt"
}) tCrsfBaroAltitude;

#define CRSF_BARO_ALTITUDE_LEN  2


//-- Command payload frames

CRSF_PACKED(
typedef struct
{
    uint8_t dest_adress;
    uint8_t src_adress;
    uint8_t cmd_id;
    uint8_t cmd;
    uint8_t model_id;
    uint8_t crc8b;
}) tCrsfCmdModelSelectId;

#define CRSF_COMMAND_MODEL_SELECT_ID_LEN  8


//-- Passthrough payload frames

CRSF_PACKED(
typedef struct
{
    uint8_t sub_type;
    uint16_t packet_type;
    uint32_t data;
}) tCrsfPassthroughSingle;

#define CRSF_PASSTHROUGH_SINGLE_LEN  7


CRSF_PACKED(
typedef struct
{
    uint8_t sub_type;
    uint8_t count;
    CRSF_PACKED(struct {
        uint16_t packet_type;
        uint32_t data;
    }) packet[9];
}) tCrsfPassthroughMulti;

#define CRSF_PASSTHROUGH_MULTI_COUNT_MAX  9


#endif // CRSF_PROTOCOL_H
