//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// common config
//*******************************************************
#ifndef COMMON_CONFIG_H
#define COMMON_CONFIG_H
#pragma once


#define VERSION             10308 // leading zero makes it octal!
#define VERSIONONLYSTR      "v1.3.08"
#define SETUPLAYOUT         10304 // this should be changed then Setup struct and/or serial changes


//-------------------------------------------------------
// Selectable System Configs
//-------------------------------------------------------
// user can modify them to their liking

// un-comment to enable Rx module to go into bind mode after power up
//#define RX_BIND_MODE_AFTER_POWERUP


// Development features. Note: They are offered for testing, but they are not for production
//currently none


//-------------------------------------------------------
// Setup
//-------------------------------------------------------
// these defines determine the default values for the setup parameters
// user can modify them to their liking

// un-comment to force setup to the below defines, else setup is taken from EEPROM
//#define SETUP_FORCE_COMMON_CONF


#define CPOWER                          RFPOWER_DEFAULT
//#define CPOWER                          0 // 0: minimum power, 255: maximum power

#define DIVERSITY                       0


#define SETUP_TX_SERIAL_DESTINATION     0 // 0: serial port, 1: serial2 (BT/ESP) port, 2: mBridge

#define SETUP_TX_CHANNELS_SOURCE        1 // 0: none, 1: Crsf (pin5), 2: In (In or pin1), 3: mBridge (pin5)

#define SETUP_TX_CHANNEL_ORDER          CHANNEL_ORDER_AETR

#define SETUP_TX_IN_MODE                0 // 0: IN_CONFIG_SBUS, 1: IN_CONFIG_SBUS_INVERTED

#define SETUP_TX_SERIAL_BAUDRATE        4 // 0: 9600, 1: 19200, 2: 38400, 3: 57600, 4: 115200, 5: 230400

#define SETUP_TX_POWER                  CPOWER

#define SETUP_TX_DIVERSITY              DIVERSITY // 0: default, 1: ANTENNA 1 if diversity available, 2: ANTENNA 2 if diversity available

#define SETUP_TX_SEND_RADIO_STATUS      1 // 0: off, 1: 1 Hz
#define SETUP_TX_MAV_COMPONENT          1 // 0: off, 1: enabled

#define SETUP_TX_BUZZER                 0 // 0: off, 1: LP, 2: rxLQ


#define SETUP_RX_CHANNEL_ORDER          CHANNEL_ORDER_AETR

#define SETUP_RX_OUT_MODE               1 // 0: OUT_CONFIG_SBUS, 1: OUT_CONFIG_CRSF, 2: OUT_CONFIG_SBUS_INVERTED

#define SETUP_RX_FAILSAFE_MODE          0 // 0: no signal 1: low throttle, 4: CH1-CH4 center signal

#define SETUP_RX_SERIAL_PORT            0 // 0: serial, 1: can

#define SETUP_RX_SERIAL_BAUDRATE        3 // 0: 9600, 1: 19200, 2: 38400, 3: 57600, 4: 115200, 5: 230400

#define SETUP_RX_POWER                  CPOWER

#define SETUP_RX_DIVERSITY              DIVERSITY // 0: default, 1: ANTENNA 1 if diversity available, 2: ANTENNA 2 if diversity available

#define SETUP_RX_SERIAL_LINK_MODE       2 // 0: transparent, 1: mavlink, 2: mavlinkX, 3: mspX

#define SETUP_RX_MAVLINK_SYSTEM_ID      0 // 0: 51, 1: 52, 2: 53, 3: 54, 4: 55  // SiK uses 51, 68
#define SETUP_RX_SEND_RADIO_STATUS      1 // 0: off, 1: ardu_1, 2: px4 aka "brad"
#define SETUP_RX_SEND_RC_CHANNELS       0 // 0: off, 1: RC_CHANNEL_OVERRIDE, 2: RC_CHANNELS

#define SETUP_RX_OUT_RSSI_CHANNEL       0 // 0: off, 5: CH5, 16: CH16
#define SETUP_RX_OUT_LQ_CHANNEL         0 // 0: off, 5: CH5, 16: CH16


#define BIND_PHRASE                     "mlrs.0" // string of 6 characters, allowed are 'a'-'z','0'-'9','_','-','#','.'


#define SETUP_MODE                      MODE_50HZ
//#define SETUP_MODE                      MODE_31HZ
//#define SETUP_MODE                      MODE_19HZ


//#define SETUP_RF_BAND                   SETUP_FREQUENCY_BAND_915_MHZ_FCC
#define SETUP_RF_BAND                   SETUP_FREQUENCY_BAND_868_MHZ // that's my privilege :)


#define SETUP_RF_ORTHO                  0 // 0: off, 1: 1/3, 2: 2/3, 3: 3/3


//-------------------------------------------------------
// User-adjustable System Configs
//-------------------------------------------------------

#define RX_GCS_SYSTEM_ID                255 // default of MissionPlanner, QGC

#define RX_DRONECAN_PREFERRED_NODE_ID   68


//-------------------------------------------------------
// System Configs
//-------------------------------------------------------
// sets various internal values
// user must not modify

#define MODE_50HZ_SEND_FRAME_TMO_MS           10 // just needs to be larger than toa, not critical
#define MODE_31HZ_SEND_FRAME_TMO_MS           15 // just needs to be larger than toa, not critical
#define MODE_19HZ_SEND_FRAME_TMO_MS           25 // just needs to be larger than toa, not critical
#define MODE_FLRC_111HZ_SEND_FRAME_TMO_MS     7  // just needs to be larger than toa, not critical
#define MODE_FSK_50HZ_SEND_FRAME_TMO_MS       10 // just needs to be larger than toa, not critical


#define FHSS_NUM_433_MHZ                2  // 2 since 1 is needed for bind
#define FHSS_NUM_70_CM_HAM              18 // to match 2.4 GHz at 31 Hz
#define FHSS_NUM_70_CM_HAM_19HZ         12 // to match 2.4 GHz at 19 Hz
#define FHSS_NUM_868_MHZ                6  // it's a very narrow band
#define FHSS_NUM_915_MHZ_FCC            25 // https://www.ecfr.gov/current/title-47/chapter-I/subchapter-A/part-15/subpart-C/subject-group-ECFR2f2e5828339709e/section-15.247#p-15.247(a)(1)(i)
#define FHSS_NUM_866_MHZ_IN             3  // 3 since 1 is needed for bind
#define FHSS_NUM_2P4_GHZ                24
#define FHSS_NUM_2P4_GHZ_31HZ           18
#define FHSS_NUM_2P4_GHZ_19HZ           12 // was 24, but a cycle takes then 1.3 sec! would need long disconnect

#define FRAME_TX_RX_LEN                 91 // we currently only support equal len

#define CONNECT_TMO_MS                  1250 // time to disconnect, was 500, then 750 to better handle 19 Hz mode, now 1250

#define CONNECT_SYNC_CNT                5 // number of packets to connect

#define LQ_AVERAGING_MS                 1000


#define TX_SERIAL_BAUDRATE              115200 // will be overwritten by setup
#define TX_SERIAL_TXBUFSIZE             512
#define TX_SERIAL_RXBUFSIZE             2048 // MissionPlanner is really rude

#define TX_MBRIDGE_TXBUFSIZE            512
#define TX_MBRIDGE_RXBUFSIZE            2048 // MissionPlanner is really rude

#define RX_SERIAL_BAUDRATE              57600 // will be overwritten by setup
#define RX_SERIAL_TXBUFSIZE             1024
#define RX_SERIAL_RXBUFSIZE             2048 // ArduPilot also can be rude

#define TX_COM_BAUDRATE                 115200
#define TX_COM_TXBUFSIZE                512 // 2048 // cli needs more than 1024   since 4.2.2025 we have cli chunks
#define TX_COM_TXBUFSIZE_SMALL          256 // 512 // we don't have enough RAM
#define TX_COM_TXBUFSIZE_LARGE          2048 // we have plenty and can easily afford
#define TX_COM_RXBUFSIZE                512


#define RX_BIND_MODE_AFTER_POWERUP_TIME_SEC   60


#endif // COMMON_CONFIG_H

