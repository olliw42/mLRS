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


#define VERSION             317 // leading zero makes it octal!
#define VERSIONONLYSTR      "v0.3.17"
#define SETUPLAYOUT         2   // this should be changed then Setup struct and/or serial changes


//-------------------------------------------------------
// Selectable System Configs
//-------------------------------------------------------

// un-comment to enable Rx module to go into bind mode after power up
//#define RX_BIND_MODE_AFTER_POWERUP


//-------------------------------------------------------
// Setup
//-------------------------------------------------------
// these defines determine the default values for the setup parameters

// un-comment to force setup to the below defines, else setup is taken from EEPROM
//#define SETUP_FORCE_COMMON_CONF


#define CPOWER                          0 // 0: minimum power, 255: maximum power
//#define CPOWER                          RFPOWER_DEFAULT

#define DIVERSITY                       0


#define SETUP_TX_SERIAL_DESTINATION     0 // 0: serial port, 1: mBridge, 2: serial2 (BT/ESP) port

#define SETUP_TX_CHANNELS_SOURCE        3 // 0: none, 1: mBridge (pin5), 2: In (In or pin1), 3: Crsf (pin5)

#define SETUP_TX_CHANNEL_ORDER          CHANNEL_ORDER_ETAR

#define SETUP_TX_IN_MODE                0 // 0: IN_CONFIG_SBUS, 1: IN_CONFIG_SBUS_INVERTED

#define SETUP_TX_SERIAL_BAUDRATE        3 // 0: 9600, 1: 19200, 2: 38400, 3: 57600, 4: 115200

#define SETUP_TX_POWER                  CPOWER

#define SETUP_TX_DIVERSITY              DIVERSITY // 0: default, 1: ANTENNA 1 if diversity available, 2: ANTENNA 2 if diversity available

#define SETUP_TX_SERIAL_LINK_MODE       0 // 0: transparent, 1: mavlink

#define SETUP_TX_SEND_RADIO_STATUS      0 // 0: off, 1: 1 Hz

#define SETUP_TX_BUZZER                 0 // 0: off, 1: LP, 2: rxLQ
#define SETUP_TX_CLI_LINE_END           0 // 0: CR, 1: LF, 2: CRLF


#define SETUP_RX_CHANNEL_ORDER          CHANNEL_ORDER_AETR

#define SETUP_RX_OUT_MODE               1 // 0: OUT_CONFIG_SBUS, 1: OUT_CONFIG_CRSF, 2: OUT_CONFIG_SBUS_INVERTED

#define SETUP_RX_FAILSAFE_MODE          0 // 0: no signal 1: low throttle, 4: CH1-CH4 center signal

#define SETUP_RX_SERIAL_BAUDRATE        3 // 0: 9600, 1: 19200, 2: 38400, 3: 57600, 4: 115200, 5: 230400

#define SETUP_RX_POWER                  CPOWER

#define SETUP_RX_DIVERSITY              DIVERSITY // 0: default, 1: ANTENNA 1 if diversity available, 2: ANTENNA 2 if diversity available

#define SETUP_RX_SERIAL_LINK_MODE       0 // 0: transparent, 1: mavlink

#define SETUP_RX_SEND_RADIO_STATUS      1 // 0: off, 1: ardu_1, 2: px4 aka "brad"
#define SETUP_RX_SEND_RC_CHANNELS       0 // 0: off, 1: RC_CHANNEL_OVERRIDE, 2: RC_CHANNELS

#define SETUP_RX_OUT_RSSI_CHANNEL       0 // 0: off, 5: CH5, 12: CH12, note: CH13 - CH16 are 3-way and not suitable

#define SETUP_RX_BUZZER                 0 // 0: off, 1: LP


#define BIND_PHRASE                     "mlrs.0" // string of 6 characters, allowed are 'a'-'z','0'-'9','_','-','#','.'


#define SETUP_MODE                      MODE_50HZ
//#define SETUP_MODE                      MODE_31HZ
//#define SETUP_MODE                      MODE_19HZ


//#define SETUP_RF_BAND                    SETUP_FREQUENCY_BAND_915_MHZ_FCC
#define SETUP_RF_BAND                    SETUP_FREQUENCY_BAND_868_MHZ // that's my privilege :)


//-------------------------------------------------------
// System Configs
//-------------------------------------------------------

#define MODE_50HZ_SEND_FRAME_TMO        10 // just needs to be larger than toa, not critical
#define MODE_31HZ_SEND_FRAME_TMO        15 // just needs to be larger than toa, not critical
#define MODE_19HZ_SEND_FRAME_TMO        25 // just needs to be larger than toa, not critical

#define FHSS_NUM_BAND_868_MHZ               6 // it's a very narrow band
#define FHSS_NUM_BAND_915_MHZ_FCC_19HZ_MODE 12 // was 24, but a cycle takes then 1.3 sec! would need long disconnect
#define FHSS_NUM_BAND_915_MHZ_FCC           12 // was 24, but a cycle takes then 1.3 sec! would need long disconnect
#define FHSS_NUM_BAND_2P4_GHZ_19HZ_MODE     12 // was 24, but a cycle takes then 1.3 sec! would need long disconnect
#define FHSS_NUM_BAND_2P4_GHZ_31HZ_MODE     18
#define FHSS_NUM_BAND_2P4_GHZ               24

#define FRAME_TX_RX_LEN                 91 // we currently only support equal len

#define CONNECT_TMO_MS                  750 // was 500, make longer to better handle 19 Hz mode

#define CONNECT_SYNC_CNT                5

#define LQ_AVERAGING_MS                 1000


#define TX_SERIAL_BAUDRATE              19200 // will be overwritten by setup
#define TX_SERIAL_TXBUFSIZE             512
#define TX_SERIAL_RXBUFSIZE             2048 // MissionPlanner is really rude

#define TX_MBRIDGE_TXBUFSIZE            512
#define TX_MBRIDGE_RXBUFSIZE            2048 // MissionPlanner is really rude

#define RX_SERIAL_BAUDRATE              19200 // will be overwritten by setup
#define RX_SERIAL_TXBUFSIZE             1024
#define RX_SERIAL_RXBUFSIZE             2048 // ArduPilot also can be rude

#define TX_COM_BAUDRATE                 115200
#define TX_COM_TXBUFSIZE                1024 // cli needs it
#define TX_COM_RXBUFSIZE                512


#define RX_BIND_MODE_AFTER_POWERUP_TIME_SEC   60


#endif // COMMON_CONFIG_H
