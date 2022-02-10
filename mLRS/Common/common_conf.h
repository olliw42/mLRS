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


#define VERSION             0025
#define VERSIONONLYSTR      "v0.0.25"
#define SETUPLAYOUT         001       // this should be changed then Setup struct and/or serial changes


//-------------------------------------------------------
// Setup
//-------------------------------------------------------

#define POWER                           POWER_m18_DBM
//#define POWER                           POWER_0_DBM
//#define POWER                           POWER_12p5_DBM

#define DIVERSITY                       0


#define SETUP_TX_SERIAL_DESTINATION     1 // 0: serial port, 1: mBridge

#define SETUP_TX_CHANNELS_SOURCE        1 // 0: none, 1: mBridge (pin5), 2: SPort In (pin1), 3: Crsf (pin5)

#define SETUP_TX_CHANNEL_ORDER          CHANNEL_ORDER_ETAR

#define SETUP_TX_POWER                  POWER

#define SETUP_TX_SEND_RADIO_STATUS      0 // 0: off, 1: RADIO_STATUS, 2: RADIO_STATUS_V2

#define SETUP_TX_DIVERSITY              DIVERSITY // 0: default, 1: ANTENNA 1 if diversity available, 2: ANTENNA 2 if diversity available


#define SETUP_RX_CHANNEL_ORDER          CHANNEL_ORDER_AETR

#define SETUP_RX_OUT_MODE               OUT_CONFIG_CRSF // OUT_CONFIG_SBUS, OUT_CONFIG_CRSF

#define SETUP_RX_FAILSAFE_MODE          1 // 0: no signal 1: CH1-CH4 center signal

#define SETUP_RX_POWER                  POWER

#define SETUP_RX_SERIAL_BAUDRATE        57600

#define SETUP_RX_SEND_RADIO_STATUS      0 // 0: off, 1: RADIO_STATUS, 2: RADIO_STATUS_V2

#define SETUP_RX_DIVERSITY              DIVERSITY // 0: default, 1: ANTENNA 1 if diversity available, 2: ANTENNA 2 if diversity available


#define BIND_DBLWORD                    0x12344281


//-------------------------------------------------------
// System Configs
//-------------------------------------------------------

#define FRAME_RATE_MS                   20 // 500 // 20 // 50 Hz
#define FHSS_NUM                        24


#define FRAME_TX_RX_LEN                 91 // we currently only support equal len

#define CONNECT_TMO_MS                  500

#define CONNECT_SYNC_CNT                5

#define LQ_AVERAGING_MS                 1000



//-------------------------------------------------------
// Derived Defines
//-------------------------------------------------------

#if (SETUP_TX_SERIAL_DESTINATION == 1) || (SETUP_TX_CHANNELS_SOURCE == 1)
#define USE_MBRIDGE
#endif


#endif // COMMON_CONFIG_H
