//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Setup and configuration list
//*******************************************************
#ifndef SETUP_LIST_H
#define SETUP_LIST_H
#pragma once


#include <stdint.h>
#include "hal/device_conf.h"
#include "setup.h"


//-------------------------------------------------------
// Setup parameter list
//-------------------------------------------------------
// allowed mask:
// 0           = option not available (do not display)
// one bit set = option not selectable (displayed, but maybe grayed out)
// this is not totally satisfying, since non-LIST options cannot be hidden

// common to Tx,Rx
#define SETUP_MSK_MODE                &SetupMetaData.Mode_allowed_mask // this we infer from the hal
#define SETUP_MSK_RFBAND              &SetupMetaData.FrequencyBand_allowed_mask // this we infer from the hal
#define SETUP_MSK_RFORTHO             &SetupMetaData.Ortho_allowed_mask // this we infer from the hal

// for Tx,Rx, options limited depending on hardware, implementation
#define SETUP_MSK_TX_DIVERSITY        &SetupMetaData.Tx_Diversity_allowed_mask // this we generate from the hal
#define SETUP_MSK_RX_DIVERSITY        &SetupMetaData.Rx_Diversity_allowed_mask // this we get from the receiver

// Tx only
#define SETUP_MSK_TX_SER_DEST         &SetupMetaData.Tx_SerialDestination_allowed_mask // this we generate from the hal
#define SETUP_MSK_TX_CH_SOURCE        &SetupMetaData.Tx_ChannelsSource_allowed_mask // this we generate from the hal
#define SETUP_MSK_TX_IN_MODE          &SetupMetaData.Tx_InMode_allowed_mask // this we generate from the hal
#define SETUP_MSK_TX_BUZZER           &SetupMetaData.Tx_Buzzer_allowed_mask // this we generate from the hal
#define SETUP_MSK_TX_WIFIPROT         &SetupMetaData.Tx_WiFiProt_allowed_mask // this we generate from the hal

// Rx only
#define SETUP_MSK_RX_OUT_MODE         &SetupMetaData.Rx_OutMode_allowed_mask // this we get from the receiver
#define SETUP_MSK_RX_SER_PORT         &SetupMetaData.Rx_SerialPort_allowed_mask // this we get from the receiver


// for Tx,Rx, option strings
#define SETUP_OPT_TX_POWER            SetupMetaData.Tx_Power_optstr // this we generate from the hal
#define SETUP_OPT_RX_POWER            SetupMetaData.Rx_Power_optstr // this we get from the receiver

#define SETUP_OPT_DIVERSITY           "enabled,antenna1,antenna2,r:en t:ant1,r:en t:ant2"
#define SETUP_OPT_DIVERSITY_LONGSTR   "enabled,antenna1,antenna2,r:en. t:ant1,r:en. t:ant2" // used e.g. in cli
#define SETUP_OPT_DIVERSITY_DISPSTR   "enabled,antenna1,antenna2,ren ta1,ren ta2" // used in display, 7 chars max

#define SETUP_OPT_CH_ORDER            "AETR,TAER,ETAR"
#define SETUP_OPT_TX_SERIAL_BAUDRATE  "9600,19200,38400,57600,115200,230400"
#define SETUP_OPT_RX_SERIAL_BAUDRATE  "9600,19200,38400,57600,115200,230400"

#ifndef USE_FEATURE_MAVLINKX
#define SETUP_OPT_SERIAL_LINK_MODE          "transp.,mavlink"
#define SETUP_OPT_SERIAL_LINK_MODE_DISPSTR  "transp.,mavlink"
#else
#define SETUP_OPT_SERIAL_LINK_MODE          "transp.,mavlink,mavlinkX,mspX"
#define SETUP_OPT_SERIAL_LINK_MODE_DISPSTR  "transp.,mavlink,mavlnkX,mspX"
#endif

#define SETUP_OPT_MODE                "50 Hz,31 Hz,19 Hz,FLRC,FSK,19 Hz 7x" // used below in LIST_COMMON, also used in e.g. cli
#define SETUP_OPT_MODE_DISPSTR        "50 Hz,31 Hz,19 Hz,FLRC,FSK,19Hz7x" // used in display, 7 chars max, should be 6 chars however

#define SETUP_OPT_RFBAND              "2.4,915 FCC,868,433,70,866 IN,915+2.4,868+2.4" // used below in LIST_COMMON
#define SETUP_OPT_RF_BAND_LONGSTR     "2.4 GHz,915 MHz FCC,868 MHz,433 MHz,70 cm HAM,866 MHz IN,915 MHz + 2.4 GHz,868 MHz + 2.4 GHz" // used e.g. in cli
#define SETUP_OPT_RF_BAND_DISPSTR     "2.4 GHz,915 FCC,868 MHz,433 MHz,70 cm,866 IN,915+2.4,868+2.4" // used in display, 7 chars max


#define MSK_ALL                       nullptr // is converted to UINT16_MAX


#define PARAM_INDEX_MODE              1


// Tx parameters must begin with "Tx "
// Rx parameters must begin with "Rx "
// a parameter name is limited to 16 chars max
// a LIST can have only 16 options at most
// a LIST option string list is limited to 67 chars max
// a LIST option string should not exceed 10 chars
#define SETUP_PARAMETER_LIST_COMMON_BINDPHRASE \
  X( Setup.Common[0].BindPhrase[0], STR6, "Bind Phrase",      "BIND_PHRASE",      0,0,0,"", "", 0)

#define SETUP_PARAMETER_LIST_COMMON_FURTHER \
  X( Setup.Common[0].Mode,          LIST, "Mode",             "MODE",             0,0,0,"", SETUP_OPT_MODE, SETUP_MSK_MODE )\
  X( Setup.Common[0].FrequencyBand, LIST, "RF Band",          "RF_BAND",          0,0,0,"", SETUP_OPT_RFBAND, SETUP_MSK_RFBAND )\
  X( Setup.Common[0].Ortho,         LIST, "RF Ortho",         "RF_ORTHO",         0,0,0,"", "off,1/3,2/3,3/3", SETUP_MSK_RFORTHO )

#define SETUP_PARAMETER_LIST_TX_MAIN \
  X( Setup.Tx[0].Power,             LIST, "Tx Power",         "TX_POWER",         0,0,0,"", SETUP_OPT_TX_POWER, MSK_ALL )\
  X( Setup.Tx[0].Diversity,         LIST, "Tx Diversity",     "TX_DIVERSITY",     0,0,0,"", SETUP_OPT_DIVERSITY, SETUP_MSK_TX_DIVERSITY )\
  X( Setup.Tx[0].ChannelsSource,    LIST, "Tx Ch Source",     "TX_CH_SOURCE",     0,0,0,"", "none,crsf,in,mbridge", SETUP_MSK_TX_CH_SOURCE )\
  X( Setup.Tx[0].ChannelOrder,      LIST, "Tx Ch Order",      "TX_CH_ORDER",      0,0,0,"", SETUP_OPT_CH_ORDER, MSK_ALL )\
  X( Setup.Tx[0].InMode,            LIST, "Tx In Mode",       "TX_IN_MODE",       0,0,0,"", "sbus,sbus inv", SETUP_MSK_TX_IN_MODE )\
  X( Setup.Tx[0].SerialDestination, LIST, "Tx Ser Dest",      "TX_SER_DEST",      0,0,0,"", "serial,serial2,mbridge", SETUP_MSK_TX_SER_DEST )\
  X( Setup.Tx[0].SerialBaudrate,    LIST, "Tx Ser Baudrate",  "TX_SER_BAUD",      0,0,0,"", SETUP_OPT_TX_SERIAL_BAUDRATE, MSK_ALL )\
  X( Setup.Tx[0].SendRadioStatus,   LIST, "Tx Snd RadioStat", "TX_SND_RADIOSTAT", 0,0,0,"", "off,1 Hz", MSK_ALL )\
  X( Setup.Tx[0].MavlinkComponent,  LIST, "Tx Mav Component", "TX_MAV_COMPONENT", 0,0,0,"", "off,enabled", MSK_ALL )\
  X( Setup.Tx[0].PowerSwitchChannel,LIST, "Tx Power Sw Ch",   "TX_POWER_SW_CH",   0,0,0,"", "off,5,6,7,8,9,10,11,12,13,14,15,16", MSK_ALL )\
  X( Setup.Tx[0].Buzzer,            LIST, "Tx Buzzer",        "TX_BUZZER",        0,0,0,"", "off,LP,rxLQ", SETUP_MSK_TX_BUZZER )

#define SETUP_PARAMETER_LIST_TX_ESP \
  X( Setup.Tx[0].WifiProtocol,      LIST, "Tx Wifi Protocol", "TX_WIFI_PROT",     0,0,0,"", "TCP,UDP,BT,UDP STA,BLE", SETUP_MSK_TX_WIFIPROT )\
  X( Setup.Tx[0].WifiChannel,       LIST, "Tx Wifi Channel",  "TX_WIFI_CHANNEL",  0,0,0,"", "1,6,11,13", MSK_ALL )\
  X( Setup.Tx[0].WifiPower,         LIST, "Tx Wifi Power",    "TX_WIFI_POWER",    0,0,0,"", "low,med,max", MSK_ALL )

#if defined USE_ESP_WIFI_BRIDGE_RST_GPIO0 && defined DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE
#define SETUP_PARAMETER_LIST_TX  SETUP_PARAMETER_LIST_TX_MAIN  SETUP_PARAMETER_LIST_TX_ESP
#else
#define SETUP_PARAMETER_LIST_TX  SETUP_PARAMETER_LIST_TX_MAIN
#endif

#define SETUP_PARAMETER_LIST_RX \
  X( Setup.Rx.Power,                LIST, "Rx Power",         "RX_POWER",         0,0,0,"", SETUP_OPT_RX_POWER, MSK_ALL )\
  X( Setup.Rx.Diversity,            LIST, "Rx Diversity",     "RX_DIVERSITY",     0,0,0,"", SETUP_OPT_DIVERSITY, SETUP_MSK_RX_DIVERSITY )\
  X( Setup.Rx.ChannelOrder,         LIST, "Rx Ch Order",      "RX_CH_ORDER",      0,0,0,"", SETUP_OPT_CH_ORDER, MSK_ALL )\
  X( Setup.Rx.OutMode,              LIST, "Rx Out Mode",      "RX_OUT_MODE",      0,0,0,"", "sbus,crsf,sbus inv", SETUP_MSK_RX_OUT_MODE )\
  X( Setup.Rx.FailsafeMode,         LIST, "Rx FailSafe Mode", "RX_FAILSAFE_MODE", 0,0,0,"", "no sig,low thr,by cnf,low thr cnt,ch1ch4 cnt", MSK_ALL )\
  X( Setup.Rx.SerialPort,           LIST, "Rx Ser Port",      "RX_SER_PORT",      0,0,0,"", "serial,can", SETUP_MSK_RX_SER_PORT )\
  X( Setup.Rx.SerialBaudrate,       LIST, "Rx Ser Baudrate",  "RX_SER_BAUD",      0,0,0,"", SETUP_OPT_RX_SERIAL_BAUDRATE, MSK_ALL )\
  X( Setup.Rx.SerialLinkMode,       LIST, "Rx Ser Link Mode", "RX_SER_LNK_MODE",  0,0,0,"", SETUP_OPT_SERIAL_LINK_MODE, MSK_ALL )\
  X( Setup.Rx.MavlinkSystemID,      LIST, "Rx Mav System ID", "RX_MAV_SYSTEM_ID", 0,0,0,"", "51,52,53,54,55", MSK_ALL )\
  X( Setup.Rx.SendRadioStatus,      LIST, "Rx Snd RadioStat", "RX_SND_RADIOSTAT", 0,0,0,"", "off,ardu_1,meth_b", MSK_ALL )\
  X( Setup.Rx.SendRcChannels,       LIST, "Rx Snd RcChannel", "RX_SND_RCCHANNEL", 0,0,0,"", "off,rc override,rc channels", MSK_ALL )\
  X( Setup.Rx.OutRssiChannelMode,   LIST, "Rx Out Rssi Ch",   "RX_OUT_RSSI_CH",   0,0,0,"", "off,5,6,7,8,9,10,11,12,13,14,15,16", MSK_ALL )\
  X( Setup.Rx.OutLqChannelMode,     LIST, "Rx Out LQ Ch",     "RX_OUT_LQ_CH",     0,0,0,"", "off,5,6,7,8,9,10,11,12,13,14,15,16", MSK_ALL )\
  X( Setup.Rx.PowerSwitchChannel,   LIST, "Rx Power Sw Ch",   "RX_POWER_SW_CH",   0,0,0,"", "off,5,6,7,8,9,10,11,12,13,14,15,16", MSK_ALL )\
  \
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[0],  INT8, "Rx FS Ch1", "RX_FS_CH1", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[1],  INT8, "Rx FS Ch2", "RX_FS_CH2", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[2],  INT8, "Rx FS Ch3", "RX_FS_CH3", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[3],  INT8, "Rx FS Ch4", "RX_FS_CH4", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[4],  INT8, "Rx FS Ch5", "RX_FS_CH5", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[5],  INT8, "Rx FS Ch6", "RX_FS_CH6", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[6],  INT8, "Rx FS Ch7", "RX_FS_CH7", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[7],  INT8, "Rx FS Ch8", "RX_FS_CH8", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[8],  INT8, "Rx FS Ch9", "RX_FS_CH9", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[9],  INT8, "Rx FS Ch10", "RX_FS_CH10", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[10], INT8, "Rx FS Ch11", "RX_FS_CH11", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[11], INT8, "Rx FS Ch12", "RX_FS_CH12", 0, -120, 120, "%", "",0 )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[0], LIST, "Rx FS Ch13", "RX_FS_CH13", 0,0,0,"", "-120 %,0 %,120 %", MSK_ALL )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[1], LIST, "Rx FS Ch14", "RX_FS_CH14", 0,0,0,"", "-120 %,0 %,120 %", MSK_ALL )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[2], LIST, "Rx FS Ch15", "RX_FS_CH15", 0,0,0,"", "-120 %,0 %,120 %", MSK_ALL )\
  X( Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[3], LIST, "Rx FS Ch16", "RX_FS_CH16", 0,0,0,"", "-120 %,0 %,120 %", MSK_ALL )


#endif // SETUP_LIST_H
