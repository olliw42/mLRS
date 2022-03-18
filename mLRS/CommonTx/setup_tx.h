//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Setup and configuration types
//*******************************************************
#ifndef SETUP_TX_H
#define SETUP_TX_H
#pragma once


#include <stdint.h>
#include "..\Common\setup.h"


#define SETUP_PARAMETER_LIST \
  X( Setup.BindPhrase[0],        STR6,  "Bind Phrase",  "BIND_PHRASE",  0,0,0, "", "" )\
  X( Setup.Mode,                 LIST,  "Mode",         "MODE",         0,0,0, "", "50 Hz,31 Hz,19 Hz" )\
  \
  X( Setup.Tx.Power,             LIST,  "Tx Power",     "TX_POWER",     0,0,0, "", "min,10 mW,50 mW,100 mW,200 mW,500 mW,1 W,max" )\
  X( Setup.Tx.Diversity,         LIST,  "Tx Diversity", "TX_DIVERSITY", 0,0,0, "", "default,antenna1,antenna2" )\
  X( Setup.Tx.SerialDestination, LIST,  "Tx Ser Dest",  "TX_SER_DEST",  0,0,0, "", "serial,mbridge" )\
  X( Setup.Tx.ChannelsSource,    LIST,  "Tx Ch Source", "TX_CH_SOURCE", 0,0,0, "", "none,mbridge,in,crsf" )\
  X( Setup.Tx.ChannelOrder,      LIST,  "Tx Ch Order",  "TX_CH_ORDER",  0,0,0, "", "AETR,TAER,ETAR" )\
  X( Setup.Tx.InMode,            LIST,  "Tx In Mode",   "TX_IN_MODE",   0,0,0, "", "sbus,sbus inv" )\
  X( Setup.Tx.SerialBaudrate,    LIST,  "Tx Ser Baudrate",  "TX_SER_BAUD",      0,0,0, "", "9600,19200,38400,57600,115200" )\
  X( Setup.Tx.SerialLinkMode,    LIST,  "Tx Ser Link Mode", "TX_SER_LNK_MODE",  0,0,0, "", "transp.,mavlink" )\
  X( Setup.Tx.SendRadioStatus,   LIST,  "Tx Snd RadioStat", "TX_SND_RADIOSTAT", 0,0,0, "", "off,on,on w txbuf" )\
  \
  X( Setup.Rx.Power,             LIST,  "Rx Power",     "RX_POWER",     0,0,0, "", "min,10 mW,50 mW,100 mW,200 mW,500 mW,1 W,max" )\
  X( Setup.Rx.Diversity,         LIST,  "Rx Diversity", "RX_DIVERSITY", 0,0,0, "", "default,antenna1,antenna2" )\
  X( Setup.Rx.ChannelOrder,      LIST,  "Rx Ch Order",  "RX_CH_ORDER",  0,0,0, "", "AETR,TAER,ETAR" )\
  X( Setup.Rx.OutMode,           LIST,  "Rx Out Mode",  "RX_OUT_MODE",  0,0,0, "", "sbus,crsf,sbus inv" )\
  X( Setup.Rx.OutRssiChannel,    UINT8, "Rx Out Rssi Ch",   "RX_OUT_RSSI_CH",   0,0,12, "", "" )\
  X( Setup.Rx.FailsafeMode,      LIST,  "Rx FailSafe Mode", "RX_FAILSAFE_MODE", 0,0,0, "", "no sig,low thr,by cnf,low thr cnt,ch1ch4 cnt" )\
  X( Setup.Rx.SerialBaudrate,    LIST,  "Rx Ser Baudrate",  "RX_SER_BAUD",      0,0,0, "", "9600,19200,38400,57600,115200" )\
  X( Setup.Rx.SerialLinkMode,    LIST,  "Rx Ser Link Mode", "RX_SER_LNK_MODE",  0,0,0, "", "transp.,mavlink" )\
  X( Setup.Rx.SendRadioStatus,   LIST,  "Rx Snd RadioStat", "RX_SND_RADIOSTAT", 0,0,0, "", "off,on,on w txbuf" )


// this must EXACTLY match MAV_PARAM_TYPE !! Otherwise Mavlink will be broken !!
typedef enum SETUP_PARAM_TYPE
{
    SETUP_PARAM_TYPE_UINT8  = 1, // 8-bit unsigned integer
    SETUP_PARAM_TYPE_INT8   = 2, // 8-bit signed integer
    SETUP_PARAM_TYPE_UINT16 = 3, // 16-bit unsigned integer
    SETUP_PARAM_TYPE_INT16  = 4, // 16-bit signed integer

    // our extensions, cannot be handled with mavlink!
    SETUP_PARAM_TYPE_LIST   = 254,
    SETUP_PARAM_TYPE_STR6   = 255,
} SETUP_PARAM_TYPE;


#define UINT8   uint8_t  // u8 fields are type case into uint16_t
#define INT8    uint8_t  // s8 fields are type case into uint16_t
#define UINT16  uint16_t // u16 fields are type case into uint16_t
#define INT16   uint16_t // s16 fields are type case into uint16_t
#define STR6    char
#define LIST    uint8_t  // list fields are type case into uint16_t


typedef union {
    uint8_t UINT8_value;
    int8_t INT8_value;
    uint16_t UINT16_value;
    int16_t INT16_value;
    char* STR6_value;
    uint8_t LIST_value;
} tSetupParameterValue;


typedef struct {
    void* ptr; // pointer to the field in the Setup structure
    uint8_t type;
    const char* name;
    const char* m_name;
    const char* unit;
    tSetupParameterValue dflt;
    tSetupParameterValue min;
    tSetupParameterValue max;
    const char* optstr;
} tSetupParameterItem;


const tSetupParameterItem SetupParameter[] = {
    #define X(p,t, n,mn, d,mi,ma,u, s) {.ptr=(t*)&(p), .type=SETUP_PARAM_TYPE_##t, .name=n, .m_name=mn, .unit=u, .dflt={.t##_value=d}, .min={.t##_value=mi}, .max={.t##_value=ma}, .optstr = s },
    SETUP_PARAMETER_LIST
    #undef X
};


#define SETUP_PARAMETER_NUM  sizeof(SetupParameter)/sizeof(tSetupParameterItem)



#endif // SETUP_TX_H
