//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// LINK TYPES
//*******************************************************
#ifndef LINK_TYPES_H
#define LINK_TYPES_H
#pragma once


#include "common_conf.h"
#include "hal/device_conf.h"


//-------------------------------------------------------
// SX12xx
//-------------------------------------------------------

typedef enum {
    CONNECT_STATE_LISTEN = 0,
    CONNECT_STATE_SYNC,
    CONNECT_STATE_CONNECTED,
} CONNECT_STATE_ENUM;


#ifdef DEVICE_IS_TRANSMITTER
typedef enum {
    LINK_STATE_IDLE = 0,
    LINK_STATE_TRANSMIT,
    LINK_STATE_TRANSMIT_WAIT,
    LINK_STATE_RECEIVE,
    LINK_STATE_RECEIVE_WAIT,
    LINK_STATE_RECEIVE_DONE,
} LINK_STATE_ENUM;
#endif
#ifdef DEVICE_IS_RECEIVER
typedef enum {
    LINK_STATE_RECEIVE = 0,
    LINK_STATE_RECEIVE_WAIT,
    LINK_STATE_TRANSMIT,
    LINK_STATE_TRANSMIT_WAIT,
} LINK_STATE_ENUM;
#endif

typedef enum {
    RX_STATUS_NONE = 0, // no frame received
    RX_STATUS_INVALID, // frame received, but crc (and crc1) invalid
#ifdef DEVICE_IS_RECEIVER
    RX_STATUS_CRC1_VALID, // frame received, crc1 valid, but crc invalid
#endif
    RX_STATUS_VALID, // frame received and crc (and crc1) valid
} RX_STATUS_ENUM;


extern const char* connectstate_str[]; // for debug purposes
#ifdef DEVICE_IS_TRANSMITTER
extern const char* linkstate_str[]; // for debug purposes
extern const char* rxstatus_str[];
#endif
#ifdef DEVICE_IS_RECEIVER
extern const char* linkstate_str[]; // for debug purposes
extern const char* rxstatus_str[];
#endif


//-- Tx/Rx cmd frame handling

typedef enum {
    LINK_TASK_NONE = 0,

#ifdef DEVICE_IS_TRANSMITTER
    LINK_TASK_TX_GET_RX_SETUPDATA,
    LINK_TASK_TX_SET_RX_PARAMS,
    LINK_TASK_TX_STORE_RX_PARAMS,
    LINK_TASK_TX_GET_RX_SETUPDATA_WRELOAD,
#endif

#ifdef DEVICE_IS_RECEIVER
    LINK_TASK_RX_SEND_RX_SETUPDATA,
#endif
} LINK_TASK_ENUM;


typedef enum {
    TRANSMIT_FRAME_TYPE_NORMAL = 0,
    TRANSMIT_FRAME_TYPE_CMD,
} TRANSMIT_FRAME_TYPE_ENUM;


#endif // LINK_TYPES_H
