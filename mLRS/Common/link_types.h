//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// LINK
//*******************************************************
#ifndef LINK_H
#define LINK_H
#pragma once


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
    RX_STATUS_INVALID,
#ifdef DEVICE_IS_RECEIVER
    RX_STATUS_CRC1_VALID,
#endif
    RX_STATUS_VALID,
} RX_STATUS_ENUM;


//-- Tx/Rx cmd frame handling

typedef enum {
    LINK_TASK_NONE = 0,
    LINK_TASK_TX_GET_RX_SETUPDATA,
    LINK_TASK_TX_SET_RX_PARAMS,
    LINK_TASK_TX_STORE_RX_PARAMS,
} LINK_TASK_ENUM;


typedef enum {
    TRANSMIT_FRAME_TYPE_NORMAL = 0,
    TRANSMIT_FRAME_TYPE_CMD,
} TRANSMIT_FRAME_TYPE_ENUM;


#endif // LINK_H
