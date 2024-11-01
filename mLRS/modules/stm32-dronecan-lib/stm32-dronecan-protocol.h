//*******************************************************
// STM32 DroneCAN Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// DroneCAN Driver Library for STM32 using HAL
// for use with libcanard
//*******************************************************
#ifndef STM32_DRONECAN_DRIVER_PROTOCOL_H
#define STM32_DRONECAN_DRIVER_PROTOCOL_H

#include "libcanard/canard.h"


//-- various

#define DC_UNIQUE_ID_LEN               16


//-- frame structure

// 3rd byte
#define DC_EMPTY_HEADER_MASK            0xE0000000
#define DC_PRIORITY_MASK                0x1F000000
// message frame
#define DC_MESSAGE_TYPE_MASK            0x00FFFF00
// service frame
#define DC_SERVICE_TYPE_MASK            0x00FF0000
#define DC_REQUEST_NOT_RESPONSE_MASK    0x00008000
#define DC_DESTINATION_ID_MASK          0x00007F00
// 0-th byte
#define DC_SERVICE_NOT_MESSAGE_MASK     0x00000080
#define DC_SOURCE_ID_MASK               0x0000007F


#define DC_PRIORITY_FROM_CAN_ID(x)              (uint8_t)(((x) & DC_PRIORITY_MASK) >> 24)
// message frame
#define DC_MESSAGE_TYPE_FROM_CAN_ID(x)          (uint16_t)(((x) & DC_MESSAGE_TYPE_MASK) >> 8)
// service frame
#define DC_SERVICE_TYPE_FROM_CAN_ID(x)          (uint8_t)(((x) & DC_SERVICE_TYPE_MASK) >> 16)
#define DC_REQUEST_NOT_RESPONSE_FROM_CAN_ID(x)  (uint8_t)(((x) & DC_REQUEST_NOT_RESPONSE_MASK) >> 15)
#define DC_DESTINATION_ID_FROM_CAN_ID(x)        (uint8_t)(((x) & DC_DESTINATION_ID_MASK) >> 8)
// 0-th byte
#define DC_SERVICE_NOT_MESSAGE_FROM_CAN_ID(x)   (uint8_t)(((x) & DC_SERVICE_NOT_MESSAGE_MASK) >> 7)
#define DC_SOURCE_ID_FROM_CAN_ID(x)             (uint8_t)(((x) & DC_SOURCE_ID_MASK) >> 0)


#define DC_PRIORITY_TO_CAN_ID(x)                (((uint32_t)(x) << 24) & DC_PRIORITY_MASK)
// message frame
#define DC_MESSAGE_TYPE_TO_CAN_ID(x)            (((uint32_t)(x) << 8) & DC_MESSAGE_TYPE_MASK)
// service frame
#define DC_SERVICE_TYPE_TO_CAN_ID(x)            (((uint32_t)(x) << 16) & DC_SERVICE_TYPE_MASK)
#define DC_REQUEST_NOT_RESPONSE_TO_CAN_ID(x)    (((uint32_t)(x) << 15) & DC_REQUEST_NOT_RESPONSE_MASK)
#define DC_DESTINATION_ID_TO_CAN_ID(x)          (((uint32_t)(x) << 8) & DC_DESTINATION_ID_MASK)
// 0-th byte
#define DC_SERVICE_NOT_MESSAGE_TO_CAN_ID(x)     (((uint32_t)(x) << 7) & DC_SERVICE_NOT_MESSAGE_MASK)
#define DC_SOURCE_ID_TO_CAN_ID(x)               (((uint32_t)(x) << 0) & DC_SOURCE_ID_MASK)


#endif // STM32_DRONECAN_DRIVER_PROTOCOL_H
