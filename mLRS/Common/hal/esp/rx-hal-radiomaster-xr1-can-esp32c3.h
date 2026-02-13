//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32C3, ELRS RadioMaster XR1 RX with DroneCAN
//-------------------------------------------------------

#define DEVICE_HAS_DRONECAN

#include "rx-hal-generic-c3-lr1121-esp32c3.h"


//-- CAN (TWAI) pins

#define CAN_TX_IO   IO_P19
#define CAN_RX_IO   IO_P18


//-- LR11xx DIO switch control

#define SX_USE_RFSW_CTRL {15, 0, 12, 8, 8, 6, 0, 5}
