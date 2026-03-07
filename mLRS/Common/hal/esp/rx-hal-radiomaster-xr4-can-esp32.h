//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, ELRS RadioMaster XR4 RX with DroneCAN
//-------------------------------------------------------

#define DEVICE_HAS_DRONECAN

#include "rx-hal-generic-lr1121-td-esp32.h"


//-- CAN (TWAI) pins

#define CAN_TX_IO   IO_P18   // TX2 pad
#define CAN_RX_IO   IO_P5    // RX2 pad


//-- LR11xx DIO switch control

#define SX_USE_RFSW_CTRL {15, 0, 12, 8, 8, 6, 0, 5}
