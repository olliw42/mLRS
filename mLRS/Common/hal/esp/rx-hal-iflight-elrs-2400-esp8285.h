//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP8285, ELRS iFlight 2400 PA RX
//-------------------------------------------------------

#include "rx-hal-generic-2400-pa-esp8285.h"


//-- POWER

#define POWER_GAIN_DBM            27 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_0_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },

};
