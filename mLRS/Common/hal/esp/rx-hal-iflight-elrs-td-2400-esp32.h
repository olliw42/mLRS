//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, ELRS iFlight 2400 True Diversity PA RX
//-------------------------------------------------------
// antenna1 = left ufl
// antenna2 = right ufl

#include "rx-hal-generic-2400-td-pa-esp32.h"


//-- POWER

#define POWER_GAIN_DBM            25 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      (SX1280_POWER_0_DBM-1) // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 }, // PA=27 ~8.2 dBm, 7 mW, PA=25 ~10.5 dBm, 11 mW
    { .dbm = POWER_17_DBM, .mW = 50 }, //                        PA=25 ~17.0 dBm, 50 mW
    { .dbm = POWER_20_DBM, .mW = 100 }, // PA=27 ~18.4 dBm, 70 mW, PA=25 ~20.2 dBm, 105 mW
    { .dbm = POWER_24_DBM, .mW = 250 }, // PA=27 ~22.7 dBm, 190 mW, PA=25 ~24.6 dBm, 290 mW
};
