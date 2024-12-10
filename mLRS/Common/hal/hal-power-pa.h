//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal generic power settings for common PAs
// and other generic cases
//*******************************************************

//-------------------------------------------------------
//-- 2.4 GHz
//-------------------------------------------------------

//-- no PA/LNA
#ifdef POWER_PA_NONE_SX128X
#define POWER_PA_DEFINED

#define POWER_GAIN_DBM            0 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_12p5_DBM // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           2 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_12p5_DBM, .mW = 18 },
};

#endif


//-- SKY85321-11
// PA max ?, PAin max 5 dBm, PA gain 28 dB, LNA gain 13.5 dB
#if defined POWER_PA_SKY85321_11 || defined POWER_PA_E28_2G4M27SX // E28-2G4M27S/SX
#define POWER_PA_DEFINED

#define POWER_GAIN_DBM            27 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_0_DBM // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
};

#endif
#if defined POWER2_PA_SKY85321_11 || defined POWER2_PA_E28_2G4M27SX
#define POWER_PA_DEFINED

#define POWER2_GAIN_DBM           27 // gain of a PA stage if present
#define POWER2_SX1280_MAX_DBM     SX1280_POWER_0_DBM // maximum allowed sx power
#define POWER2_USE_DEFAULT_RFPOWER_CALC

#endif


//-- SKY65383-11
// PA max 30 dBm, PAin max 6 dBm, PA gain 34 dB, LNA gain 12 dB
#if defined POWER_PA_SKY65383_11 || defined POWER_PA_MATEK_MR24_30 // Matek mR24-30
#define POWER_PA_DEFINED

#include "../setup_types.h"

// SX1280 power setting can vary from 0 .. 31 which corresponds to -18 dBm .. 13 dBm
void sx1280_rfpower_calc(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm)
{
    if (power_dbm >= POWER_30_DBM) {
        *sx_power = 19;
        *actual_power_dbm = 30;
    } else
    if (power_dbm >= POWER_27_DBM) {
        *sx_power = 12;
        *actual_power_dbm = 27;
    } else
    if (power_dbm >= POWER_24_DBM) {
        *sx_power = 8;
        *actual_power_dbm = 24;
    } else
    if (power_dbm >= POWER_20_DBM) {
        *sx_power = 4;
        *actual_power_dbm = 20;
    } else
	if (power_dbm >= POWER_17_DBM) {
		*sx_power = 1;
		*actual_power_dbm = 17;
	} else {
        *sx_power = 0;
        *actual_power_dbm = 16;
    }
}

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};

#endif
#if defined POWER2_PA_SKY65383_11 || defined POWER2_PA_MATEK_MR24_30 || defined POWER2_PA_MATEK_MTX_DB30
#define POWER_PA_DEFINED

#define POWER2_GAIN_DBM           31 // gain of a PA stage if present
#define POWER2_SX1280_MAX_DBM     SX1280_POWER_0_DBM // maximum allowed sx power
#define POWER2_USE_DEFAULT_RFPOWER_CALC

#error mR24-30 dual band, power2 needs to be worked out!

#endif



//-------------------------------------------------------
//-- 868/915 MHz
//-------------------------------------------------------

//-- no PA/LNA
#ifdef POWER_PA_NONE_SX127X
#define POWER_PA_DEFINED

#define POWER_GAIN_DBM            0 // gain of a PA stage if present
#define POWER_SX1276_MAX_DBM      SX1276_OUTPUT_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_17_DBM, .mW = 50 },
};

#endif


//-- no PA/LNA
#ifdef POWER_PA_NONE_SX126X
#define POWER_PA_DEFINED

#define POWER_GAIN_DBM            0 // gain of a PA stage if present
#define POWER_SX126X_MAX_DBM      SX126X_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           2 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_22_DBM, .mW = 158 },
};

#endif


//-- SE2435L
// PA max 30 dBm, PAin max 10 dBm, PA gain 26 dB, LNA gain 16 dB
#if defined POWER_PA_SE2435L || defined POWER_PA_MATEK_MR900_30 || defined POWER_PA_MATEK_MTX_DB30
#define POWER_PA_DEFINED

#define SX_USE_PA_CONFIG_10_DBM

// SX126X power setting can vary from -9 .. 22 which corresponds to -9 dBm .. 22 dBm
#include "../setup_types.h"

void sx126x_rfpower_calc(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    if (power_dbm >= POWER_30_DBM) {
        *sx_power = (frequency_band == SX_FHSS_CONFIG_FREQUENCY_BAND_868_MHZ) ? 10 : 10;
        *actual_power_dbm = 30;
    } else
    if (power_dbm >= POWER_27_DBM) {
        *sx_power = (frequency_band == SX_FHSS_CONFIG_FREQUENCY_BAND_868_MHZ) ? 2 : 4;
        *actual_power_dbm = 27;
    } else
    if (power_dbm >= POWER_24_DBM) {
        *sx_power = (frequency_band == SX_FHSS_CONFIG_FREQUENCY_BAND_868_MHZ) ? -2 : 0;
        *actual_power_dbm = 24;
    } else
    if (power_dbm >= POWER_20_DBM) {
        *sx_power = (frequency_band == SX_FHSS_CONFIG_FREQUENCY_BAND_868_MHZ) ? -6 : -4;
        *actual_power_dbm = 20;
    } else
	if (power_dbm >= POWER_17_DBM) {
		*sx_power = (frequency_band == SX_FHSS_CONFIG_FREQUENCY_BAND_868_MHZ) ? -8 : -7;
		*actual_power_dbm = 17;
	} else {
        *sx_power = -9;
        *actual_power_dbm = (frequency_band == SX_FHSS_CONFIG_FREQUENCY_BAND_868_MHZ) ? 16 : 14;
    }
}

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};

#endif



//-------------------------------------------------------
//-- footer
//-------------------------------------------------------

#ifndef POWER_PA_DEFINED
#error hal-power-pa.h included without proper define !
#endif


/*
2.4 GHz Devices
---------------

* SKY85321-11
E28-2G4M27S/SX

* SKY66312-11
FRM303

* SKY68383-11
Matek mR24-30

* AT2401C
ELRS pa-esp8285, ELRS td-pa-esp32, RM ELRS rp4td

* SE2431L
ELRS d-pa-esp8285


900 MHz Devices
---------------

* SE2435L
Matek mR900-30

* SKY66319-11 
ELRS td-pa-esp32

*/
