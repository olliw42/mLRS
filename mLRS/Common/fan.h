//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Cooling Fan
//********************************************************
#ifndef FAN_H
#define FAN_H
#pragma once


#include <stdlib.h>
#include <ctype.h>
#include "hal/hal.h"


#ifndef USE_FAN

#ifdef FAN_ALWAYS_ON
  #warning FAN_ALWAYS_ON is defined, but the device has no fan!
#endif

class tFan
{
  public:
    void Init(void) {}

    void SetPower(int8_t power_dbm) {}
    void Tick_ms(void) {}
};

#else

#ifdef FAN_ALWAYS_ON
  #ifndef FAN_ALWAYS_ON_DBM
    #define FAN_ALWAYS_ON_DBM  POWER_MAX // POWER_MAX selects the highest fan setting the device offers
  #endif
#endif


class tFan
{
  public:
    void Init(void);

    void SetPower(int8_t power_dbm);
    void Tick_ms(void);

  private:
    bool initialized;
    int8_t power_dbm_curr;
};


void tFan::Init(void)
{
    fan_init();
    initialized = false;
    power_dbm_curr = POWER_MIN;
#if defined FAN_ALWAYS_ON && defined DEVICE_HAS_FAN_TEMPCONTROLLED_ONOFF
    fan_on(); // temperature controlled fans are only switched here, Tick_ms() is disabled
#endif
}


void tFan::SetPower(int8_t power_dbm)
{
#ifdef FAN_ALWAYS_ON
    power_dbm = FAN_ALWAYS_ON_DBM; // ignore the rf power, the fan runs always
#endif
#ifdef DEVICE_HAS_FAN_ONOFF
    if (power_dbm_curr != power_dbm || !initialized) {
        initialized = true;
        fan_set_power(power_dbm);
        power_dbm_curr = power_dbm;
    }
#endif
}


void tFan::Tick_ms(void)
{
#if defined DEVICE_HAS_FAN_TEMPCONTROLLED_ONOFF && !defined FAN_ALWAYS_ON
    int16_t temp_dC = fan_tempsensor_read_dC();

    if (temp_dC > 500) { // 50.0 C
        fan_on();
    } else
    if (temp_dC < 400) { // 40.0 C
        fan_off();
    }
#endif
}


#endif // DEVICE_HAS_FAN

#endif // FAN_H



