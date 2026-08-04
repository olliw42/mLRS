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
#include "setup_types.h"


#ifndef USE_FAN

#ifdef FAN_ALWAYS_ON
  #warning FAN_ALWAYS_ON is defined, but the device has no fan!
#endif

class tFan
{
  public:
    void Init(void) {}

    void SetMode(uint8_t fan_mode) {}
    void SetPower(int8_t power_dbm) {}
    void Tick_ms(void) {}
};

#else

#ifndef FAN_ALWAYS_ON_DBM
  #define FAN_ALWAYS_ON_DBM  POWER_MAX // POWER_MAX selects the highest fan setting the device offers
#endif


class tFan
{
  public:
    void Init(void);

    void SetMode(uint8_t fan_mode); // FAN_MODE_AUTO or FAN_MODE_ALWAYS_ON, see Setup.Tx[].FanMode
    void SetPower(int8_t power_dbm);
    void Tick_ms(void);

  private:
    bool always_on(void);

    bool initialized;
    int8_t power_dbm_curr;
    uint8_t mode;
};


bool tFan::always_on(void)
{
#ifdef FAN_ALWAYS_ON
    return true; // hard forced, the setup parameter is locked to always on
#else
    return (mode == FAN_MODE_ALWAYS_ON);
#endif
}


void tFan::Init(void)
{
    fan_init();
    initialized = false;
    power_dbm_curr = POWER_MIN;
    mode = FAN_MODE_AUTO; // the Tx overwrites this with the setup value before the first SetPower()
}


void tFan::SetMode(uint8_t fan_mode)
{
    mode = fan_mode;
}


void tFan::SetPower(int8_t power_dbm)
{
    if (always_on()) power_dbm = FAN_ALWAYS_ON_DBM; // ignore the rf power

#ifdef DEVICE_HAS_FAN_ONOFF
    if (power_dbm_curr != power_dbm || !initialized) { // a mode change also changes power_dbm, so is caught here
        initialized = true;
        fan_set_power(power_dbm);
        power_dbm_curr = power_dbm;
    }
#endif
}


void tFan::Tick_ms(void)
{
#ifdef DEVICE_HAS_FAN_TEMPCONTROLLED_ONOFF
    if (always_on()) { fan_on(); return; }

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



