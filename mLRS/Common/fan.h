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


#ifndef USE_FAN

class tFan
{
  public:
    void Init(void) {}
    void Do(void) {}

    void SetPower(int8_t power_dbm) {}
};

#else

class tFan
{
  public:
    void Init(void);
    void Do(void);

    void SetPower(int8_t power_dbm);

  private:
    bool initialized;
    int8_t power_dbm_curr;
};


void tFan::Init(void)
{
    fan_init();
    initialized = false;
    power_dbm_curr = POWER_MIN;
}


void tFan::SetPower(int8_t power_dbm)
{
#ifdef DEVICE_HAS_FAN_ONOFF
    if (power_dbm_curr != power_dbm || !initialized) {
        initialized = true;
        fan_set_power(power_dbm);
        power_dbm_curr = power_dbm;
    }
#endif
}


void tFan::Do(void)
{
}


//-------------------------------------------------------
// Low-level beep implementation
//-------------------------------------------------------



#endif // DEVICE_HAS_FAN

#endif // FAN_H



