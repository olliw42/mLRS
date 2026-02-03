//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// RF Power dynamic handling
//********************************************************
#ifndef RF_POWER_H
#define RF_POWER_H
#pragma once


#include "hal/hal.h"


class tRfPower
{
  public:
    void Init(void);
    void Update(void);
    void Set(tRcData* const rc, uint8_t power_switch_channel, uint8_t power);
    void Set(uint8_t power);

  private:
    uint8_t rfpower_current_idx;
    uint8_t rfpower_new_idx;
};


void tRfPower::Init(void)
{
    rfpower_current_idx = UINT8_MAX;  // to initialize properly
    rfpower_new_idx = rfpower_current_idx; // to prevent update before first Set
}


void tRfPower::Update(void)
{
    if (rfpower_new_idx == rfpower_current_idx) return; // no change required

    rfpower_current_idx = rfpower_new_idx;

    if (rfpower_current_idx >= RFPOWER_LIST_NUM) { // should not happen, play it safe
        rfpower_current_idx = RFPOWER_LIST_NUM - 1;
        rfpower_new_idx = rfpower_current_idx;
    }

    Config.Sx.Power_dbm = rfpower_list[rfpower_current_idx].dbm;
    Config.Sx2.Power_dbm = Config.Sx.Power_dbm;

    sx.UpdateRfPower(&Config.Sx);
    sx2.UpdateRfPower(&Config.Sx2);
}


void tRfPower::Set(uint8_t power)
{
    rfpower_new_idx = power;
}


void tRfPower::Set(tRcData* const rc, uint8_t power_switch_channel, uint8_t power)
{
    if (power_switch_channel == POWER_SWITCH_CHANNEL_OFF) { // disabled
        rfpower_new_idx = power;
        return;
    }

    if (power_switch_channel < POWER_SWITCH_CHANNEL_CH5 || power_switch_channel > POWER_SWITCH_CHANNEL_CH16) {
        return; // argh
    }

    if (power_switch_channel + 3 >= RC_DATA_LEN) while(1){} // should not happen

    int16_t rc_val = rc->ch[power_switch_channel + 3]; // ch5 .. ch16 -> 3 .. 15

    // linear mapping: divide RC range evenly across available power levels
    // rc_val: 11 bits, 1 .. 1024 .. 2047 for +-120%
    // use safe range: ~200 (-100%) to ~1848 (+100%)
    // low stick = low power, high stick = high power
    // +1 in denominator: range 200..1848 inclusive has 1649 values, ensures max input maps to max index

    int16_t num_levels = power + 1;  // number of power levels available (0 to power)
    int16_t new_idx = ((rc_val - 200) * num_levels) / (1848 - 200 + 1); // map linearly

    if (new_idx < 0) new_idx = 0; // constrain to min
    if (new_idx > power) new_idx = power; // constrain by Setup Power setting

    rfpower_new_idx = new_idx;
}


#endif // RF_POWER_H



