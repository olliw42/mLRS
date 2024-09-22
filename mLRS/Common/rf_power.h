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

  private:
    uint8_t rfpower_current_idx;
};


void tRfPower::Init(void)
{
    rfpower_current_idx = UINT8_MAX;
}


void tRfPower::Update(void)
{
#ifdef DEVICE_IS_TRANSMITTER
    if (Setup.Tx[Config.ConfigId].Power == rfpower_current_idx) return; // Setup.Tx[].Power has not changed
    rfpower_current_idx = Setup.Tx[Config.ConfigId].Power;
#endif
#ifdef DEVICE_IS_RECEIVER
    if (Setup.Rx.Power == rfpower_current_idx) return; // Setup.Rx.Power has not changed
    rfpower_current_idx = Setup.Rx.Power;
#endif

    if (rfpower_current_idx >= RFPOWER_LIST_NUM) rfpower_current_idx = RFPOWER_LIST_NUM; // should not happen, play it safe

    Config.Sx.Power_dbm = rfpower_list[rfpower_current_idx].dbm;
    Config.Sx2.Power_dbm = Config.Sx.Power_dbm;

    sx.UpdateRfPower(&Config.Sx);
    sx2.UpdateRfPower(&Config.Sx2);

dbg.puts("\ntx pow = ");dbg.puts(u8toBCD_s(Setup.Rx.Power));
dbg.puts(" , ");dbg.puts(s8toBCD_s(Config.Sx.Power_dbm));
}


#endif // RF_POWER_H



