//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// PLL
//********************************************************
#ifndef PLL_H
#define PLL_H
#pragma once


#include <inttypes.h>

// all times are in 10us
// I didn't really got the PLL stable with only detecting the phases
// the trick was to get the rx period, and do the shifts with respect to it
// this really works extremely well and made the PLL super quick and stable
// I also have tested it with 1us time base, and it also works fine
// however, with 1us I didn't get the PLL to converge as fast and stable as with 10us
// I decided that the expected period is quite accurate enough, so no need to also lock period

#define PLL_LOCK_RANGE_10US         10
#define PLL_PHASE_ERR_LIMIT_10US    100
#define PLL_PHASE_ABS_FILT_FACT     4


class PllBase
{
  public:
    PllBase(); // constructor

    typedef enum {
      RX_PERIOD_INVALID = 0,
      RX_PERIOD_INITIALIZED,
      RX_PERIOD_LOCKED,
    } RX_PERIOD_ENUM;

    void Init(uint16_t expected_period_us);
    void ResetAll(void); // this resets the PLL completely
    void Reset(void); // this should be called after a disconnect
    bool IsLocked(void) { return locked; }

    bool Ticked(void) { bool t = ticked; return t; }

    //-- interface

    virtual uint16_t tim_10us(void) = 0; // do not name it micros(), and give it an unusual name
    virtual void tim_set_period_10us(uint16_t period_10us) = 0;

    // called by clock on "rising" edge
    void tick(void);

    // called by clock on "falling" edge
    // we synchronize on RxDone
    // it must be called in the middle of the cycle, i.e., the clock must have 50:50 duty cycle
    void update(void);

    // called when RxDone
    // is split in two since the SX seems to not like it, if lots is done before the rx packet is read
    void rx_done1(void);
    void rx_done2(void);

//  private:
    uint16_t expected_period;

    bool tick_obtained;
    uint16_t tick_tlast;

    bool rx_done_obtained;
    uint16_t rx_done_tlast;
    uint16_t rx_done_tnow;

    uint32_t phase_abs_filt_n;
    const uint16_t phase_abs_filt_fact = PLL_PHASE_ABS_FILT_FACT;

    uint8_t rx_period_state;

    uint16_t obtained;
    uint16_t missed;
    bool locked;
    bool ticked = false;

    //-- for testing

    bool is_updated(void)
    {
      if (!updated) return false;
      updated = false;
      return true;
    }

    void rx_start(void) { tlast_rx_start = tim_10us(); }
    void tx_start(void) { tlast_tx_start = tim_10us(); }
    void tx_done(void) { tlast_tx_done = tim_10us(); }

    uint16_t snap_tlast_tick;
    uint16_t snap_rx_period;
    uint16_t snap_idx;
    uint16_t snap_period;
    int16_t snap_phase;
    uint16_t snap_tlast_rx_done;

    bool updated;
    uint16_t idx;

    uint16_t tlast_rx_start;
    uint16_t tlast_tx_start;
    uint16_t tlast_tx_done;
};


#endif // PLL_H
