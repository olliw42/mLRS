//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// PLL
//********************************************************

#include "pll.h"


PllBase::PllBase() // constructor
{
    ResetAll();
}


void PllBase::Init(uint16_t expected_period_us)
{
    expected_period = expected_period_us/10;

    ResetAll();

    tim_set_period_10us(expected_period);
}


void PllBase::ResetAll(void)
{
    tick_obtained = false;
    tick_tlast = 0;
    rx_done_obtained = false;
    rx_done_tlast = 0;

    phase_abs_filt_n = UINT32_MAX; // indicator to initialize

    rx_period_state = RX_PERIOD_INVALID;

    locked = false;
    obtained = 0;
    missed = UINT8_MAX;

  updated = false;
  idx = 0;
}


// when the PLL was locked before, it doesn't need to and shouldn't start from zero
// e.g., we can keep our info on rx_period
void PllBase::Reset(void)
{
    tick_obtained = false;
    tick_tlast = 0;
    rx_done_obtained = false;
    rx_done_tlast = 0;

    phase_abs_filt_n = UINT32_MAX; // indicator to initialize

    locked = false;
    obtained = 0;
    missed = UINT8_MAX;
}


// called by clock at "rising" edge
void PllBase::tick(void)
{
    tick_obtained = true;
    tick_tlast = tim_10us();

    ticked = true;
}


// called by clock at "falling" edge
// we synchronize on RxDone
void PllBase::update(void)
{
    int16_t phase = INT16_MAX; //0;
    uint16_t period = UINT16_MAX;

    if (tick_obtained && rx_done_obtained && (rx_period_state >= RX_PERIOD_LOCKED)) {
        if (obtained < UINT8_MAX) obtained++;
        missed = 0;

        phase = tick_tlast - rx_done_tlast;

        // close the loop
        // /4 works well /8 takes significantly longer to lock /2 is great
        // these are empirically determined, may have to be changed when rate changes ?
        int16_t error = phase/2;

        if (locked) {
          if (error > PLL_PHASE_ERR_LIMIT_10US) error = PLL_PHASE_ERR_LIMIT_10US;
          if (error < -PLL_PHASE_ERR_LIMIT_10US) error = -PLL_PHASE_ERR_LIMIT_10US;
        }

        period = expected_period - error;

        tim_set_period_10us(period);

        // get a filtered absolute phase
        uint32_t phase_abs = (phase >= 0) ? phase : -phase;
        if (phase_abs_filt_n == UINT32_MAX) {
            phase_abs_filt_n = phase_abs * phase_abs_filt_fact;
        }
        phase_abs_filt_n += (phase_abs - phase_abs_filt_n / phase_abs_filt_fact);

        // decide based on it if pll has locked
        if (!locked) {
            if (phase_abs_filt_n/phase_abs_filt_fact < 5) locked = true;
        }
    } else
    if (tick_obtained && (rx_period_state >= RX_PERIOD_LOCKED)) { // well, tick_obtained = false should not happen
        obtained = 0;
        if (missed < UINT8_MAX) missed++;
    }

    tick_obtained = false;
    rx_done_obtained = false;

  snap_idx = idx;
  snap_tlast_rx_done = rx_done_tlast;
  snap_tlast_tick = tick_tlast;
  snap_period = period;
  snap_phase = phase;

  updated = true;
  idx++;
}


// called when RxDone
// is split in two to keep ISR short

void PllBase::rx_done1(void)
{
    rx_done_tnow = tim_10us();
}


void PllBase::rx_done2(void)
{
uint16_t tnow = rx_done_tnow;

    switch (rx_period_state) {
    case RX_PERIOD_INVALID:
        // rx_done_tlast is initialized below
        rx_period_state = RX_PERIOD_INITIALIZED;
        break;

    case RX_PERIOD_INITIALIZED: {
        rx_period_state = RX_PERIOD_INITIALIZED;

        // now we can get an estimate for the period
        uint16_t est_period = tnow - rx_done_tlast;

        // is period in expected/acceptable range?
        if (est_period > (expected_period - PLL_LOCK_RANGE_10US) && est_period < (expected_period + PLL_LOCK_RANGE_10US)) {
            rx_period_state = RX_PERIOD_LOCKED;
        }
        }break;

    case RX_PERIOD_LOCKED:
        break;
    }

    rx_done_obtained = true;
    rx_done_tlast = tnow;
}




