//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Some Filters
//*******************************************************

#include "filters.h"


//-- simple rate filter for MAVLink interface

void tLPFilterRate::Reset(void)
{
    xlast = 0;
    tlast_ms = 0;
    filt_internal = 0;
    state = 0;
}


void tLPFilterRate::Update(int32_t tnow_ms, int32_t x, int32_t T_ms)
{
    Tfilt_ms = T_ms;
    if (state > 1) {
        int32_t xdiff = (x - xlast) * 1000;
        int32_t tdiff_ms = tnow_ms - tlast_ms;
        filt_internal += (xdiff - filt_internal * tdiff_ms) / (Tfilt_ms + tdiff_ms);
    } else
    if (state == 0) {
        filt_internal = 0;
        state++;
    } else
    if (state == 1) {
        int32_t xdiff = (x - xlast) * 1000;
        int32_t tdiff_ms = tnow_ms - tlast_ms;
        filt_internal = xdiff / tdiff_ms;
        state++;
    }
    xlast = x;
    tlast_ms = tnow_ms;
}


//-- simple LPF filter

void tLpFilter::Init(uint32_t _T_ms, int32_t _dt_ms, int32_t _yn_start)
{
    alpha = (float)_dt_ms / (float)(_T_ms + _dt_ms);
    yn_start = _yn_start;
    yn = yn_start;
}


void tLpFilter::Clear(void)
{
    yn = yn_start;
};


void tLpFilter::Put(int32_t x)
{
    yn += alpha * ((float)x - yn);
}


int32_t tLpFilter::Get(void)
{
    return (int32_t)(yn + 0.5f);
}

