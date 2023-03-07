//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Some Filters
//*******************************************************

#include "filters.h"


//-- simple rate filter for mavlink interface

void LPFilterRate::Reset(void)
{
    xlast = 0;
    tlast_ms = 0;
    filt_internal = 0;
    state = 0;
}


void LPFilterRate::Update(int32_t tnow_ms, int32_t x, int32_t T_ms)
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

