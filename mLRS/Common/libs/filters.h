//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Some Filters
//*******************************************************
#ifndef FILTERS_H
#define FILTERS_H
#pragma once


#include <inttypes.h>


// simple rate filter for mavlink interface
class LPFilterRate
{
  public:
    void Reset(void);
    void Update(int32_t tnow_ms, int32_t x, int32_t T_ms);
    int32_t Get(void) { return filt_internal; }

    int32_t Tfilt_ms;
    int32_t xlast;
    int32_t tlast_ms;
    int32_t filt_internal;
    uint8_t state;
};


#endif // FILTERS_H
