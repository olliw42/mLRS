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


// simple rate filter for MAVLink interface
class tLPFilterRate
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


// simple LPF filter

class tLpFilter
{
  public:
    void Init(uint32_t _T_ms, int32_t _dt_ms, int32_t _yn_start = 0);
    void Clear(void);
    void Put(int32_t x);
    int32_t Get(void);

    float alpha;
    int32_t yn_start;
    float yn;
};


#endif // FILTERS_H
