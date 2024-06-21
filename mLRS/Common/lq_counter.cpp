//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// LQ
//*******************************************************

#include "lq_counter.h"


//-------------------------------------------------------
// StatsCount
//-------------------------------------------------------

void tStatsCount::Init(uint16_t _frame_rate_hz)
{
    frame_rate_hz = _frame_rate_hz;

    count = count_last = counts_per_sec = 0;
    LQ = 0;
}


//-------------------------------------------------------
// StatsLQ
//-------------------------------------------------------
// targeted at LQ

void tStatsLQ::Update1Hz(void)
{
    // the period is Config.frame_rate_ms * Config.frame_rate_hz, and may not be exactly 1000

    counts_per_sec = count - count_last;
    LQ = (counts_per_sec * 100) / frame_rate_hz;

    count_last = count;
}


void tStatsLQ::Inc(void)
{
    count++;
}


uint8_t tStatsLQ::GetLQ(void)
{
    return LQ;
}


//-------------------------------------------------------
// StatsBytes
//-------------------------------------------------------
// targeted at bytes/sec

void tStatsBytes::Update1Hz(void)
{
    counts_per_sec = count - count_last; // are not exactly the bytes per sec, but the difference is 0.8% at most
    count_last = count;
}


void tStatsBytes::Add(uint16_t num)
{
    count += num;
}


uint32_t tStatsBytes::GetBytesPerSec(void) // this make a presumption about its usage :)
{
    return counts_per_sec;
}


//-------------------------------------------------------
// StatsMavlinkLQ
//-------------------------------------------------------
// targeted at MAVLink packet LQ

void tStatsMavlinkLQ::Update1Hz(void)
{
    if (count) {
        LQ = (counts_per_sec * 100) / count;
    } else {
        LQ = 0;
    }

    count = counts_per_sec = 0;
}


void tStatsMavlinkLQ::Cnt(bool valid)
{
    if (valid) counts_per_sec++;
    count++;
}


uint8_t tStatsMavlinkLQ::GetLQ(void)
{
    return LQ;
}


//-------------------------------------------------------
// Moving window statistics
// LqCounterBase
//-------------------------------------------------------
// that's another way to get stats

void tLqCounterBase::Init(uint8_t _period)
{
    SetPeriod(_period);
    Reset();
}


void tLqCounterBase::SetPeriod(uint8_t new_period)
{
    period = new_period;
    periodbit = (uint64_t)1 << (period - 1);
}


void tLqCounterBase::Reset(void)
{
    shiftreg = 0xFFFFFFFFFFFFFFFF; // this makes it that it starts with 100%
    curbit = periodbit; // so that calling Next() makes it to start at bit 1
    LQraw_last = period;
}


void tLqCounterBase::Set(void)
{
    shiftreg |= curbit;
}


void tLqCounterBase::Next(void)
{
    LQraw_last = calc_raw(); // buffer it, required since Next() and Set() do not coincide

    curbit <<= 1;
    if (curbit > periodbit) curbit = 1;
    shiftreg &=~ curbit;
}


uint8_t tLqCounterBase::GetRaw(void)
{
    return LQraw_last;
}


uint8_t tLqCounterBase::GetNormalized(void)
{
    return (LQraw_last * 100 + period/2) / period;
}


uint8_t tLqCounterBase::calc_raw(void)
{
    uint16_t LQraw = 0;
    uint64_t bit = 1;

    while (bit <= periodbit) {
        if (shiftreg & bit) LQraw++;
        bit <<= 1;
    }

    return LQraw;
}

