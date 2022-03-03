//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// LQ
//*******************************************************
#ifndef LQ_COUNTER_H
#define LQ_COUNTER_H
#pragma once


//-------------------------------------------------------
// 1 Hz count statistics
//-------------------------------------------------------
// that's one way to get stats

class StatsCount {
  public:
    void Init(void)
    {
        count = count_last = counts_per_sec = 0;
        LQ = 0;
    }

    virtual void Update1Hz(void);

    uint32_t count;
    uint32_t count_last;
    uint32_t counts_per_sec;
    uint8_t LQ;
};


// targeted at LQ
class StatsLQ : public StatsCount
{
  public:
    void Update1Hz(void) override
    {
        // the period is Config.frame_rate_ms * Config.frame_rate_hz, and may not be exactly 1000

        counts_per_sec = count - count_last;
        LQ = (counts_per_sec * 100) / Config.frame_rate_hz;

        count_last = count;
    }

    void Inc(void)
    {
        count++;
    }

    uint8_t GetLQ(void)
    {
        return LQ;
    }
};


// targeted at bytes/sec
class StatsBytes : public StatsCount
{
  public:
    void Update1Hz(void) override
    {
        counts_per_sec = count - count_last; // are not exactly the bytes per sec, but the difference is 0.8% at most
        count_last = count;
    }

    void Add(uint16_t num)
    {
        count += num;
    }

    uint32_t GetBytesPerSec(void) // this make a presumption about its usage :)
    {
        return counts_per_sec;
    }
};


//-------------------------------------------------------
// moving window statistics
//-------------------------------------------------------
// that's another way to get stats

class LqCounterBase
{
  public:
    void Init(uint8_t _period)
    {
        SetPeriod(_period);
        Reset();
    }

    void SetPeriod(uint8_t new_period)
    {
        period = new_period;
        periodbit = (uint64_t)1 << (period - 1);
    }

    void Reset(void)
    {
        shiftreg = 0xFFFFFFFFFFFFFFFF; // this makes it that it starts with 100%
        curbit = periodbit; // so that calling Next() makes it to start at bit 1
        last_LQraw = period;
    }

    void Set(void)
    {
        shiftreg |= curbit;
    }

    void Next(void)
    {
        last_LQraw = calc_raw(); // buffer it, required since Next() and Set() do not coincide

        curbit <<= 1;
        if (curbit > periodbit) curbit = 1;
        shiftreg &=~ curbit;
    }

    uint8_t GetRaw(void)
    {
        return last_LQraw;
    }

    uint8_t GetNormalized(void)
    {
        return (last_LQraw * 100 + period/2) / period;
    }

  private:
    uint8_t period;
    uint64_t periodbit;
    uint64_t shiftreg;
    uint64_t curbit;
    uint8_t last_LQraw;

    uint8_t calc_raw(void)
    {
        uint16_t LQraw = 0;
        uint64_t bit = 1;
        while (bit <= periodbit) {
            if (shiftreg & bit) LQraw++;
            bit <<= 1;
        }
        return LQraw;
    }
};


#endif // LQ_COUNTER_H
