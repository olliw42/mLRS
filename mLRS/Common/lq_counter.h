//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// LQ
//*******************************************************
#ifndef LQ_H
#define LQ_H
#pragma once


class LqCounterBase
{
  public:
    void Init(uint8_t _period)
    {
        SetPeriod(_period);
        Reset();
    }

    void SetPeriod(uint8_t _period)
    {
        period = _period;
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
        last_LQraw = _calc_raw(); // buffer it, required since Next() and Set() do not coincide

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

    uint8_t _calc_raw(void)
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


#endif // LQ_H
