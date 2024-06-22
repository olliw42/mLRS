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

#include <stdint.h>


//-------------------------------------------------------
// 1 Hz count statistics
//-------------------------------------------------------
// that's one way to get stats
// is based on 1 Hz time slots

class tStatsCount {
  public:
    void Init(uint16_t _frame_rate_hz);
    virtual void Update1Hz(void) = 0; // called at 1 Hz

  protected:
    uint32_t count;
    uint32_t count_last;
    uint32_t counts_per_sec;
    uint8_t LQ;

    uint16_t frame_rate_hz;
};


// targeted at LQ
class tStatsLQ : public tStatsCount
{
  public:
    void Update1Hz(void) override;
    void Inc(void); // called at each cycle
    uint8_t GetLQ(void);
};


// targeted at bytes/sec
class tStatsBytes : public tStatsCount
{
  public:
    void Update1Hz(void) override;
    void Add(uint16_t num); // called at each cycle
    uint32_t GetBytesPerSec(void);
};


// targeted at MAVLink packet LQ
class tStatsMavlinkLQ : public tStatsCount
{
  public:
    void Update1Hz(void) override;
    void Cnt(bool valid);
    uint8_t GetLQ(void);
};


//-------------------------------------------------------
// Moving window statistics
//-------------------------------------------------------
// that's another way to get stats

class tLqCounterBase
{
  public:
    void Init(uint8_t _period);
    void SetPeriod(uint8_t new_period);
    void Reset(void);
    void Set(void);
    void Next(void);
    uint8_t GetRaw(void);
    uint8_t GetNormalized(void);

  private:
    uint8_t period;
    uint64_t periodbit;
    uint64_t shiftreg;
    uint64_t curbit;
    uint8_t LQraw_last;

    uint8_t calc_raw(void);
};


#endif // LQ_COUNTER_H
