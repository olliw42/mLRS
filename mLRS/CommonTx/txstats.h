//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Statistics for Transmitter
//********************************************************
#ifndef TXSTATS_H
#define TXSTATS_H
#pragma once


//-------------------------------------------------------

class TxStatsBase
{
  public:
    void Init(uint8_t _period);

    void Update1Hz(void);
    void Next(void);

    void doFrameReceived(void);
    void doValidFrameReceived(void);

    uint8_t GetRawLQ(void);
    uint8_t GetNormalizedLQ(void);
    uint8_t GetLQ(void);

  private:
    LqCounterBase valid_lq;
    LqCounterBase received_lq;

    virtual bool is_connected(void);
};


void TxStatsBase::Init(uint8_t _period)
{
    stats.Init();

    valid_lq.Init(_period);
    received_lq.Init(_period);
}


void TxStatsBase::Update1Hz(void)
{
    stats.Update1Hz();
}


void TxStatsBase::Next(void)
{
    valid_lq.Next();
    received_lq.Next();
}


void TxStatsBase::doFrameReceived(void)
{
    received_lq.Set();
    stats.frames_received++;
}


void TxStatsBase::doValidFrameReceived(void)
{
    valid_lq.Set();
    stats.valid_frames_received++;
}


uint8_t TxStatsBase::GetRawLQ(void)
{
    return valid_lq.GetRaw();
}


uint8_t TxStatsBase::GetNormalizedLQ(void)
{
    return valid_lq.GetNormalized();
}


uint8_t TxStatsBase::GetLQ(void)
{
    if (!is_connected()) return 0;
    if (stats.LQ == 0) return 1;
    return stats.LQ;
}


#endif // TXSTATS_H
