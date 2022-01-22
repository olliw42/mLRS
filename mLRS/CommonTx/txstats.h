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
    LqCounterBase LQma_received;
    LqCounterBase LQma_valid;

    virtual bool is_connected(void);
};


void TxStatsBase::Init(uint8_t _period)
{
    stats.Init();

    LQma_received.Init(_period);
    LQma_valid.Init(_period);
}


void TxStatsBase::Update1Hz(void)
{
    stats.Update1Hz();
}


void TxStatsBase::Next(void) // this is called when transmit starts, or shortly after
{
    LQma_valid.Next();
    LQma_received.Next();

    if (!is_connected()) { // start with 100% if not connected
      LQma_valid.Reset();
      LQma_received.Reset();
    }
}


void TxStatsBase::doFrameReceived(void)
{
    LQma_received.Set();
    stats.frames_received++;
}


void TxStatsBase::doValidFrameReceived(void)
{
    LQma_valid.Set();
    stats.valid_frames_received++;
}


uint8_t TxStatsBase::GetRawLQ(void)
{
    return LQma_valid.GetRaw();
}


uint8_t TxStatsBase::GetNormalizedLQ(void)
{
    return LQma_valid.GetNormalized();
}


uint8_t TxStatsBase::GetLQ(void)
{
    if (!is_connected()) return 0;
    uint8_t LQ = stats.GetLQ();
    if (LQ == 0) return 1;
    return LQ;
}


#endif // TXSTATS_H
