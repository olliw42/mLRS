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


static inline bool connected(void);


//-------------------------------------------------------

class TxStatsBase
{
  public:
    void Init(uint8_t _period);

    void Update1Hz(void);
    void Next(void);

    void doFrameReceived(void);
    void doValidFrameReceived(void);

    uint8_t GetLQ(void);
    uint8_t GetLQ_serial_data(void);

    bool rx1_valid;
    bool rx2_valid;
    uint8_t fhss_curr_i;

  private:
    LqCounterBase LQma_received;
    LqCounterBase LQma_valid;
};


void TxStatsBase::Init(uint8_t _period)
{
    stats.Init();

    LQma_received.Init(_period);
    LQma_valid.Init(_period);

    rx1_valid = false;
    rx2_valid = false;
    fhss_curr_i = UINT8_MAX;
}


void TxStatsBase::Update1Hz(void)
{
    stats.Update1Hz();
}


void TxStatsBase::Next(void) // this is called when transmit starts, or shortly after
{
    LQma_valid.Next();
    LQma_received.Next();

    if (!connected()) { // start with 100% if not connected
      LQma_valid.Reset();
      LQma_received.Reset();
    }
}


void TxStatsBase::doFrameReceived(void)
{
    LQma_received.Set();
    stats.frames_received.Inc();
}


void TxStatsBase::doValidFrameReceived(void)
{
    LQma_valid.Set();
    stats.valid_frames_received.Inc();
}


uint8_t TxStatsBase::GetLQ(void)
{
    return GetLQ_serial_data(); // it is the same for Tx, we also could set it to invalid INT8_MAX
}


uint8_t TxStatsBase::GetLQ_serial_data(void)
{
    if (!connected()) return 0;
    uint8_t LQser = stats.serial_data_received.GetLQ(); // stats.valid_frames_received.GetLQ();
    if (LQser == 0) return 1;
    return LQser;
}


#endif // TXSTATS_H
