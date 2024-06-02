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


extern tStats stats;
static inline bool connected(void);


//-------------------------------------------------------
// we also handle the stats field with this class, this is somewhat dirty

class tTxStats
{
  public:
    void Init(uint8_t _maverage_period, uint16_t _frame_rate_hz);

    void Update1Hz(void);   // called at 1 Hz
    void Next(void);        // called at each cycle, is called when transmit starts, or shortly after
    void Clear(void);       // called then not connected, is called when transmit starts, or shortly after

    void doFrameReceived(void);
    void doValidFrameReceived(void);

    uint8_t GetLQ_serial(void);

    // extra stats available with mBridge
    bool rx1_valid;
    bool rx2_valid;
    uint8_t fhss_curr_i;

  private:
//    tLqCounterBase LQma_received;
//    tLqCounterBase LQma_valid;
};


void tTxStats::Init(uint8_t _maverage_period, uint16_t _frame_rate_hz)
{
    stats.Init(_frame_rate_hz);

    rx1_valid = false;
    rx2_valid = false;
    fhss_curr_i = UINT8_MAX;

//    LQma_received.Init(_maverage_period);
//    LQma_valid.Init(_maverage_period);
}


void tTxStats::Update1Hz(void)
{
    stats.Update1Hz();
}


void tTxStats::Next(void)
{
//    LQma_valid.Next();
//    LQma_received.Next();
}


void tTxStats::Clear(void)
{
    stats.Clear();

//    LQma_valid.Reset(); // start with 100% if not connected
//    LQma_received.Reset();
}


void tTxStats::doFrameReceived(void)
{
    stats.frames_received.Inc();

//    LQma_received.Set();
}


void tTxStats::doValidFrameReceived(void)
{
    stats.valid_frames_received.Inc();

//    LQma_valid.Set();
}


uint8_t tTxStats::GetLQ_serial(void)
{
    if (!connected()) return 0;

    uint8_t LQser = stats.serial_data_received.GetLQ(); // stats.valid_frames_received.GetLQ();
    if (LQser == 0) return 1;
    return LQser;
}


#endif // TXSTATS_H
