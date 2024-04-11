//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Statistics for Receiver
//********************************************************
#ifndef RXSTATS_H
#define RXSTATS_H
#pragma once


static inline bool connected(void);


//-------------------------------------------------------
// we also handle the stats field with this class, this is somewhat dirty

class tRxStats
{
  public:
    void Init(uint8_t _period);

    void Update1Hz(void); // called at 1 Hz
    void Next(void); // called at each cycle
    void Clear(void); // called then not connected

    void doFrameReceived(void);
    void doValidCrc1FrameReceived(void);
    void doValidFrameReceived(void);

    uint8_t GetLQ_rc(void); // this is the "main" LQ, in case of Rx reflects the crc1-rcdata LQ
    uint8_t GetLQ_serial(void);

  private:
    LqCounterBase LQma_received;
    LqCounterBase LQma_valid_crc1;
    LqCounterBase LQma_valid;
};


void tRxStats::Init(uint8_t _period)
{
    stats.Init();

    LQma_valid_crc1.Init(_period);
    LQma_valid.Init(_period);
    LQma_received.Init(_period);
}


void tRxStats::Update1Hz(void)
{
    stats.Update1Hz();
}


void tRxStats::Next(void) // this is called when transmit starts, or shortly after
{
    LQma_valid_crc1.Next();
    LQma_valid.Next();
    LQma_received.Next();
}


void tRxStats::Clear(void)
{
    stats.Clear();

    LQma_valid_crc1.Reset(); // start with 100% if not connected
    LQma_valid.Reset();
    LQma_received.Reset();
}


void tRxStats::doFrameReceived(void)
{
    LQma_received.Set();
    stats.frames_received.Inc();
}


void tRxStats::doValidCrc1FrameReceived(void)
{
    LQma_valid_crc1.Set();
    stats.valid_crc1_received.Inc();
}


void tRxStats::doValidFrameReceived(void)
{
    LQma_valid.Set();
    stats.valid_frames_received.Inc();
}


uint8_t tRxStats::GetLQ_rc(void)
{
    if (!connected()) return 0;

    uint8_t LQ = stats.valid_crc1_received.GetLQ();
    if (LQ == 0) return 1;
    return LQ;
}


uint8_t tRxStats::GetLQ_serial(void)
{
    if (!connected()) return 0;

    uint8_t LQser = stats.serial_data_received.GetLQ(); // stats.valid_frames_received.GetLQ();
    if (LQser == 0) return 1;
    return LQser;
}


#endif // RXSTATS_H
