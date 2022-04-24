//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// common
//*******************************************************
#ifndef COMMON_H
#define COMMON_H
#pragma once


#include "common_conf.h"
#include "sx-drivers\sx12xx_driver.h"
#include "lq_counter.h"
#include "fhss.h"
#include "frames.h"
#include "frame_types.h"
#include "link_types.h"
#include "common_stats.h"


//-------------------------------------------------------
// SysTask
//-------------------------------------------------------

volatile uint8_t doSysTask = 0;


void HAL_IncTick(void)
{
    uwTick += uwTickFreq;
    doSysTask = 1;
}


uint32_t millis32(void)
{
    return uwTick;
}


//-------------------------------------------------------
// Serial Classes
//-------------------------------------------------------

#ifdef USE_COM_ON_SERIAL
  // TODO: when we swap ser/com, we may want to flush, we need to change baudrate
  #define SERORCOMINIT  ser_or_com_init(); if (!ser_or_com_serial()) uartb_setbaudrate(TX_COM_BAUDRATE);
  #define IFNSER(x)  if (!ser_or_com_serial()) return x;
  #define IFNCOM(x)  if (ser_or_com_serial()) return x;
#else
  #define SERORCOMINIT
  #define IFNSER(x)
  #define IFNCOM(x)
#endif


// is always uartb
class tSerialPort : public tSerialBase
{
#ifdef USE_SERIAL
  public:
    void Init(void) override { uartb_init(); SERORCOMINIT; }
    void SetBaudRate(uint32_t baud) override { IFNSER(); uartb_setprotocol(baud, XUART_PARITY_NO, UART_STOPBIT_1); }
    void putc(char c) override { IFNSER(); uartb_putc(c); }
    bool available(void) override { IFNSER(0); return uartb_rx_available(); }
    char getc(void) override { IFNSER(0); return uartb_getc(); }
    void flush(void) override { IFNSER(); uartb_rx_flush(); uartb_tx_flush(); }
    uint16_t bytes_available(void) override { IFNSER(0); return uartb_rx_bytesavailable(); }
//XX    const uint16_t rx_buf_size(void) override { return UARTB_RXBUFSIZE; }
//XX    bool tx_is_empty(void) override { IFNSER(0); return uartb_tx_isempty(); }
#endif
};


// is always uartc
class tDebugPort : public tSerialBase
{
#ifdef USE_DEBUG
  public:
    void Init(void) { uartc_init(); }
    void putc(char c) override { uartc_putc(c); }
#endif
};


// is uartc or uartb
class tComPort : public tSerialBase
{
#ifdef USE_COM_ON_SERIAL
  public:
    // we do not initialize it as it is initialized by serial
    void putc(char c) override { IFNCOM(); uartb_putc(c); }
    bool available(void) override { IFNCOM(0); return uartb_rx_available(); }
    char getc(void) override { IFNCOM(0); return uartb_getc(); }
    void flush(void) override { IFNCOM(); uartb_rx_flush(); uartb_tx_flush(); }
#endif
#ifdef USE_COM_ON_C
  public:
    void Init(void) override { uartc_init(); }
    void putc(char c) override { uartc_putc(c); }
    bool available(void) override { return uartc_rx_available(); }
    char getc(void) override { return uartc_getc(); }
#endif
};


//-------------------------------------------------------
// Sx/Sx2 convenience wrapper
//-------------------------------------------------------
// only declarations, implementations follow below

void sxReadFrame(uint8_t antenna, void* data, void* data2, uint8_t len);
void sxSendFrame(uint8_t antenna, void* data, void* data2, uint8_t len, uint16_t tmo_ms);
void sxGetPacketStatus(uint8_t antenna, Stats* stats);


//-------------------------------------------------------
// Common Variables
//-------------------------------------------------------

tSerialPort serial;
tDebugPort dbg;

tRcData rcData;

tTxFrame txFrame;
tRxFrame rxFrame;

tTxFrame txFrame2;
tRxFrame rxFrame2;

SX_DRIVER sx;
SX2_DRIVER sx2;

Stats stats;

FhssBase fhss;

#include "bind.h" // bind needs sx,sx2 variables

BindBase bind;


//-------------------------------------------------------
// Sx/Sx2 convenience wrapper
//-------------------------------------------------------

void sxReadFrame(uint8_t antenna, void* data, void* data2, uint8_t len)
{
    if (antenna == ANTENNA_1) {
        sx.ReadFrame((uint8_t*)data, len);
    } else {
        sx2.ReadFrame((uint8_t*)data2, len);
    }
}


void sxSendFrame(uint8_t antenna, void* data, void* data2, uint8_t len, uint16_t tmo_ms)
{
    if (antenna == ANTENNA_1) {
        sx.SendFrame((uint8_t*)data, len, tmo_ms);
    } else {
        sx2.SendFrame((uint8_t*)data2, len, tmo_ms);
    }
}


void sxGetPacketStatus(uint8_t antenna, Stats* stats)
{
    if (antenna == ANTENNA_1) {
        sx.GetPacketStatus(&(stats->last_rx_rssi1), &(stats->last_rx_snr1)); //&stats.last_rx_rssi1, &stats.last_rx_snr1);
    } else {
        sx2.GetPacketStatus(&(stats->last_rx_rssi2), &(stats->last_rx_snr2));
    }
}


//-------------------------------------------------------
//-- check some sizes
//-------------------------------------------------------

STATIC_ASSERT(sizeof(tFrameStatus) == FRAME_TX_RX_HEADER_LEN - 2, "tFrameStatus len missmatch")
STATIC_ASSERT(sizeof(tTxFrame) == FRAME_TX_RX_LEN, "tTxFrame len missmatch")
STATIC_ASSERT(sizeof(tRxFrame) == FRAME_TX_RX_LEN, "tRxFrame len missmatch")

STATIC_ASSERT(sizeof(tTxBindFrame) == FRAME_TX_RX_LEN, "tTxBindFrame len missmatch")
STATIC_ASSERT(sizeof(tRxBindFrame) == FRAME_TX_RX_LEN, "tRxBindFrame len missmatch")

STATIC_ASSERT(sizeof(tTxCmdFrameRxParams) == FRAME_TX_PAYLOAD_LEN, "tTxCmdFrameRxParams len missmatch")
STATIC_ASSERT(sizeof(tRxCmdFrameRxSetupData) == FRAME_RX_PAYLOAD_LEN, "tRxCmdFrameRxSetupData len missmatch")


#endif // COMMON_H
