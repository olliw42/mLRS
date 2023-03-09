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
#include "sx-drivers/sx12xx_driver.h"
#include "lq_counter.h"
#include "fhss.h"
#include "frames.h"
#include "frame_types.h"
#include "link_types.h"
#include "common_stats.h"
#include "bind.h"
#include "fail.h"
#include "buzzer.h"
#include "fan.h"


//-------------------------------------------------------
// SysTask
//-------------------------------------------------------

volatile uint32_t doSysTask = 0;


void HAL_IncTick(void)
{
    uwTick += uwTickFreq;
    doSysTask++;
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
  #define SERORCOM_INIT  ser_or_com_init(); if (!ser_or_com_serial()) uartb_setbaudrate(TX_COM_BAUDRATE);
  #define IFNSER(x)  if (!ser_or_com_serial()) return x;
  #define IFNCOM(x)  if (ser_or_com_serial()) return x;
#else
  #define SERORCOM_INIT
  #define IFNSER(x)
  #define IFNCOM(x)
#endif


// is always uartb
class tSerialPort : public tSerialBase
{
#ifdef USE_SERIAL
  public:
    void Init(void) override { uartb_init(); SERORCOM_INIT; }
    void SetBaudRate(uint32_t baud) override { IFNSER(); uartb_setprotocol(baud, XUART_PARITY_NO, UART_STOPBIT_1); }
    void putc(char c) override { IFNSER(); uartb_putc(c); }
    bool available(void) override { IFNSER(0); return uartb_rx_available(); }
    char getc(void) override { IFNSER(0); return uartb_getc(); }
    void flush(void) override { IFNSER(); uartb_rx_flush(); uartb_tx_flush(); }
    uint16_t bytes_available(void) override { IFNSER(0); return uartb_rx_bytesavailable(); }
#endif
};


// is uartc on rx / uartf on tx (or swuart)
class tDebugPort : public tSerialBase
{
#ifdef USE_DEBUG
  public:
#ifdef DEVICE_HAS_DEBUG_SWUART
    void Init(void) { swuart_init(); }
    void putc(char c) override { swuart_putc(c); }
#else
#ifdef DEVICE_IS_RECEIVER
    void Init(void) { uartc_init(); }
    void putc(char c) override { uartc_putc(c); }
#endif
#ifdef DEVICE_IS_TRANSMITTER
    void Init(void) { uartf_init(); }
    void putc(char c) override { uartf_putc(c); }
#endif
#endif
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
#ifdef USE_COM
  public:
    void Init(void) override { uartc_init(); }
    void putc(char c) override { uartc_putc(c); }
    bool available(void) override { return uartc_rx_available(); }
    char getc(void) override { return uartc_getc(); }
#endif
};


// is always uartd
class tSerial2Port : public tSerialBase
{
#ifdef USE_SERIAL2
  public:
    void Init(void) override { uartd_init(); }
    void SetBaudRate(uint32_t baud) override { uartd_setprotocol(baud, XUART_PARITY_NO, UART_STOPBIT_1); }
    void putc(char c) override { uartd_putc(c); }
    bool available(void) override { return uartd_rx_available(); }
    char getc(void) override { return uartd_getc(); }
    void flush(void) override { uartd_rx_flush(); uartd_tx_flush(); }
    uint16_t bytes_available(void) override { return uartd_rx_bytesavailable(); }
#endif
};


//-------------------------------------------------------
// Common Variables
//-------------------------------------------------------

tSerialPort serial;
tSerial2Port serial2;
tDebugPort dbg;

tRcData rcData;

#ifdef DEVICE_IS_RECEIVER
tTxFrame txFrame, txFrame2;
tRxFrame rxFrame;
#endif
#ifdef DEVICE_IS_TRANSMITTER
tTxFrame txFrame;
tRxFrame rxFrame, rxFrame2;
#endif

SX_DRIVER sx;
SX2_DRIVER sx2;

Stats stats;

FhssBase fhss;

BindBase bind;

tBuzzer buzzer;
tFan fan;


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


void sxSendFrame(uint8_t antenna, void* data, uint8_t len, uint16_t tmo_ms)
{
    if (antenna == ANTENNA_1) {
        sx.SendFrame((uint8_t*)data, len, tmo_ms);
    } else {
        sx2.SendFrame((uint8_t*)data, len, tmo_ms);
    }
}


void sxGetPacketStatus(uint8_t antenna, Stats* stats)
{
    if (antenna == ANTENNA_1) {
        sx.GetPacketStatus(&(stats->last_rx_rssi1), &(stats->last_rx_snr1));
    } else {
        sx2.GetPacketStatus(&(stats->last_rx_rssi2), &(stats->last_rx_snr2));
    }
}


//-------------------------------------------------------
//-- FAIL
//-------------------------------------------------------

typedef enum {
    GR_OFF_RD_BLINK = FAIL_LED_PATTERN_GR_OFF_RD_BLINK,
    RD_OFF_GR_BLINK = FAIL_LED_PATTERN_RD_OFF_GR_BLINK,
    GR_ON_RD_BLINK = FAIL_LED_PATTERN_GR_ON_RD_BLINK,
    RD_ON_GR_BLINK = FAIL_LED_PATTERN_RD_ON_GR_BLINK,
    BLINK_COMMON = FAIL_LED_PATTERN_BLINK_COMMON,
    BLINK_ALTERNATE = FAIL_LED_PATTERN_BLINK_ALTERNATE,
    BLINK_1 = FAIL_LED_PATTERN_RD_BLINK_GR_BLINK1,
    BLINK_2 = FAIL_LED_PATTERN_RD_BLINK_GR_BLINK2,
    BLINK_3 = FAIL_LED_PATTERN_RD_BLINK_GR_BLINK3,
    BLINK_4 = FAIL_LED_PATTERN_RD_BLINK_GR_BLINK4,
    BLINK_5 = FAIL_LED_PATTERN_RD_BLINK_GR_BLINK5,
} FAIL_ENUM;


void FAIL(uint8_t led_pattern, const char* msg)
{
#ifdef FAIL_ENABLED
    fail(&dbg, led_pattern, msg);
#endif
}


void FAIL(const char* msg)
{
    fail(&dbg, 0, msg);
}


void FAILALWAYS(uint8_t led_pattern, const char* msg)
{
    fail(&dbg, led_pattern, msg);
}


//-------------------------------------------------------
//-- LED defines
//-------------------------------------------------------

#define LED_GREEN_ON              led_green_on()
#define LED_RED_ON                led_red_on()

#define LED_GREEN_OFF             led_green_off()
#define LED_RED_OFF               led_red_off()

#define LED_GREEN_TOGGLE          led_green_toggle()
#define LED_RED_TOGGLE            led_red_toggle()


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

STATIC_ASSERT(sizeof(tTxSetup) == 20, "tTxSetup len missmatch")
STATIC_ASSERT(sizeof(tRxSetup) == 36, "tRxSetup len missmatch")
STATIC_ASSERT(sizeof(tSetup) == 38+20+36+8+2, "tSetup len missmatch")

STATIC_ASSERT(sizeof(fhss_config) == sizeof(tFhssConfig) * FHSS_CONFIG_NUM, "fhss_config size missmatch")

#endif // COMMON_H
