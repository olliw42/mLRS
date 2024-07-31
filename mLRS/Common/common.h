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
#include "leds.h"


//-------------------------------------------------------
// Serial Classes
//-------------------------------------------------------

#if defined DEVICE_IS_TRANSMITTER && defined USE_COM_ON_SERIAL
  // TODO: when we swap ser/com, we may want to flush, we need to change baudrate
#ifdef DEVICE_HAS_SERIAL_ON_USB
  #define SERORCOM_INIT  ser_or_com_init();
#else
  #define SERORCOM_INIT  ser_or_com_init(); if (!ser_or_com_serial()) uartb_setbaudrate(TX_COM_BAUDRATE);
#endif
  #define IFNSER(x)  if (!ser_or_com_serial()) return x;
  #define IFNCOM(x)  if (ser_or_com_serial()) return x;
#else
  #define SERORCOM_INIT
  #define IFNSER(x)
  #define IFNCOM(x)
#endif


// is always uartb (or usb)
class tSerialPort : public tSerialBase
{
#ifdef USE_SERIAL
  public:
#ifdef DEVICE_HAS_SERIAL_ON_USB // USE_USB
    void InitOnce(void) override { usb_init(); }
    void Init(void) override { SERORCOM_INIT; }
    void putbuf(uint8_t* buf, uint16_t len) override { IFNSER(); usb_putbuf(buf, len); }
    bool available(void) override { IFNSER(0); return usb_rx_available(); }
    char getc(void) override { IFNSER(0); return usb_getc(); }
    void flush(void) override { IFNSER(); usb_flush(); }
    uint16_t bytes_available(void) override { IFNSER(0); return usb_rx_bytesavailable(); }
#else
    void Init(void) override { uartb_init(); SERORCOM_INIT; }
    void SetBaudRate(uint32_t baud) override { IFNSER(); uartb_setprotocol(baud, XUART_PARITY_NO, UART_STOPBIT_1); }
    void putbuf(uint8_t* buf, uint16_t len) override { IFNSER(); uartb_putbuf(buf, len); }
    bool available(void) override { IFNSER(0); return uartb_rx_available(); }
    char getc(void) override { IFNSER(0); return uartb_getc(); }
    void flush(void) override { IFNSER(); uartb_rx_flush(); uartb_tx_flush(); }
    uint16_t bytes_available(void) override { IFNSER(0); return uartb_rx_bytesavailable(); }
    bool has_systemboot(void) override { return uartb_has_systemboot(); }
#endif
#endif
};


// is uartc on rx / uartf on tx (or swuart)
class tDebugPort : public tSerialBase
{
#ifdef USE_DEBUG
  public:
#ifdef DEVICE_HAS_DEBUG_SWUART
    void Init(void) { swuart_init(); }
    void putbuf(uint8_t* buf, uint16_t len) override { swuart_putbuf(buf, len); }
#else
#ifdef DEVICE_IS_RECEIVER
    void Init(void) { uartc_init(); }
    void putbuf(uint8_t* buf, uint16_t len) override { uartc_putbuf(buf, len); }
#endif
#ifdef DEVICE_IS_TRANSMITTER
    void Init(void) { uartf_init(); }
    void putbuf(uint8_t* buf, uint16_t len) override { uartf_putbuf(buf, len); }
#endif
#endif
#endif
};


// is uartc or uartb (or usb)
class tComPort : public tSerialBase
{
#ifdef USE_COM_ON_SERIAL
  public:
    // we do not initialize it as it is initialized by serial
#ifdef DEVICE_HAS_SERIAL_ON_USB // USE_USB
    void putbuf(uint8_t* buf, uint16_t len) override { IFNCOM(); usb_putbuf(buf, len); }
    bool available(void) override { IFNCOM(0); return usb_rx_available(); }
    char getc(void) override { IFNCOM(0); return usb_getc(); }
    void flush(void) override { IFNCOM(); usb_flush(); }
#else
    void putbuf(uint8_t* buf, uint16_t len) override { IFNCOM(); uartb_putbuf(buf, len); }
    bool available(void) override { IFNCOM(0); return uartb_rx_available(); }
    char getc(void) override { IFNCOM(0); return uartb_getc(); }
    void flush(void) override { IFNCOM(); uartb_rx_flush(); uartb_tx_flush(); }
#endif
#endif
#ifdef USE_COM
  public:
#ifdef DEVICE_HAS_COM_ON_USB // USE_USB
    void InitOnce(void) override { usb_init(); }
    void Init(void) override { }
    void putbuf(uint8_t* buf, uint16_t len) override { usb_putbuf(buf, len); }
    bool available(void) override { return usb_rx_available(); }
    char getc(void) override { return usb_getc(); }
#else
    void Init(void) override { uartc_init(); }
    void putbuf(uint8_t* buf, uint16_t len) override { uartc_putbuf(buf, len); }
    bool available(void) override { return uartc_rx_available(); }
    char getc(void) override { return uartc_getc(); }
#endif
#endif
};


// is always uartd
class tSerial2Port : public tSerialBase
{
#ifdef USE_SERIAL2
  public:
    void Init(void) override { uartd_init(); }
    void SetBaudRate(uint32_t baud) override { uartd_setprotocol(baud, XUART_PARITY_NO, UART_STOPBIT_1); }
    void putbuf(uint8_t* buf, uint16_t len) override { uartd_putbuf(buf, len); }
    bool available(void) override { return uartd_rx_available(); }
    char getc(void) override { return uartd_getc(); }
    void flush(void) override { uartd_rx_flush(); uartd_tx_flush(); }
    uint16_t bytes_available(void) override { return uartd_rx_bytesavailable(); }
    bool has_systemboot(void) override { return uartd_has_systemboot(); }
#endif
};


//-------------------------------------------------------
// Common Variables
//-------------------------------------------------------

tSerialPort serial;
tDebugPort dbg;
#ifdef DEVICE_IS_TRANSMITTER
tSerial2Port serial2;
tComPort comport;
#endif

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

tStats stats;

tFhss fhss;

tBindBase bind;

#ifdef DEVICE_IS_TRANSMITTER
tBuzzer buzzer;
#endif
tFan fan;
tLEDs leds;


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
#if !defined DEVICE_HAS_DUAL_SX126x_SX128x && !defined DEVICE_HAS_DUAL_SX126x_SX126x
    if (antenna == ANTENNA_1) {
        sx.SendFrame((uint8_t*)data, len, tmo_ms);
        sx2.SetToIdle();
    } else {
        sx2.SendFrame((uint8_t*)data, len, tmo_ms);
        sx.SetToIdle();
    }
#else
    sx.SendFrame((uint8_t*)data, len, tmo_ms);
    sx2.SendFrame((uint8_t*)data, len, tmo_ms);
#endif
}


void sxGetPacketStatus(uint8_t antenna, tStats* stats)
{
    if (antenna == ANTENNA_1) {
        sx.GetPacketStatus(&(stats->last_rssi1), &(stats->last_snr1));
    } else {
        sx2.GetPacketStatus(&(stats->last_rssi2), &(stats->last_snr2));
    }
}


//-------------------------------------------------------
//-- FAIL
//-------------------------------------------------------

typedef enum {
    BLINK_RD_GR_OFF = FAIL_LED_PATTERN_GR_OFF_RD_BLINK,
    BLINK_GR_RD_OFF = FAIL_LED_PATTERN_RD_OFF_GR_BLINK,
    BLINK_RD_GR_ON = FAIL_LED_PATTERN_GR_ON_RD_BLINK,
    BLINK_GR_RD_ON = FAIL_LED_PATTERN_RD_ON_GR_BLINK,
    BLINK_COMMON = FAIL_LED_PATTERN_BLINK_COMMON,
    BLINK_ALTERNATE = FAIL_LED_PATTERN_BLINK_ALTERNATE,
    BLINK_1 = FAIL_LED_PATTERN_RD_BLINK_GR_BLINK1,
    BLINK_2 = FAIL_LED_PATTERN_RD_BLINK_GR_BLINK2,
    BLINK_3 = FAIL_LED_PATTERN_RD_BLINK_GR_BLINK3,
    BLINK_4 = FAIL_LED_PATTERN_RD_BLINK_GR_BLINK4,
    BLINK_5 = FAIL_LED_PATTERN_RD_BLINK_GR_BLINK5,
} FAIL_ENUM;


void FAILALWAYS(uint8_t led_pattern, const char* msg)
{
    fail(&dbg, led_pattern, msg);
}


void FAILALWAYS_WSTATE(uint8_t led_pattern, const char* msg, uint16_t irq_status, uint8_t link_state, uint8_t link_rx1_status, uint8_t link_rx2_status)
{
char s[64];

    strcpy(s, msg);
    strcat(s, " irq=x");
    strcat(s, u16toHEX_s(irq_status));
    strcat(s, " ls=");
    strcat(s, linkstate_str[link_state]);
    strcat(s, " rx1s=");
    strcat(s, rxstatus_str[link_rx1_status]);
    strcat(s, " rx2s=");
    strcat(s, rxstatus_str[link_rx2_status]);
    fail(&dbg, led_pattern, s);
}


void FAIL_WPATTERN(uint8_t led_pattern, const char* msg)
{
#ifdef FAIL_ENABLED
    fail(&dbg, led_pattern, msg);
#endif
}


void FAIL_WMSG(const char* msg)
{
#ifdef FAIL_ENABLED
    fail(&dbg, 0, msg);
#endif
}


void FAIL_WSTATE(uint8_t led_pattern, const char* msg, uint16_t irq_status, uint8_t link_state, uint8_t link_rx1_status, uint8_t link_rx2_status)
{
#ifdef FAIL_ENABLED
    FAILALWAYS_WSTATE(led_pattern, msg, irq_status, link_state, link_rx1_status, link_rx2_status);
#endif
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

STATIC_ASSERT(sizeof(tRxSetup) == 36, "tRxSetup len missmatch")
STATIC_ASSERT(sizeof(tTxSetup) == 20, "tTxSetup len missmatch")
STATIC_ASSERT(sizeof(tCommonSetup) == 16, "tCommonSetup len missmatch")
STATIC_ASSERT(sizeof(tSetup) == 22+16+36+(20+16)*SETUP_CONFIG_NUM+8+2, "tSetup len missmatch")

STATIC_ASSERT(sizeof(fhss_config) == sizeof(tFhssConfig) * FHSS_CONFIG_NUM, "fhss_config size missmatch")

#endif // COMMON_H
