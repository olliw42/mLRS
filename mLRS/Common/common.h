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
#include "rf_power.h"
#include "setup_types.h"


//-------------------------------------------------------
// Serial Classes
//-------------------------------------------------------

// tx: serial or com, rx: serial
class tUartBPort : public tSerialBase
{
#if defined USE_SERIAL || defined USE_COM_ON_SERIAL
  public:
    void Init(void) override { uartb_init(); }
    void SetBaudRate(uint32_t baud) override { uartb_setprotocol(baud, XUART_PARITY_NO, UART_STOPBIT_1); }
    bool full(void) override { return !uartb_tx_notfull(); }
    void putbuf(uint8_t* const buf, uint16_t len) override { uartb_putbuf(buf, len); }
    bool available(void) override { return uartb_rx_available(); }
    char getc(void) override { return uartb_getc(); }
    void flush(void) override { uartb_rx_flush(); uartb_tx_flush(); }
    uint16_t bytes_available(void) override { return uartb_rx_bytesavailable(); }
    bool has_systemboot(void) override { return uartb_has_systemboot(); }
#endif
};


// tx: com
class tUartCPort : public tSerialBase
{
#if defined USE_COM && !defined DEVICE_HAS_COM_ON_USB
  public:
    void Init(void) override { uartc_init(); }
    bool full(void) override { return !uartc_tx_notfull(); }
    void putbuf(uint8_t* const buf, uint16_t len) override { uartc_putbuf(buf, len); }
    bool available(void) override { return uartc_rx_available(); }
    char getc(void) override { return uartc_getc(); }
#endif
};


// tx: serial or com
class tUsbPort : public tSerialBase
{
#ifdef USE_USB
  public:
    void InitOnce(void) override { usb_init(); }
    bool full(void) override { return usb_tx_full(); }
    void putbuf(uint8_t* const buf, uint16_t len) override { usb_putbuf(buf, len); }
    bool available(void) override { return usb_rx_available(); }
    char getc(void) override { return usb_getc(); }
    void flush(void) override { usb_flush(); }
    uint16_t bytes_available(void) override { return usb_rx_bytesavailable(); }
#endif
};


// tx: serial2
class tUartDPort : public tSerialBase
{
#ifdef USE_SERIAL2
  public:
    void Init(void) override { uartd_init(); }
    void SetBaudRate(uint32_t baud) override { uartd_setprotocol(baud, XUART_PARITY_NO, UART_STOPBIT_1); }
    bool full(void) override { return !uartd_tx_notfull(); }
    void putbuf(uint8_t* const buf, uint16_t len) override { uartd_putbuf(buf, len); }
    bool available(void) override { return uartd_rx_available(); }
    char getc(void) override { return uartd_getc(); }
    void flush(void) override { uartd_rx_flush(); uartd_tx_flush(); }
    uint16_t bytes_available(void) override { return uartd_rx_bytesavailable(); }
    bool has_systemboot(void) override { return uartd_has_systemboot(); }
#endif
};


// is always uartf (or swuart)
class tDebugPort : public tSerialBase
{
#ifdef USE_DEBUG
  public:
#ifdef DEVICE_HAS_DEBUG_SWUART
    void Init(void) override { swuart_init(); }
    void putbuf(uint8_t* const buf, uint16_t len) override { swuart_putbuf(buf, len); }
#else
    void Init(void) override { uartf_init(); }
    void putbuf(uint8_t* const buf, uint16_t len) override { uartf_putbuf(buf, len); }
#endif
#endif
};


// rx: dronecan
#if defined DEVICE_HAS_DRONECAN && defined DEVICE_IS_RECEIVER
#include "../CommonRx/dronecan_interface_rx_types.h"
extern tRxDroneCan dronecan;
#endif

class tDroneCANPort : public tSerialBase
{
#ifdef DEVICE_HAS_DRONECAN
  public:
    void putbuf(uint8_t* buf, uint16_t len) override { dronecan.putbuf(buf, len); }
    bool available(void) override { return dronecan.available(); }
    char getc(void) override { return dronecan.getc(); }
    void flush(void) override { dronecan.flush(); }
    uint16_t bytes_available(void) override { return dronecan.bytes_available(); }
#endif
};


//-------------------------------------------------------
// SerialPorts Class
//-------------------------------------------------------

#ifdef DEVICE_IS_TRANSMITTER
tUartBPort uartb_port; // usually serial port
tUartCPort uartc_port; // usually COM/CLI port
tUartDPort uartd_port; // usually serial2 BT/ESP port
tUsbPort usb_port;

typedef struct
{
    tSerialBase* serial;
    tSerialBase* com;
    tSerialBase* jrpin5serial; // can be nullptr!
    tSerialBase* uartb;
    tSerialBase* uartc;
    tSerialBase* uartd;
    tSerialBase* usb;

    void Init(uint8_t serial_destination, uint32_t baud);

    tSerialBase* com_port(void);
    void set_serial_com_swapped(void);

#ifdef USE_COM_ON_SERIAL
    tSerialBase* ser_or_com_set_to_com(void);
#endif
} tSerialPorts;


tSerialBase* tSerialPorts::com_port(void) // uartc or usb
{
#ifdef DEVICE_HAS_COM_ON_USB
    return &usb_port;
#else
    return &uartc_port;
#endif
}

void tSerialPorts::set_serial_com_swapped(void)
{
    serial = com_port();
    com = &uartb_port;
}


#ifdef USE_COM_ON_SERIAL
// device does not have an extra com port, but allows us to use the serial port as com by e.g. a button thing
// is determined at startup
// note: in that case, UARTC is not used and tUartCPort is a dummy port
// let's check that
#if defined USE_COM && !defined DEVICE_HAS_COM_ON_USB
  #error UARTC is not a dummy port!
#endif

tSerialBase* tSerialPorts::ser_or_com_set_to_com(void)
{
    set_serial_com_swapped();
    return com;
}
#endif


void tSerialPorts::Init(uint8_t serial_destination, uint32_t baud)
{
#if defined USE_COM_ON_SERIAL // button forces com onto uartB serial pins (no separate com port)
    if (!ser_or_com_init()) { serial_destination = SERIAL_DESTINATION_COM; } // force swap
#elif defined DEVICE_HAS_SERIAL_OR_COM // button forces com onto uartC or usb
    if (!ser_or_com_init()) { serial_destination = 0; } // force default
#endif

    switch (serial_destination) {
    case SERIAL_DESTINATION_SERIAL2:
        serial = &uartd_port;
        com = com_port();
        break;
    case SERIAL_DESTINATION_COM:
        set_serial_com_swapped();
        break;
    default:
        serial = &uartb_port;
        com = com_port();
    }

    jrpin5serial = nullptr;

    uartb = &uartb_port;
    uartc = &uartc_port;
    uartd = &uartd_port;
    usb = &usb_port;

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_AX12_ESP32
    serial->SetBaudRate(460800); // dirty workaround, fixed baud rate for AX12 due to limitation
#else
    serial->SetBaudRate(baud);
#endif
    // ensure com has correct baud rate
    com->SetBaudRate(TX_COM_BAUDRATE);
}

#endif // DEVICE_IS_TRANSMITTER
#ifdef DEVICE_IS_RECEIVER
tUartBPort uartb_port;
tDroneCANPort dronecan_port;
tSerialBase* serial;

void serial_ports_init(bool is_serial)
{
#if defined USE_SERIAL && defined DEVICE_HAS_DRONECAN
    if (is_serial) {
        serial = &uartb_port; // assign uartb to serial
    } else {
        serial = &dronecan_port; // assign dronecan to serial
    }
#elif defined DEVICE_HAS_DRONECAN
    serial = &dronecan_port;
#elif defined USE_SERIAL
    serial = &uartb_port;
#endif
}
#endif // DEVICE_IS_RECEIVER


//-------------------------------------------------------
// Common Variables
//-------------------------------------------------------

#ifdef DEVICE_IS_TRANSMITTER
tSerialPorts serials;
#endif
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

tStats stats;
tFhss fhss;
tBindBase bind;

#ifdef DEVICE_IS_TRANSMITTER
tBuzzer buzzer;
#endif
tFan fan;
tLEDs leds;
tRfPower rfpower;


//-------------------------------------------------------
// Sx/Sx2 convenience wrapper
//-------------------------------------------------------

void sxReadFrame(uint8_t antenna, void* const data, void* const data2, uint8_t len)
{
    if (antenna == ANTENNA_1) {
        sx.ReadFrame((uint8_t*)data, len); // should never happen that SX is not set up when antenna1
    } else {
        sx2.ReadFrame((uint8_t*)data2, len); // should never happen that SX2 is not set up when antenna2
    }
}


void sxSendFrame(uint8_t antenna, void* const data, uint8_t len, uint16_t tmo_ms)
{
#if defined DEVICE_HAS_DUAL_SX126x_SX128x || defined DEVICE_HAS_DUAL_SX126x_SX126x // DUAL BAND
    if (Config.IsDualBand) {
        sx.SendFrame((uint8_t*)data, len, tmo_ms);
        sx2.SendFrame((uint8_t*)data, len, tmo_ms);
    } else
#endif
    if (antenna == ANTENNA_1) {
        sx.SendFrame((uint8_t*)data, len, tmo_ms); // should never happen that SX is not set up when antenna1
        IF_SX2(sx2.SetToIdle();)
    } else {
        sx2.SendFrame((uint8_t*)data, len, tmo_ms); // should never happen that SX2 is not set up when antenna2
        IF_SX(sx.SetToIdle();)
    }
}


void sxGetPacketStatus(uint8_t antenna, tStats* const stats)
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


void FAILALWAYS(uint8_t led_pattern, const char* const msg)
{
    fail(&dbg, led_pattern, msg);
}


void FAILALWAYS_WSTATE(
    uint8_t led_pattern,
    const char* const msg,
    uint16_t irq_status,
    uint8_t link_state,
    uint8_t link_rx1_status,
    uint8_t link_rx2_status)
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


void FAIL_WPATTERN(uint8_t led_pattern, const char* const msg)
{
#ifdef FAIL_ENABLED
    fail(&dbg, led_pattern, msg);
#endif
}


void FAIL_WMSG(const char* const msg)
{
#ifdef FAIL_ENABLED
    fail(&dbg, 0, msg);
#endif
}


void FAIL_WSTATE(
    uint8_t led_pattern,
    const char* const msg,
    uint16_t irq_status,
    uint8_t link_state,
    uint8_t link_rx1_status,
    uint8_t link_rx2_status)
{
#ifdef FAIL_ENABLED
    FAILALWAYS_WSTATE(led_pattern, msg, irq_status, link_state, link_rx1_status, link_rx2_status);
#endif
}


//-------------------------------------------------------
//-- check some sizes
//-------------------------------------------------------

STATIC_ASSERT(sizeof(tTxFrameStatus) == FRAME_TX_RX_HEADER_LEN - 2, "tTxFrameStatus len missmatch")
STATIC_ASSERT(sizeof(tRxFrameStatus) == FRAME_TX_RX_HEADER_LEN - 2, "tRxFrameStatus len missmatch")
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

STATIC_ASSERT(sizeof(fhss_config) == sizeof(tFhssConfig) * SX_FHSS_FREQUENCY_BAND_NUM, "fhss_config size missmatch")

#endif // COMMON_H
