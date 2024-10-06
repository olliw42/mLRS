//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// EPS Wifi Bridge Interface Header
//********************************************************
#ifndef TX_ESP_H
#define TX_ESP_H
#pragma once


#include <stdlib.h>
#include <ctype.h>


//-------------------------------------------------------
// ESP Helper
//-------------------------------------------------------

void esp_enable(uint8_t serial_destination)
{
#ifdef USE_ESP_WIFI_BRIDGE_RST_GPIO0
  #ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL
    if (serial_destination == SERIAL_DESTINATION_SERIAL) {  // enable/disable ESP on serial
  #endif
  #ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
    if (serial_destination == SERIAL_DESTINATION_SERIAL2) {  // enable/disable ESP on serial2
  #endif
        esp_gpio0_high(); esp_reset_high();
    } else {
        esp_reset_low();
    }
#endif
}


//-------------------------------------------------------
// Uart/usb helper
//-------------------------------------------------------
// could be added to serial base class, but let's just do it here for now

bool ser_is_full(void)
{
#ifdef DEVICE_HAS_COM_ON_USB
  #ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL
    return !uartb_tx_notfull();
  #endif
  #ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
    return !uartd_tx_notfull();
  #endif
#else
    return false;
#endif
}


bool com_is_full(void)
{
#ifdef DEVICE_HAS_COM_ON_USB
    return usb_tx_full();
#else
    return false;
#endif
}


//-------------------------------------------------------
// ESP WifiBridge class
//-------------------------------------------------------

#define ESP_PASSTHROUGH_TMO_MS  4000 // esptool uses 3 secs, so be a bit more generous


#ifndef USE_ESP_WIFI_BRIDGE

class tTxEspWifiBridge
{
  public:
    void Init(tSerialBase* _comport, tSerialBase* _serialport, uint32_t _serial_baudrate) {}
    void Do(void) {}
    uint8_t Task(void) { return TX_TASK_NONE; }

    void EnterFlash(void) {}
    void EnterPassthrough(void) {}
};

#else

extern volatile uint32_t millis32(void);
extern tTxDisp disp;


typedef enum {
    ESP_DTR_SET = 0x01,
    ESP_RTS_SET = 0x02,
} ESP_DTR_RTS_ENUM;


#ifdef ESP_DTR_RTS_USB
uint8_t esp_dtr_rts(void)
{
    return (usb_dtr_is_set() ? 0 : ESP_DTR_SET) + (usb_rts_is_set() ? 0 : ESP_RTS_SET);
}
#endif


class tTxEspWifiBridge
{
  public:
    void Init(tSerialBase* const _comport, tSerialBase* const _serialport, uint32_t const _serial_baudrate);
    void Do(void);
    uint8_t Task(void);

    void EnterFlash(void);
    void EnterPassthrough(void);

  private:
    void passthrough_do_rts_cts(void);
    void passthrough_do(void);

    tSerialBase* com;
    tSerialBase* ser;
    uint32_t ser_baud;

    bool initialized;
    uint8_t task_pending;

    bool passthrough_is_running;
    uint8_t dtr_rts_last;
};


void tTxEspWifiBridge::Init(tSerialBase* const _comport, tSerialBase* const _serialport, uint32_t const _serial_baudrate)
{
    com = _comport;
    ser = _serialport;
    ser_baud = _serial_baudrate;

    initialized = (com != nullptr && ser != nullptr) ? true : false;

    task_pending = TX_TASK_NONE;

    passthrough_is_running = false;
    dtr_rts_last = 0;
}


uint8_t tTxEspWifiBridge::Task(void)
{
    uint8_t task = task_pending;
    task_pending = TX_TASK_NONE;
    return task;
}


void tTxEspWifiBridge::Do(void)
{
#if defined USE_ESP_WIFI_BRIDGE_RST_GPIO0 && defined USE_ESP_WIFI_BRIDGE_DTR_RTS
    if (!initialized) return;

    uint8_t dtr_rts = esp_dtr_rts();

    if ((dtr_rts_last == (ESP_DTR_SET | ESP_RTS_SET)) && !(dtr_rts & ESP_RTS_SET)) { // == 0x03, & 0x02

        //dbg.puts("\npst");

        passthrough_do_rts_cts();

        task_pending = TX_TASK_RESTART_CONTROLLER;

        //dbg.puts("\nend");delay_ms(500);
    }

    dtr_rts_last = dtr_rts;
#endif
}


void tTxEspWifiBridge::passthrough_do_rts_cts(void)
{
#if defined USE_ESP_WIFI_BRIDGE_RST_GPIO0 && defined USE_ESP_WIFI_BRIDGE_DTR_RTS
    if (!initialized) return;

    uint32_t serial_tlast_ms = millis32();

    disp.DrawNotify("ESP\nFLASHING");
    delay_ms(50); // give display some time

    leds.InitPassthrough();

    uint32_t baudrate = 115200;
    ser->SetBaudRate(baudrate);
    ser->flush();
    com->flush();

    while (1) {

        if (doSysTask) {
            doSysTask = 0;
            leds.TickPassthrough_ms();
        }

        uint8_t dtr_rts = esp_dtr_rts();

        if (dtr_rts != dtr_rts_last) {
            if (dtr_rts & ESP_RTS_SET) esp_reset_high(); else esp_reset_low(); // & 0x02
            if (dtr_rts & ESP_DTR_SET) esp_gpio0_high(); else esp_gpio0_low(); // & 0x01
            //dbg.puts("\ndtr,rts ="); dbg.putc(dtr_rts + '0');
        }
        dtr_rts_last = dtr_rts;

#ifdef DEVICE_HAS_COM_ON_USB
        if (usb_baudrate() != baudrate) {
             baudrate = usb_baudrate();
             ser->SetBaudRate(baudrate);
             ser->flush();
             com->flush();
        }
#endif

        uint32_t tnow_ms = millis32();

        uint16_t cnt = 0;
        while (com->available() && !ser_is_full() && (cnt < 64)) { // works fine without cnt, but needs is_full() check
            char c = com->getc();
            ser->putc(c);
            cnt++;
            serial_tlast_ms = tnow_ms;
        }
        cnt = 0;
        while (ser->available() && !com_is_full() && (cnt < 64)) {
            char c = ser->getc();
            com->putc(c);
            cnt++;
        }

        if (tnow_ms - serial_tlast_ms > ESP_PASSTHROUGH_TMO_MS) {

            passthrough_is_running = false;

            esp_reset_low();
            delay_ms(100);
            esp_reset_high();

            disp.DrawNotify("");

            return;
        }

    }
#endif // USE_ESP_WIFI_BRIDGE_RST_GPIO0 && USE_ESP_WIFI_BRIDGE_DTR_RTS
}


// enter ESP flashing, can only be exited by re-powering
void tTxEspWifiBridge::EnterFlash(void)
{
    if (!initialized) return;

    disp.DrawNotify("FLASH ESP");
    delay_ms(30);

#ifdef USE_ESP_WIFI_BRIDGE_RST_GPIO0
    esp_reset_low();
    esp_gpio0_low();
    delay_ms(10); // delay_ms(100);
    esp_reset_high();
    delay_ms(10); // delay_ms(100);
    esp_gpio0_high();
    delay_ms(10); // delay_ms(100);
#endif

    passthrough_do();
}


// enter ESP passthrough, can only be exited by re-powering
void tTxEspWifiBridge::EnterPassthrough(void)
{
    if (!initialized) return;

    passthrough_do();
}


void tTxEspWifiBridge::passthrough_do(void)
{
    if (!initialized) return;

    leds.InitPassthrough();

    uint32_t baudrate = 115200;
    ser->SetBaudRate(baudrate);
#if defined DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2 && defined DEVICE_HAS_SERIAL_OR_COM
    ser_or_com_set_to_com();
#endif
    ser->flush();
    com->flush();

    while (1) {
        if (doSysTask) {
            doSysTask = 0;
            leds.TickPassthrough_ms();
        }

#ifdef DEVICE_HAS_COM_ON_USB
        if (usb_baudrate() != baudrate) {
             baudrate = usb_baudrate();
             ser->SetBaudRate(baudrate);
             ser->flush();
             com->flush();
        }
#endif

        uint16_t cnt = 0;
        while (com->available() && !ser_is_full() && (cnt < 64)) { // works fine without cnt, but needs is_full() check
            char c = com->getc();
            ser->putc(c);
            cnt++;
        }
        cnt = 0;
        while (ser->available() && !com_is_full() && (cnt < 64)) {
            char c = ser->getc();
            com->putc(c);
            cnt++;
        }
    }
}


#endif // USE_ESP_WIFI_BRIDGE

#endif // TX_ESP_H



/*
void tTxEspWifiBridge::_read_write(void)
{
    uint8_t cnt = 0;
    //while (com->available() && uartb_tx_notfull() && (cnt < 64)) {
    //while (com->available() && (cnt < 64)) { // does not work
    if (com->available() && !ser->full()) { //uartb_tx_notfull()) {
    //if (com->available()) { // does not work
        char c = com->getc();
        ser->putc(c);
        cnt++;
    }
    cnt = 0;
    //while (ser->available() && usb_tx_notfull() && (cnt < 64)) {
    //while (ser->available() && (cnt < 64)) {
    if (ser->available() && !com->full()) { //usb_tx_notfull()) {
    //if (ser->available()) {
        char c = ser->getc();
        com->putc(c);
        cnt++;
    }
} */
