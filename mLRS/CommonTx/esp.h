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
// ESP WifiBridge class
//-------------------------------------------------------

#define ESP_PASSTHROUGH_TMO_MS  3000


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
    void enter_bootloader(void);
    void passthrough_do(bool _reset);

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

    ser->SetBaudRate(115200);

    leds.InitPassthrough();

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

        uint32_t tnow_ms = millis32();

        if (com->available()) {
            char c = com->getc();
            ser->putc(c);
            serial_tlast_ms = tnow_ms;
        }
        if (ser->available()) {
            char c = ser->getc();
            com->putc(c);
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

    passthrough_do(true);
}


// enter ESP passthrough, can only be exited by re-powering
void tTxEspWifiBridge::EnterPassthrough(void)
{
    if (!initialized) return;

    passthrough_do(false);
}


void tTxEspWifiBridge::enter_bootloader(void)
{
#ifdef USE_ESP_WIFI_BRIDGE_RST_GPIO0
    esp_reset_low();
    esp_gpio0_low();
    delay_ms(100);
    esp_reset_high();
    delay_ms(100);
    esp_gpio0_high();
    delay_ms(100);
#endif
}


void tTxEspWifiBridge::passthrough_do(bool _reset)
{
    if (!initialized) return;

    ser->SetBaudRate(115200);
    //com->SetBaudRate(115200);

#if defined DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2 && defined DEVICE_HAS_SERIAL_OR_COM
    ser_or_com_set_to_com();
#endif

    leds.InitPassthrough();

    ser->flush();
    com->flush();

    if (_reset) enter_bootloader();

    while (1) {
        if (doSysTask) {
            doSysTask = 0;
            leds.TickPassthrough_ms();
        }

        //if (com->available() && uartb_tx_notfull()) {
        if (com->available()) {
            char c = com->getc();
            ser->putc(c);
        }
        //if (ser->available() && usb_tx_notfull()) {
        if (ser->available()) {
            char c = ser->getc();
            com->putc(c);
        }
    }
}


#endif // USE_ESP_WIFI_BRIDGE

#endif // TX_ESP_H



