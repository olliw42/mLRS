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


extern volatile uint32_t millis32(void);


//-------------------------------------------------------
// WEPS WifiBridge class
//-------------------------------------------------------

#define ESP_PASSTHROUGH_TMO_MS  3000


typedef enum {
    ESP_TASK_NONE = 0,
    ESP_TASK_RESTART_CONTROLLER,
} ESP_TASK_ENUM;


#if !(defined USE_ESP_WIFI_BRIDGE_DTR_RTS && defined USE_ESP_WIFI_BRIDGE_RST_GPIO0)
class tTxEspWifiBridge
{
  public:
    void Init(tSerialBase* _com, tSerialBase* _ser) {}
    void Do(void) {}
    uint8_t Task(void) { return ESP_TASK_NONE; }
};
#else

class tTxEspWifiBridge
{
  public:
    void Init(tSerialBase* _com, tSerialBase* _ser);
    void Do(void);
    uint8_t Task(void);

  private:
    void passthrough_do(void);

    tSerialBase* com;
    tSerialBase* ser;

    bool initialized;

    bool passthrough_is_running;
    uint8_t dtr_rts_last;
    uint8_t task;
};


void tTxEspWifiBridge::Init(tSerialBase* _com, tSerialBase* _ser)
{
    com = _com;
    ser = _ser;

    initialized = (com != nullptr && ser != nullptr) ? true : false;

    passthrough_is_running = false;
    dtr_rts_last = 0;
    task = ESP_TASK_NONE;
}


void tTxEspWifiBridge::Do(void)
{
    if (!initialized) return;

    uint8_t dtr_rts = esp_dtr_rts();

    if ((dtr_rts_last == 3) && !(dtr_rts & 0x02)) {

        //dbg.puts("\npst");

        passthrough_do();

        task = ESP_TASK_RESTART_CONTROLLER;

        //dbg.puts("\nend");delay_ms(500);
    }

    dtr_rts_last = dtr_rts;
}


uint8_t tTxEspWifiBridge::Task(void)
{
    return task;
}


void tTxEspWifiBridge::passthrough_do(void)
{
    if (!initialized) return;

    uint16_t led_blink = 0;
    uint32_t serial_tlast_ms = millis32();

    disp.DrawNotify("ESP\nFLASHING");

    ser->SetBaudRate(115200);

    LED_RED_OFF;
    LED_GREEN_ON;

    while (1) {

        if (doSysTask) {
            doSysTask = 0;
            DECc(led_blink, SYSTICK_DELAY_MS(100));
            if (!led_blink) { LED_GREEN_TOGGLE; LED_RED_TOGGLE; }
        }

        uint8_t dtr_rts = esp_dtr_rts();

        if (dtr_rts != dtr_rts_last) {
            if (dtr_rts & 0x02) esp_reset_high(); else esp_reset_low();
            if (dtr_rts & 0x01) esp_gpio0_high(); else esp_gpio0_low();
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
}


#endif // USE_ESP_WIFI_BRIDGE_DTR_RTS

#endif // TX_ESP_H



