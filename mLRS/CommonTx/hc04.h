//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// HC04 BT Bridge Interface Header
//********************************************************
#ifndef TX_HC04_H
#define TX_HC04_H
#pragma once


#include <stdlib.h>
#include <ctype.h>


//-------------------------------------------------------
// HC04 Bridge class
//-------------------------------------------------------

#ifndef USE_HC04_MODULE

class tTxHc04Bridge
{
  public:
    void Init(tSerialBase* _comport, tSerialBase* _serialport, uint32_t _serial_baudrate) {}
    uint8_t Task(void) { return TX_TASK_NONE; }

    void EnterPassthrough(void) {}
    void SepPin(uint16_t pin) {}
};

#else

extern volatile uint32_t millis32(void);
extern tTxDisp disp;


class tTxHc04Bridge
{
  public:
    void Init(tSerialBase* _comport, tSerialBase* _serialport, uint32_t _serial_baudrate);
    uint8_t Task(void);

    void EnterPassthrough(void);
    void SepPin(uint16_t pin);

  private:
    void run_autoconfigure(void);
    void passthrough_do(void);
    void hc04_read(const char* cmd, uint8_t* res, uint8_t* len);
    void hc04_configure(void);

    tSerialBase* com;
    tSerialBase* ser;
    uint32_t ser_baud;

    uint8_t task_pending;
};


void tTxHc04Bridge::Init(tSerialBase* _comport, tSerialBase* _serialport, uint32_t _serial_baudrate)
{
    com = _comport;
    ser = _serialport;
    ser_baud = _serial_baudrate;

    task_pending = TX_TASK_NONE;

    run_autoconfigure();
}


uint8_t tTxHc04Bridge::Task(void)
{
    uint8_t task = task_pending;
    task_pending = TX_TASK_NONE;
    return task;
}


// enter HC04 passthrough, can only be exited by re-powering
void tTxHc04Bridge::EnterPassthrough(void)
{
    if (com == nullptr || ser == nullptr) return; // we need both for passthrough

    passthrough_do();
}


void tTxHc04Bridge::SepPin(uint16_t pin)
{
uint8_t s[34];
uint8_t len;

    if (pin < 1000) pin = 1000;
    if (pin > 9999) pin = 9999;

    char pin_str[16];
    u16toBCDstr(pin, pin_str);
    remove_leading_zeros(pin_str);
    char cmd_str[16];
    strcpy(cmd_str, "AT+PIN=");
    strcat(cmd_str, pin_str);

    hc04_read(cmd_str, s, &len);
    if (len > 3 && s[0] == 'O' && s[1] == 'K' && s[len-2] == 0x0D) {
        delay_ms(1500);
    }
}


void tTxHc04Bridge::passthrough_do(void)
{
    ser->SetBaudRate(ser_baud);
    ser->flush();
#if defined DEVICE_HAS_SERIAL_OR_COM && defined DEVICE_HAS_HC04_MODULE_ON_SERIAL2
    ser_or_com_set_to_com();
#endif

    leds.InitPassthrough();
    disp.DrawNotify("HC04\nPASSTHRU");

    while (1) {
        if (doSysTask) {
            doSysTask = 0;
            leds.TickPassthrough_ms();
        }

        if (com->available()) {
            char c = com->getc();
            ser->putc(c);
        }
        if (ser->available()) {
            char c = ser->getc();
            com->putc(c);
        }
    }
}


void tTxHc04Bridge::hc04_read(const char* cmd, uint8_t* res, uint8_t* len)
{
    ser->puts(cmd);

    *len = 0;
    uint32_t tnow_ms = millis32();
    char c = 0x00;
    while (*len < 32 && (millis32() - tnow_ms) < 100) { // TODO: check timing, to see if this can be shortened
        if (ser->available()) {
            c = ser->getc();
            res[(*len)++] = c;
            if (c == 0x0A) return;
        }
    }
    *len = 0;
}


void tTxHc04Bridge::hc04_configure(void)
{
uint8_t s[34];
uint8_t len;

    hc04_read("AT+NAME=Matek-mLRS-BT", s, &len);
    if (len > 3 && s[0] == 'O' && s[1] == 'K' && s[len-2] == 0x0D) {
    } else {
        return;
    }

    delay_ms(1500); // 500 did not work, 1000 did work, so play it safe

    //hc04_read("AT+BAUD=115200", s, &len);
    char baud_str[32];
    u32toBCDstr(ser_baud, baud_str);
    remove_leading_zeros(baud_str);
    char cmd_str[32];
    strcpy(cmd_str, "AT+BAUD=");
    strcat(cmd_str, baud_str);
    strcat(cmd_str, ",N");
    hc04_read(cmd_str, s, &len); // AT+BAUD seems to send response with "old" baud rate, and only when changes to the new
    if (len > 3 && s[0] == 'O' && s[1] == 'K' && s[len-2] == 0x0D) {
    } else {
        return;
    }

    delay_ms(1500);

    ser->SetBaudRate(ser_baud);
    ser->flush();

    hc04_read("AT", s, &len);
    if (len > 3 && s[0] == 'O' && s[1] == 'K' && s[len-2] == 0x0D) { // found !
    }
}


void tTxHc04Bridge::run_autoconfigure(void)
{
uint8_t s[34];
uint8_t len;

    if (ser == nullptr) return; // we need a serial

    uint32_t bauds[7] = { ser_baud, 9600, 19200, 38400, 57600, 115200, 230400 };

    for (uint8_t baud_idx = 0; baud_idx < 7; baud_idx++) {
        ser->SetBaudRate(bauds[baud_idx]);
        ser->flush();

        hc04_read("AT+NAME=?", s, &len);
        if (len > 3 && s[0] == 'O' && s[1] == 'K' && s[len-2] == 0x0D) { // found !
            s[len-2] = '\0';
            if (!strcmp((char*)s, "OK+NAME=Matek-mLRS-BT") && bauds[baud_idx] == ser_baud) { // is correct name and baud rate
                return;
            } else {
                hc04_configure();
                ser->SetBaudRate(ser_baud);
                ser->flush();
                return;
            }
        }
    }
}


#endif // USE_HC04_MODULE

#endif // TX_HC04_H



