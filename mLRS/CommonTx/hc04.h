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
    void Init(tSerialBase* const _comport, tSerialBase* const _serialport, uint32_t const _serial_baudrate) {}
    uint8_t Task(void) { return TX_TASK_NONE; }

    void EnterPassthrough(void) {}
    void SetPin(uint16_t pin) {}
};

#else

extern volatile uint32_t millis32(void);
extern tTxDisp disp;


class tTxHc04Bridge
{
  public:
    void Init(tSerialBase* const _comport, tSerialBase* const _serialport, uint32_t const _serial_baudrate);
    uint8_t Task(void);

    void EnterPassthrough(void);
    void SetPin(uint16_t pin);

  private:
    void run_autoconfigure(void);
    bool hc04_read(const char* const cmd, uint8_t* const res, uint8_t* const len);
    void hc04_configure(char* const ok_device_name);

    void passthrough_do(void);

    tSerialBase* com;
    tSerialBase* ser;
    uint32_t ser_baud;

    uint8_t task_pending;
};


void tTxHc04Bridge::Init(tSerialBase* const _comport, tSerialBase* const _serialport, uint32_t const _serial_baudrate)
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


void tTxHc04Bridge::SetPin(uint16_t pin)
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

    if (hc04_read(cmd_str, s, &len)) {
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
        if (doSysTask()) {
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


#define HC04_DBG(x)

#define HC04_CMDRES_LEN     32
#define HC04_CMDRES_TMO_MS  100


bool tTxHc04Bridge::hc04_read(const char* const cmd, uint8_t* const res, uint8_t* const len)
{
    ser->puts(cmd);
HC04_DBG(dbg.puts("\r\n");dbg.puts(cmd);dbg.puts(" -> ");)
    *len = 0;
    uint32_t tnow_ms = millis32();
    char c = 0x00;
    while (*len < HC04_CMDRES_LEN && (millis32() - tnow_ms) < HC04_CMDRES_TMO_MS) { // TODO: check timing, to see if this can be shortened
        if (ser->available()) {
            c = ser->getc();
HC04_DBG(dbg.putc(c);)
            res[(*len)++] = c;
            if (c == 0x0A) { // '\n'
HC04_DBG(dbg.puts("!ENDE!");)
                return (*len > 3 && res[0] == 'O' && res[1] == 'K' && res[*len - 2] == 0x0D); // '\r'
            }
        }
    }
    *len = 0;
    return false;
}


void tTxHc04Bridge::hc04_configure(char* const ok_device_name)
{
uint8_t s[HC04_CMDRES_LEN+2];
uint8_t len;

    // ok_device_name looks like "OK+NAME=Matek-mLRS-xxxxx BT"
    // so to set we simply change the first two chars to "AT"
    // we can do this as we don't need ok_device_name anymore
    ok_device_name[0] = 'A';
    ok_device_name[1] = 'T';
    if (!hc04_read(ok_device_name, s, &len)) { // set name
        return;
    }

    delay_ms(1500); // 500 did not work, 1000 did work, so play it safe

/*    if (!hc04_read("AT+PIN=1234", s, &len)) { // set PIN to default
        return;
    }
    delay_ms(1500); */

    //hc04_read("AT+BAUD=115200", s, &len);
    char baud_str[32];
    u32toBCDstr(ser_baud, baud_str);
    remove_leading_zeros(baud_str);
    char cmd_str[32];
    strcpy(cmd_str, "AT+BAUD=");
    strcat(cmd_str, baud_str);
    strcat(cmd_str, ",N");
    if (!hc04_read(cmd_str, s, &len)) { // AT+BAUD seems to send response with "old" baud rate, and only when it changes to the new
        return;
    }

    delay_ms(1500);

    ser->SetBaudRate(ser_baud);
    ser->flush();

    if (hc04_read("AT", s, &len)) { // found !
    }
}


void tTxHc04Bridge::run_autoconfigure(void)
{
uint8_t s[HC04_CMDRES_LEN+2];
uint8_t len;

    if (ser == nullptr) return; // we need a serial

    // we assume the HC04 has not gotten data for enough time so it can respond to AT commands

    uint32_t bauds[7] = { ser_baud, 9600, 19200, 38400, 57600, 115200, 230400 };

    char ok_device_name[48];
    uint8_t uid[STM32_MCU_UID_LEN];
    mcu_uid(uid);
    uint16_t device_id = 0;
    for (uint8_t i = 0; i < STM32_MCU_UID_LEN - 1; i++) device_id += uid[i] + ((uint16_t)uid[i + 1] << 8);
    strcpy(ok_device_name, "OK+NAME=Matek-mLRS-");
    strcat(ok_device_name, u16toBCD_s(device_id));
    strcat(ok_device_name, "-BT");

    for (uint8_t baud_idx = 0; baud_idx < 7; baud_idx++) {
        ser->SetBaudRate(bauds[baud_idx]);
        ser->flush();

        if (hc04_read("AT+NAME=?", s, &len)) { // found !
            s[len-2] = '\0';
            if (!strcmp((char*)s, ok_device_name) && bauds[baud_idx] == ser_baud) { // is correct name and baud rate
                return;
            } else
            if (!strncmp((char*)s, "OK+NAME=", 8)) {
                hc04_configure(ok_device_name);
                ser->SetBaudRate(ser_baud);
                ser->flush();
                return;
            }
        }
    }

    // not found, ensure serial has correct baud rate
    ser->SetBaudRate(ser_baud);
    ser->flush();
}


#endif // USE_HC04_MODULE

#endif // TX_HC04_H



