//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// ESP Wifi Bridge Interface Header
//********************************************************
//
// USE_ESP_WIFI_BRIDGE
//   is defined when DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL || DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
//
// USE_ESP_WIFI_BRIDGE_RST_GPIO0
//   is defined when RESET & GPIO0 pin handling is available (ESP_RESET & ESP_GPIO0 defined)
//   this allows:
//   - flashing via passthrough, flash mode invoked by "FLASH ESP" command
//   - ESP TX parameters, esp configuration
//
// USE_ESP_WIFI_BRIDGE_DTR_RTS
//   is defined when DTR & RTS pin handling is available (ESP_DTR & ESP_RTS defined)
//   this allows in addition:
//   - flashing via passthrough, flash mode entered via DTR/RTS, no need for invoking "FLASH ESP" command
//
// USE_ESP_WIFI_BRIDGE_BOOT0
//   is defined when BOOT0 pin handling is available (ESP_BOOT0 defined)
//   this allows in addition:
//   - flashing via passthrough, flash mode entered via BOOT0, no need for invoking "FLASH ESP" command
//********************************************************
#ifndef TX_ESP_H
#define TX_ESP_H
#pragma once


#include <stdlib.h>
#include <ctype.h>


#if defined DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL && defined USE_COM_ON_SERIAL
  #error ESP: ESP wireless bridge is on serial but board has serial/com !
#endif


//-------------------------------------------------------
// ESP Helper
//-------------------------------------------------------

// called in init_hw() to reset ESP early on, so it is hopefully booted when we attempt auto configure
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
        esp_reset_low(); // hold it in reset, should be already so but ensure it
    }
#endif
}


//-------------------------------------------------------
// ESP WifiBridge class
//-------------------------------------------------------

#if defined USE_ESP_WIFI_BRIDGE_RST_GPIO0 && defined DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE
  // we currently require RST, GPIO0 for ESP configuration option
  #define ESP_STARTUP_CONFIGURE
#endif


#define ESP_PASSTHROUGH_TMO_MS  4000 // esptool uses 3 secs, so be a bit more generous


#ifndef USE_ESP_WIFI_BRIDGE

class tTxEspWifiBridge
{
  public:
    void Init(tSerialBase* const _comport, tSerialBase* const _serialport, tSerialBase* const _serial2port, uint32_t _serial_baudrate, tTxSetup* const _tx_setup, tCommonSetup* const _common_setup) {}
    void Do(void) {}

    void EnterFlash(void) {}
    void EnterPassthrough(void) {}
};

#else

#include "../Common/tasks.h"


extern volatile uint32_t millis32(void);
extern tTasks tasks;


typedef enum {
    ESP_DTR_SET = 0x01,
    ESP_RTS_SET = 0x02,
} ESP_DTR_RTS_ENUM;


class tTxEspWifiBridge
{
  public:
    void Init(tSerialBase* const _comport, tSerialBase* const _serialport, tSerialBase* const _serial2port, uint32_t _serial_baudrate, tTxSetup* const _tx_setup, tCommonSetup* const _common_setup);
    void Do(void);

    void EnterFlash(void);
    void EnterPassthrough(void);

  private:
#ifdef ESP_STARTUP_CONFIGURE
    bool esp_read(const char* const cmd, char* const res, uint8_t* const len);
    void esp_wait_after_read(const char* const res);
    void esp_configure_baudrate(void);
    void esp_configure_wifiprotocol(void);
    void esp_configure_wifichannel(void);
    void esp_configure_wifipower(void);
    void esp_configure_bindphrase(void);
    void run_configure(void);
#endif

    void passthrough_do_flashing(void);
    void passthrough_do(void);

    tTxSetup* tx_setup;
    tCommonSetup* common_setup;

    tSerialBase* com;
    tSerialBase* ser;
    uint32_t ser_baud;

    bool passthrough; // indicates passthrough is possible

    uint8_t dtr_rts_last;
    uint8_t boot0_last;

    uint32_t version;
};


void tTxEspWifiBridge::Init(
    tSerialBase* const _comport,
    tSerialBase* const _serialport,
    tSerialBase* const _serial2port,
    uint32_t _serial_baudrate,
    tTxSetup* const _tx_setup,
    tCommonSetup* const _common_setup)
{
    tx_setup = _tx_setup;
    common_setup = _common_setup;

    com = _comport;
    ser = nullptr;
    if (tx_setup->SerialDestination == SERIAL_DESTINATION_SERIAL) {
#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL
        ser = _serialport;
#endif
    } else
    if (tx_setup->SerialDestination == SERIAL_DESTINATION_SERIAL2) {
#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
        ser = _serial2port;
#endif
    }
    ser_baud = _serial_baudrate;

    passthrough = (com != nullptr && ser != nullptr); // we need both for passthrough

    dtr_rts_last = 0;
    boot0_last = 0;

    version = 0; // unknown

#ifdef ESP_STARTUP_CONFIGURE
    run_configure();
#endif
}


void tTxEspWifiBridge::Do(void)
{
#if defined USE_ESP_WIFI_BRIDGE_RST_GPIO0 && (defined USE_ESP_WIFI_BRIDGE_DTR_RTS || defined USE_ESP_WIFI_BRIDGE_BOOT0)
    if (!passthrough) return;

#ifdef USE_ESP_WIFI_BRIDGE_DTR_RTS
    uint8_t dtr_rts = esp_dtr_rts();

    if ((dtr_rts_last == (ESP_DTR_SET | ESP_RTS_SET)) && !(dtr_rts & ESP_RTS_SET)) { // toggle 0x03 -> 0x02
        passthrough_do_flashing();
        tasks.SetEspTask(MAIN_TASK_RESTART_CONTROLLER);
    }

    dtr_rts_last = dtr_rts;
#endif
#ifdef USE_ESP_WIFI_BRIDGE_BOOT0
    uint8_t boot0 = esp_boot0();

    if (boot0_last == 1 && boot0 == 0) { // toggle 1 -> 0
        passthrough_do_flashing();
        tasks.SetEspTask(MAIN_TASK_RESTART_CONTROLLER);
    }

    boot0_last = boot0;
#endif
#endif // USE_ESP_WIFI_BRIDGE_RST_GPIO0 && (USE_ESP_WIFI_BRIDGE_DTR_RTS || USE_ESP_WIFI_BRIDGE_BOOT0)
}


void tTxEspWifiBridge::passthrough_do_flashing(void)
{
#if defined USE_ESP_WIFI_BRIDGE_RST_GPIO0 && (defined USE_ESP_WIFI_BRIDGE_DTR_RTS || defined USE_ESP_WIFI_BRIDGE_BOOT0)
    uint32_t serial_tlast_ms = millis32();

    disp.DrawNotify("ESP\nFLASHING");
    delay_ms(50); // give display some time

#ifdef USE_ESP_WIFI_BRIDGE_BOOT0
    esp_reset_low();
    esp_gpio0_low();
    delay_ms(10);
    esp_reset_high();
    delay_ms(100); // 10 ms is too short, ESP8285 needs more time
    esp_gpio0_high();
    delay_ms(10);
#endif

    leds.InitPassthrough();

    uint32_t baudrate = 115200; // Note: this is what is used for flashing, can be different to ESP_CONFIGURE setting
    ser->SetBaudRate(baudrate);
    com->SetBaudRate(baudrate); // Standard tools should specify 115200 to avoid baudrate change
    ser->flush();
    com->flush();

    while (1) {
        if (doSysTask()) {
            leds.TickPassthrough_ms();
        }

#ifdef USE_ESP_WIFI_BRIDGE_DTR_RTS
        uint8_t dtr_rts = esp_dtr_rts();
        if (dtr_rts != dtr_rts_last) {
            if (dtr_rts & ESP_RTS_SET) esp_reset_high(); else esp_reset_low(); // & 0x02
            if (dtr_rts & ESP_DTR_SET) esp_gpio0_high(); else esp_gpio0_low(); // & 0x01
        }
        dtr_rts_last = dtr_rts;
#endif

        uint32_t tnow_ms = millis32();

        uint16_t cnt = 0;
        while (com->available() && !ser->full() && (cnt < 64)) { // works fine without cnt, but needs is_full() check
            char c = com->getc();
            ser->putc(c);
            cnt++;
            serial_tlast_ms = tnow_ms;
        }
        cnt = 0;
        while (ser->available() && !com->full() && (cnt < 64)) {
            char c = ser->getc();
            com->putc(c);
            cnt++;
        }

#ifndef USE_ESP_WIFI_BRIDGE_BOOT0
        if (tnow_ms - serial_tlast_ms > ESP_PASSTHROUGH_TMO_MS) {
            // reset ESP
            esp_reset_low();
            delay_ms(100);
            esp_reset_high();
            // finish
            disp.DrawNotify("");
            return;
        }
#endif

    }
#endif // USE_ESP_WIFI_BRIDGE_RST_GPIO0 && (USE_ESP_WIFI_BRIDGE_DTR_RTS || USE_ESP_WIFI_BRIDGE_BOOT0)
}


// enter ESP flashing, can only be exited by re-powering
void tTxEspWifiBridge::EnterFlash(void)
{
    if (!passthrough) return;

    disp.DrawNotify("FLASH ESP");
    delay_ms(30);

#ifdef USE_ESP_WIFI_BRIDGE_RST_GPIO0
    esp_reset_low();
    esp_gpio0_low();
    delay_ms(10); // delay_ms(100);
    esp_reset_high();
    delay_ms(100); // delay_ms(10) is too short, ESP8285 needs more time
    esp_gpio0_high();
    delay_ms(10); // delay_ms(100);
#endif

    passthrough_do();
}


// enter ESP passthrough, can only be exited by re-powering
void tTxEspWifiBridge::EnterPassthrough(void)
{
    if (!passthrough) return;

    disp.DrawNotify("ESP\nPASSTHRU");
    delay_ms(30);

    passthrough_do();
}


void tTxEspWifiBridge::passthrough_do(void)
{
    leds.InitPassthrough();

    uint32_t baudrate = 115200; // Note: this is what is used for flashing, can be different to ESP_CONFIGURE setting
    ser->SetBaudRate(baudrate);
#if defined DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2 && defined USE_COM_ON_SERIAL
    ser_or_com_set_to_com();
#endif
    ser->flush();
    com->flush();

    while (1) {
        if (doSysTask()) {
            leds.TickPassthrough_ms();
        }

        uint16_t cnt = 0;
        while (com->available() && !ser->full() && (cnt < 64)) { // works fine without cnt, but needs is_full() check
            char c = com->getc();
            ser->putc(c);
            cnt++;
        }
        cnt = 0;
        while (ser->available() && !com->full() && (cnt < 64)) {
            char c = ser->getc();
            com->putc(c);
            cnt++;
        }
    }
}


#ifdef ESP_STARTUP_CONFIGURE

#define ESP_DBG(x)

#define ESP_CMDRES_LEN      46
#define ESP_CMDRES_TMO_MS   70 // 50 was not enough at 9600 baud.  60 worked.


bool tTxEspWifiBridge::esp_read(const char* const cmd, char* const res, uint8_t* const len)
{
    ser->puts(cmd);

ESP_DBG(dbg.puts("\r\n");dbg.puts(cmd);dbg.puts(" -> ");)

    *len = 0;
    uint32_t tnow_ms = millis32();
    char c = 0x00;
    while (*len < ESP_CMDRES_LEN && (millis32() - tnow_ms) < ESP_CMDRES_TMO_MS) {
        if (ser->available()) {
            c = ser->getc();
ESP_DBG(dbg.putc(c);)
            res[(*len)++] = c;
            if (c == '\n') { // 0x0A
ESP_DBG(dbg.puts("!ENDE!");)
                return (*len > 3 && res[0] == 'O' && res[1] == 'K' && res[*len - 2] == '\r'); // 0x0D
            }
        }
    }
    *len = 0;
    return false;
}


void tTxEspWifiBridge::esp_wait_after_read(const char* const res)
{
    if (version >= 10307){ // sends a '*' instead of a '+' if setting had been changed
        if (res[2] == '+') { // no change, so no need to wait for long
            delay_ms(5);
            return;
        }
    }

    // wait for save on esp to finish
    delay_ms(100);
}


void tTxEspWifiBridge::esp_configure_baudrate(void)
{
char s[ESP_CMDRES_LEN+2];
uint8_t len;
char cmd_str[32];

    char baud_str[32];
    u32toBCDstr(ser_baud, baud_str);
    remove_leading_zeros(baud_str);
    strcpy(cmd_str, "AT+BAUD=");
    strcat(cmd_str, baud_str);

    if (!esp_read(cmd_str, s, &len)) { // AT+BAUD sends response with "old" baud rate, when stores it, but does NOT change it
        return;
    }

    esp_wait_after_read(s);
}


void tTxEspWifiBridge::esp_configure_wifiprotocol(void)
{
char s[ESP_CMDRES_LEN+2];
uint8_t len;
char cmd_str[32];

    strcpy(cmd_str, "AT+PROTOCOL=");
    switch (tx_setup->WifiProtocol) {
        case WIFI_PROTOCOL_TCP: strcat(cmd_str, "0"); break;
        case WIFI_PROTOCOL_UDP: strcat(cmd_str, "1"); break;
        case WIFI_PROTOCOL_BT: strcat(cmd_str, "3"); break;
        case WIFI_PROTOCOL_UDPSTA: strcat(cmd_str, "2"); break;
        default:
            strcat(cmd_str, "1"); // should not happen
    }

    if (!esp_read(cmd_str, s, &len)) {
        return;
    }

    esp_wait_after_read(s);
}


void tTxEspWifiBridge::esp_configure_wifichannel(void)
{
char s[ESP_CMDRES_LEN+2];
uint8_t len;
char cmd_str[32];

    strcpy(cmd_str, "AT+WIFICHANNEL=");
    switch (tx_setup->WifiChannel) {
        case WIFI_CHANNEL_1: strcat(cmd_str, "01"); break;
        case WIFI_CHANNEL_6: strcat(cmd_str, "06"); break;
        case WIFI_CHANNEL_11: strcat(cmd_str, "11"); break;
        case WIFI_CHANNEL_13: strcat(cmd_str, "13"); break;
        default:
            strcat(cmd_str, "6"); // should not happen
    }

    if (!esp_read(cmd_str, s, &len)) {
        return;
    }

    esp_wait_after_read(s);
}


void tTxEspWifiBridge::esp_configure_wifipower(void)
{
char s[ESP_CMDRES_LEN+2];
uint8_t len;
char cmd_str[32];

    strcpy(cmd_str, "AT+WIFIPOWER=");
    switch (tx_setup->WifiPower) {
        case WIFI_POWER_LOW: strcat(cmd_str, "0"); break;
        case WIFI_POWER_MED: strcat(cmd_str, "1"); break;
        case WIFI_POWER_MAX: strcat(cmd_str, "2"); break;
        default:
            strcat(cmd_str, "1"); // should not happen
    }

    if (!esp_read(cmd_str, s, &len)) {
        return;
    }

    esp_wait_after_read(s);
}


void tTxEspWifiBridge::esp_configure_bindphrase(void)
{
char s[ESP_CMDRES_LEN+2];
uint8_t len;
char cmd_str[32];

    strcpy(cmd_str, "AT+BINDPHRASE=");
    strcat(cmd_str, common_setup->BindPhrase);

    if (!esp_read(cmd_str, s, &len)) {
        return;
    }

    esp_wait_after_read(s);
}


void tTxEspWifiBridge::run_configure(void)
{
char s[ESP_CMDRES_LEN+2];
uint8_t len;

    if (ser == nullptr) return; // we need a serial

    // needs a delay of e.g 1 ms from the reset, but this should be ensured by calling esp_enable() early
    esp_gpio0_low(); // force AT mode
    delay_ms(500); // not so nice, but it starts up really slowly ...

    bool found = false;

    uint32_t bauds[7] = { ser_baud, 9600, 19200, 38400, 57600, 115200, 230400 };
    uint8_t baud_idx = 0;
    for (uint8_t cc = 0; cc < 3; cc++) { // when in BT it seems to need f-ing long to start up
        for (baud_idx = 0; baud_idx < sizeof(bauds)/4; baud_idx++) {
            ser->SetBaudRate(bauds[baud_idx]);
            delay_ms(5); // allow a few character times to settle
            ser->flush();

            if (esp_read("AT+NAME=?", s, &len)) { // detected !
                s[len-2] = '\0';
                if (!strncmp(s, "OK+NAME=mLRS-Wireless-Bridge", 28)) { // correct name, it's her we are looking for
                    found = true;
                    if (strlen(s) > 32) version = version_from_str(s + 28);
                }
                cc = 128; // break also higher for loop, don't do 255 LOL
                break;
            }
        }
    }

    if (found) {
ESP_DBG(
esp_read("dAT+BAUD=?", s, &len);
esp_read("dAT+PROTOCOL=?", s, &len);
esp_read("dAT+WIFICHANNEL=?", s, &len);
esp_read("dAT+WIFIPOWER=?", s, &len);
esp_read("dAT+BINDPHRASE=?", s, &len);)

        if (bauds[baud_idx] != ser_baud) { // incorrect baud rate
            esp_configure_baudrate();
        }

        esp_configure_wifiprotocol();
        esp_configure_wifichannel();
        esp_configure_wifipower();
        if (version >= 10307) { // not available before v1.3.07
            esp_configure_bindphrase();
        } else {
            // Houston, we have a problem. UDPCl is not available but we allow the user to select
        }
        if (version >= 10307) { // not available before v1.3.07
//            esp_read("AT+WIFIDEVICEID=?", s, &len);
//            if (len > 18) device_id = atoi(s + 16);
            esp_read("AT+WIFIDEVICENAME=?", s, &len);
            if (len > 22) {
                strcpy(info.wireless.device_name, s + 18);
                info.wireless.device_name[strlen(info.wireless.device_name)-1] = '\0'; // strip off '\n'
                info.wireless.device_name[strlen(info.wireless.device_name)-1] = '\0'; // strip off '\r'
            }
            if (strlen(info.wireless.device_name) > 9 && !strncmp(info.wireless.device_name, "mLRS-", 5)) {
                info.wireless.device_id = atoi(info.wireless.device_name + 5);
            }
        }

        if (esp_read("AT+RESTART", s, &len)) { // will respond with 'KO' if a restart isn't needed
            delay_ms(1500); // 500 ms is too short, 1000 ms is sometimes too short, 1200 ms works fine, play it safe
        }
    }

    // if not found or baud rate change, ensure serial has correct baud rate
    if (!found || bauds[baud_idx] != ser_baud) {
        ser->SetBaudRate(ser_baud);
    }
    ser->flush();

//ESP_DBG(if (esp_read("AT+NAME=?", s, &len)) { dbg.puts("!ALL GOOD!\r\n"); } else { dbg.puts("!F IT!\r\n"); })
if (esp_read("AT+NAME=?", s, &len)) { dbg.puts("!ALL GOOD!\r\n"); } else { dbg.puts("!F IT!\r\n"); }

    esp_gpio0_high(); // leave forced AT mode
}


#endif // ESP_AUTOCONFIGURE

#endif // USE_ESP_WIFI_BRIDGE

#endif // TX_ESP_H



