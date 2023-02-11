//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// CLI Interface Header
//********************************************************
#ifndef TX_CLI_H
#define TX_CLI_H
#pragma once


#include <stdlib.h>
#include <ctype.h>
#include "setup_tx.h"
extern TxStatsBase txstats;


typedef enum {
    PARAM_FORMAT_DEFAULT = 0,
    PARAM_FORMAT_CLI
} PARAM_FORMAT_ENUM;


typedef enum {
    CLI_TASK_NONE = 0,
    CLI_TASK_RX_PARAM_SET,
    CLI_TASK_PARAM_STORE,
    CLI_TASK_BIND,
    CLI_TASK_PARAM_RELOAD,
} CLI_TASK_ENUM;


typedef enum {
    CLI_STATE_NORMAL = 0,
    CLI_STATE_STATS,
} CLI_STATE_ENUM;


class tTxCli
{
  public:
    void Init(tSerialBase* _com = nullptr);
    void Set(uint8_t new_line_end = CLI_LINE_END_CR);
    void Do(void);
    uint8_t Task(void);

    void addc(uint8_t c);
    void clear(void);
    void print_help(void);
    void print_param(uint8_t idx);
    void print_param_list(uint8_t flag);
    void print_param_opt_list(uint8_t idx);
    void stream(void);

    bool cmd_param_set(char* name, char* svalue);
    bool cmd_param_opt(char* name);

    void puts(const char* s) { com->puts(s); }
    void putsn(const char* s) { com->puts(s); com->puts(ret); }

    tSerialBase* com;

    uint8_t line_end;
    char ret[4];

    char buf[128];
    uint8_t pos;
    uint32_t t_last_ms;

    uint8_t task_pending;

    uint8_t state;
};


void tTxCli::Init(tSerialBase* _com)
{
    com = _com;

    line_end = CLI_LINE_END_CR;
    strcpy(ret, "\r");

    pos = 0;
    buf[pos] = '\0';
    t_last_ms = 0;

    task_pending = CLI_TASK_NONE;

    state = CLI_STATE_NORMAL;
}


void tTxCli::Set(uint8_t new_line_end)
{
    if (new_line_end == line_end) return;

    line_end = new_line_end;

    switch (line_end) {
    case CLI_LINE_END_CR: strcpy(ret, "\r"); break;
    case CLI_LINE_END_LF: strcpy(ret, "\n"); break;
    case CLI_LINE_END_CRLF: strcpy(ret, "\r\n"); break;
    }
}


uint8_t tTxCli::Task(void)
{
    uint8_t task = task_pending;
    task_pending = 0;
    return task;
}


void tTxCli::addc(uint8_t c)
{
    if (pos > 100) return;

    buf[pos] = c;
    pos++;
    buf[pos] = '\0';
}


void tTxCli::clear(void)
{
    pos = 0;
    buf[pos] = '\0';
}


// p name = value or p name
bool tTxCli::cmd_param_set(char* name, char* svalue)
{
char s[64];
uint8_t sep, n;

    if (buf[0] != 'p') return false;
    if (buf[1] != ' ') return false;
    if (!isalpha(buf[2])) return false;

    // cleanify: remove p, remove blanks, find '='
    n = 0;
    sep = 0;
    for (uint8_t i = 2; i < strlen(buf); i++) {
        if (buf[i] == '=') sep = n;
        if (buf[i] != ' ') s[n++] = buf[i];
    }
    s[n] = '\0';

   if (sep == 0) {
       n = 0;
       for (uint8_t i = 0; i < strlen(s); i++) name[n++] = toupper(s[i]);
       name[n] = '\0';
       strcpy(svalue, "?");
   } else {
       n = 0;
       for (uint8_t i = 0; i < sep; i++) name[n++] = toupper(s[i]);
       name[n] = '\0';
       n = 0;
       for (uint8_t i = sep + 1; i < strlen(s); i++) svalue[n++] = s[i]; // we must not modify it, e.g. bind phrase
       svalue[n] = '\0';
   }

   return true;
}


// "50 Hz,31 Hz,19 Hz" etc
bool param_get_optstr(char* s, uint8_t param_idx, uint8_t value, uint8_t format)
{
int8_t seps[24];
uint8_t nr, n;

    const char* optstr = SetupParameter[param_idx].optstr;

    if (format == PARAM_FORMAT_CLI) { // we do some cli specific faking
         if (param_idx == 2) { // RF Mode
             optstr = SETUP_OPT_RF_BAND_LONGSTR;
         }
    }

    seps[0] = -1;
    nr = 1;
    for (n = 0; n < strlen(optstr); n++) {
        if (optstr[n] == ',') { seps[nr] = n; nr++; }
    }
    seps[nr] = n;

    // we have now: -1, 5, 11, 17, and nr = 3

    if (value >= nr) { s[0] = '\0'; return false; }

    n = 0;
    for (uint8_t i = seps[value] + 1; i < seps[value + 1]; i++) s[n++] = optstr[i];
    s[n] = '\0';

    return true;
}


bool param_get_setting_str(char* s, uint8_t param_idx, uint8_t format = PARAM_FORMAT_DEFAULT)
{
    switch (SetupParameter[param_idx].type) {
    case SETUP_PARAM_TYPE_UINT8:
        break;
    case SETUP_PARAM_TYPE_INT8:{
        int8_t i8 = *(int8_t*)(SetupParameter[param_idx].ptr);
        stoBCDstr(i8, s);
        if (SetupParameter[param_idx].unit[0] != '\0') {
          strcat(s, " ");
          strcat(s, SetupParameter[param_idx].unit);
        }
        return true;
        }break;
    case SETUP_PARAM_TYPE_UINT16:
        break;
    case SETUP_PARAM_TYPE_INT16:
        break;
    case SETUP_PARAM_TYPE_LIST:{
        uint8_t u8 = *(uint8_t*)(SetupParameter[param_idx].ptr);
        return param_get_optstr(s, param_idx, u8, format);
        }break;
    case SETUP_PARAM_TYPE_STR6:
        strstrbufcpy(s, (char*)SetupParameter[param_idx].ptr, 6);
        return true;
    }
    s[0] = '\0';
    return false;
}


uint8_t param_get_opt_num(uint8_t param_idx)
{
    const char* optstr = SetupParameter[param_idx].optstr;

    uint8_t nr = 0;
    for (uint8_t n = 0; n < strlen(optstr); n++) {
        if (optstr[n] == ',') nr++;
    }
    return nr + 1;
}


uint16_t param_get_allowed_mask(uint8_t param_idx)
{
    return (SetupParameter[param_idx].allowed_mask_ptr) ? *(SetupParameter[param_idx].allowed_mask_ptr) : UINT16_MAX;
}


uint8_t param_get_allowed_opt_num(uint8_t param_idx)
{
    uint16_t allowed_mask = param_get_allowed_mask(param_idx);

    uint8_t nr = 0;
    for (uint8_t i = 0; i < param_get_opt_num(param_idx); i++) {
        if (allowed_mask & (1 << i)) nr++;
    }
    return nr;
}


bool param_get_idx(uint8_t* param_idx, char* name)
{
char s[64];

    for (uint8_t idx = 0; idx < SETUP_PARAMETER_NUM; idx++) {
        uint8_t n = 0;
        for (uint8_t i = 0; i < strlen(SetupParameter[idx].name); i++) {
          s[n] = toupper(SetupParameter[idx].name[i]);
          if (s[n] == ' ') s[n] = '_';
          n++;
        }
        s[n] = '\0';

        if (strcmp(s, name) == 0) { *param_idx = idx; return true; }
    }

    *param_idx = 0;
    return false;
}


bool param_set_val(bool* rx_param_changed, char* svalue, uint8_t idx)
{
    *rx_param_changed = false;

    switch (SetupParameter[idx].type) {
    case SETUP_PARAM_TYPE_UINT8:
        break;
    case SETUP_PARAM_TYPE_INT8:{
        int32_t v = atoi(svalue);
        int8_t i8 = SetupParameter[idx].min.INT8_value;
        if (v < i8) return false;
        i8 = SetupParameter[idx].max.INT8_value;
        if (v > i8) return false;
        // set
        tParamValue vv;
        vv.i8 = v;
        *rx_param_changed = setup_set_param(idx, vv);
        return true;
        }break;
    case SETUP_PARAM_TYPE_UINT16:
        break;
    case SETUP_PARAM_TYPE_INT16:
        break;
    case SETUP_PARAM_TYPE_LIST:{
        int32_t v = atoi(svalue);
        if (v < 0) return false;
        if (v >= param_get_opt_num(idx)) return false;
        if ((param_get_allowed_mask(idx) & (1 << v)) == 0) return false;
        // set
        tParamValue vv;
        vv.u8 = v;
        *rx_param_changed = setup_set_param(idx, vv);
        return true;
        }break;
    case SETUP_PARAM_TYPE_STR6:
        if (strlen(svalue) != 6) return false;
        for (uint8_t i = 0; i < 6; i++) {
            if (!is_valid_bindphrase_char(svalue[i])) return false;
        }
        // set
        *rx_param_changed = setup_set_param_str6(idx, svalue);
        return true;
        break;
    }

    return false;
}



void tTxCli::print_param_opt_list(uint8_t idx)
{
char s[16];

    switch (SetupParameter[idx].type) {
    case SETUP_PARAM_TYPE_UINT8:
        break;
    case SETUP_PARAM_TYPE_INT8:{
        int8_t i8 = SetupParameter[idx].min.INT8_value;
        puts("  min: ");
        stoBCDstr(i8, s); putsn(s);
        i8 = SetupParameter[idx].max.INT8_value;
        puts("  max: ");
        stoBCDstr(i8, s); putsn(s);
        }break;
    case SETUP_PARAM_TYPE_UINT16:
        break;
    case SETUP_PARAM_TYPE_INT16:
        break;
    case SETUP_PARAM_TYPE_LIST:{
        uint16_t i = 0;
        uint16_t allowed_mask = param_get_allowed_mask(idx);
        while (param_get_optstr(s, idx, i, PARAM_FORMAT_CLI)) {
            if (allowed_mask & (1 << i)) {
                com->puts("  "); com->putc(i + '0'); com->puts(" = "); putsn(s);
            }
            i++;
        }
        }break;
    case SETUP_PARAM_TYPE_STR6:
        putsn("  [a-z0-9#-._]");
        break;
    }
}


void tTxCli::print_param(uint8_t idx)
{
    uint8_t allowed_nr = param_get_allowed_opt_num(idx);

    puts("  ");
    puts(SetupParameter[idx].name);
    puts(" = ");
    if (allowed_nr == 0) {
      puts("-"); // this parameter is not available on this device
      puts(ret);
      return;
    }
    char s[32];
    param_get_setting_str(s, idx, PARAM_FORMAT_CLI);
    puts(s);
    switch (SetupParameter[idx].type) {
    case SETUP_PARAM_TYPE_UINT8:
        break;
    case SETUP_PARAM_TYPE_INT8:
        break;
    case SETUP_PARAM_TYPE_UINT16:
        break;
    case SETUP_PARAM_TYPE_INT16:
        break;
    case SETUP_PARAM_TYPE_LIST:{
        uint8_t u8 = *(uint8_t*)(SetupParameter[idx].ptr);
        puts(" ["); com->putc(u8 + '0'); puts("]");
        if (allowed_nr == 1) puts("(unchangeable)"); // unmodifiable unalterable immutable unchangeable
        }break;
    case SETUP_PARAM_TYPE_STR6:
        break;
    }
    puts(ret);
}


void tTxCli::print_param_list(uint8_t flag)
{
    if ((flag == 0 || flag == 1 || flag == 3) && !connected()) {
        putsn("warn: receiver not connected");
    }

    for (uint8_t idx = 0; idx < SETUP_PARAMETER_NUM; idx++) {
        if ((flag == 1) && (setup_param_is_tx(idx) || setup_param_is_rx(idx))) continue;
        if ((flag == 2) && !setup_param_is_tx(idx)) continue;
        if ((flag == 3) && !setup_param_is_rx(idx)) continue;

        if ((flag == 0 || flag == 3) && !connected() && setup_param_is_rx(idx)) continue;

        print_param(idx);
    }
}


void tTxCli::stream(void)
{
    uint32_t t_now = millis32();

    if (state == CLI_STATE_STATS) {
        if (t_now - t_last_ms >= 500) {
            t_last_ms = t_now;

            puts(u8toBCD_s(txstats.GetLQ()));
            puts("(");
            puts(u8toBCD_s(stats.valid_frames_received.GetLQ()));
            puts("),");
            puts(u8toBCD_s(stats.received_LQ));
            puts(", ");

            puts(s8toBCD_s(stats.last_rx_rssi1));
            puts(",");
            puts(s8toBCD_s(stats.received_rssi));
            puts(", ");
            puts(s8toBCD_s(stats.last_rx_snr1));
            puts("; ");

            puts(u16toBCD_s(stats.bytes_transmitted.GetBytesPerSec()));
            puts(", ");
            puts(u16toBCD_s(stats.bytes_received.GetBytesPerSec()));
            putsn(";");
        }
    }
}


void tTxCli::print_help(void)
{
    putsn("  help, h, ?  -> this help page");
    putsn("  pl          -> list all parameters");
    putsn("  pl c        -> list common parameters");
    putsn("  pl tx       -> list Tx parameters");
    putsn("  pl rx       -> list Rx parameters");
    putsn("  p name          -> get parameter value");
    putsn("  p name = value  -> set parameter value");
    putsn("  p name = ?      -> get parameter value and list of allowed values");
    putsn("  pstore      -> store parameters");
    putsn("  bind        -> start binding");
    putsn("  reload      -> reload all parameter settings");
    putsn("  stats       -> starts streaming statistics");

    putsn("  ptser       -> enter serial passthrough");
#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE
    putsn("  espboot     -> reboot ESP and enter serial passthrough");
    putsn("  espcli      -> GPIO0 = low and enter serial passthrough");
#endif
}


// TODO: should really be abstracted out, but for the moment let's be happy to have it here
void passthrough_do(tSerialBase* com, tSerialBase* com2)
{
uint16_t led_blink = 0;

    if (!com) return;
    if (!com2) return;

    LED_RED_OFF;
    LED_GREEN_ON;

    while (1) {
        if (doSysTask) {
            doSysTask = 0;
            DECc(led_blink, SYSTICK_DELAY_MS(100));
            if (!led_blink) { LED_GREEN_TOGGLE; LED_RED_TOGGLE; }
        }

        if (com->available()) {
            char c = com->getc();
            com2->putc(c);
        }
        if (com2->available()) {
            char c = com2->getc();
            com->putc(c);
        }
    }
}


void tTxCli::Do(void)
{
char sname[32], svalue[32];
uint8_t param_idx;
bool rx_param_changed;

    if (!com) return;

    //com->puts(".");

    uint32_t t_now = millis32();
    if (pos && (t_now - t_last_ms > 2000)) { putsn(">"); putsn("  timeout"); clear(); }

    if (state != CLI_STATE_NORMAL) {
      if (com->available()) { com->getc(); state = CLI_STATE_NORMAL; putsn("  streaming stats stopped"); return; }
      stream();
    }

    while (com->available()) {
      char c = com->getc();
      t_last_ms = t_now;

      if (c != '\n' && c != '\r' && c != ',' && c != ';') {
        com->putc(c);
        addc(c);
        continue;
      }
      putsn(">");

      //-- basic commands
      if (strcmp(buf, "h") == 0)     { print_help(); } else
      if (strcmp(buf, "help") == 0)  { print_help(); } else
      if (strcmp(buf, "?") == 0)     { print_help(); } else
      if (strcmp(buf, "pl") == 0)    { print_param_list(0); } else
      if (strcmp(buf, "pl c") == 0)  { print_param_list(1); } else
      if (strcmp(buf, "pl tx") == 0) { print_param_list(2); } else
      if (strcmp(buf, "pl rx") == 0) { print_param_list(3);

      } else
      if (cmd_param_set(sname, svalue)) { // p name, p name = value
          if (!param_get_idx(&param_idx, sname)) {
              putsn("err: invalid parameter name");
          } else if (!connected() && setup_param_is_rx(param_idx)) {
              putsn("warn: receiver not connected");
          } else if (svalue[0] == '?') {
              print_param(param_idx);
              print_param_opt_list(param_idx);
          } else if (svalue[0] == '\0') {
              putsn("err: no value specified");
          } else if (!param_set_val(&rx_param_changed, svalue, param_idx)) {
              putsn("err: invalid value");
          } else {
              print_param(param_idx);
              if (rx_param_changed) task_pending = CLI_TASK_RX_PARAM_SET;
          }

      } else
      if (strcmp(buf, "pstore") == 0) {
          task_pending = CLI_TASK_PARAM_STORE;
          if (!connected()) {
              putsn("warn: receiver not connected");
              putsn("  Tx parameters stored");
          } else {
              putsn("  parameters stored");
          }

      } else
      if (strcmp(buf, "bind") == 0) {
          task_pending = CLI_TASK_BIND;
          putsn("  Tx entered bind mode");

      } else
      if (strcmp(buf, "reload") == 0) {
          task_pending = CLI_TASK_PARAM_RELOAD;
          if (!connected()) {
              putsn("warn: receiver not connected");
              putsn("  Tx parameters reloaded");
          } else {
              putsn("  parameters reloaded");
          }

      } else
      if (strcmp(buf, "stats") == 0) {
          state = CLI_STATE_STATS;
          putsn("  starts streaming stats");
          putsn("  send any character to stop");

      //-- ESP handling
      } else
      if (strcmp(buf, "ptser") == 0) {
          // enter passthrough to serial, can only be exited by re-powering
          serial.SetBaudRate(115200);
          passthrough_do(com, &serial);
#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE
      } else
      if (strcmp(buf, "espboot") == 0) {
          esp_gpio0_low();
          esp_reset_low();
          delay_ms(100);
          esp_reset_high();
          delay_ms(100);
          esp_gpio0_high();
          delay_ms(100);
          serial.SetBaudRate(115200);
          passthrough_do(com, &serial);
      } else
      if (strcmp(buf, "espcli") == 0) {
          // enter esp cli, can only be exited by re-powering
          esp_gpio0_low();
          delay_ms(100);
          serial.SetBaudRate(115200);
          passthrough_do(com, &serial);
#endif

      //-- invalid command
      } else {
          putsn((!pos) ? "  empty cmd" : "  invalid cmd");
      }

      clear();
    }
}


#endif // TX_CLI_H



