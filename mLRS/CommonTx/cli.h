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


typedef enum {
    CLI_TASK_NONE = 0,
    CLI_TASK_RX_PARAM_SET,
    CLI_TASK_PARAM_STORE,
    CLI_TASK_BIND,
    CLI_TASK_RX_RELOAD,
} CLI_TASK_ENUM;


class tTxCli
{
  public:
    void Init(tSerialBase* _com = nullptr);
    void Do(void);
    uint8_t Task(void);

    void addc(uint8_t c);
    void clear(void);
    void print_help(void);
    void print_param(uint8_t idx);
    void print_param_list(uint8_t flag);
    void print_param_opt_list(uint8_t idx);

    bool cmd_param_set(char* name, char* svalue);
    bool cmd_param_opt(char* name);

    tSerialBase* com;
    char buf[128];
    uint8_t pos;
    uint32_t t_last_ms;

    uint8_t task_pending;
};


void tTxCli::Init(tSerialBase* _com)
{
    com = _com;
    pos = 0;
    buf[pos] = '\0';
    task_pending = CLI_TASK_NONE;

    t_last_ms = 0;
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
       for (uint8_t i = sep + 1; i < strlen(s); i++) svalue[n++] = toupper(s[i]);
       svalue[n] = '\0';
   }

   return true;
}


// po name
bool tTxCli::cmd_param_opt(char* name)
{
uint8_t n;

    if (buf[0] != 'p') return false;
    if (buf[1] != 'o') return false;
    if (buf[2] != ' ') return false;

    // cleanify: remove p, remove blanks
    n = 0;
    for (uint8_t i = 2; i < strlen(buf); i++) {
        if (buf[i] != ' ') name[n++] = toupper(buf[i]);
    }
    name[n] = '\0';

    return true;
}


// "50 Hz,31 Hz,19 Hz"
bool param_get_optstr(char* s, uint8_t param_idx, uint8_t value)
{
int8_t seps[24];
uint8_t nr, n;

    const char* optstr = SetupParameter[param_idx].optstr;

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
            if (!((svalue[i] >= 'a' && svalue[i] <= 'z') ||
                  (svalue[i] >= '0' && svalue[i] <= '9' ) ||
                  (svalue[i] == '_') || (svalue[i] == '#') ||
                  (svalue[i] == '-') || (svalue[i] == '.')   )) return false;
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
        com->puts("  min: ");
        if (i8 >= 0) { utoBCDstr(i8, s); com->puts(s); } else { utoBCDstr(-i8, s); com->putc('-'); com->puts(s); }
        com->puts("\n");
        i8 = SetupParameter[idx].max.INT8_value;
        com->puts("  max: ");
        if (i8 >= 0) { utoBCDstr(i8, s); com->puts(s); } else { utoBCDstr(-i8, s); com->putc('-'); com->puts(s); }
        com->puts("\n");
        }break;
    case SETUP_PARAM_TYPE_UINT16:
        break;
    case SETUP_PARAM_TYPE_INT16:
        break;
    case SETUP_PARAM_TYPE_LIST:{
        uint16_t i = 0;
        uint16_t allowed_mask = param_get_allowed_mask(idx);
        while (param_get_optstr(s, idx, i)) {
            if (allowed_mask & (1 << i)) {
                com->puts("  "); com->putc(i + '0'); com->puts(" = "); com->puts(s);
                com->puts("\n");
            }
            i++;
        }
        }break;
    case SETUP_PARAM_TYPE_STR6:
        com->puts("  [a-zA-Z0-9#-._]\n");
        break;
    }
}


void tTxCli::print_param(uint8_t idx)
{
    com->puts("  ");
    com->puts(SetupParameter[idx].name);
    com->puts(" = ");
    char s[16];
    switch (SetupParameter[idx].type) {
    case SETUP_PARAM_TYPE_UINT8:
        break;
    case SETUP_PARAM_TYPE_INT8:{
        int8_t i8 = *(int8_t*)(SetupParameter[idx].ptr);
        if (i8 >= 0) { utoBCDstr(i8, s); com->puts(s); } else { utoBCDstr(-i8, s); com->putc('-'); com->puts(s); }
        com->puts(" ");
        com->puts(SetupParameter[idx].unit);
        }break;
    case SETUP_PARAM_TYPE_UINT16:
        break;
    case SETUP_PARAM_TYPE_INT16:
        break;
    case SETUP_PARAM_TYPE_LIST:{
        uint8_t u8 = *(uint8_t*)(SetupParameter[idx].ptr);
        param_get_optstr(s, idx, u8);
        com->puts(s);
        com->puts(" ["); com->putc(u8 + '0'); com->puts("]");
        }break;
    case SETUP_PARAM_TYPE_STR6:
        strstrbufcpy(s, (char*)(SetupParameter[idx].ptr), 6);
        com->puts(s);
        break;
    }
    com->puts("\n");
}


void tTxCli::print_param_list(uint8_t flag)
{
    if ((flag == 0 || flag == 1 || flag == 3) && !connected()) {
        com->puts("warn: receiver not connected\n");
    }

    for (uint8_t idx = 0; idx < SETUP_PARAMETER_NUM; idx++) {
        if ((flag == 1) && (SetupParameter[idx].name[0] == 'T' || SetupParameter[idx].name[0] == 'R')) continue;
        if ((flag == 2) && (SetupParameter[idx].name[0] != 'T')) continue;
        if ((flag == 3) && (SetupParameter[idx].name[0] != 'R')) continue;

        if ((flag == 0 || flag == 3) && !connected() && (SetupParameter[idx].name[0] == 'R')) continue;

        print_param(idx);
    }
}


void tTxCli::print_help(void)
{
    com->puts("  help, h, ?  -> this help page\n");
    com->puts("  pl          -> list all parameters\n");
    com->puts("  pl c        -> list common parameters\n");
    com->puts("  pl tx       -> list Tx parameters\n");
    com->puts("  pl rx       -> list Rx parameters\n");
    com->puts("  po name     -> list of allowed parameter values\n");
    com->puts("  p name          -> get parameter value\n");
    com->puts("  p name = value  -> set parameter value\n");
    com->puts("  p name = ?      -> get parameter value and list of allowed values\n");
    com->puts("  pstore      -> store parameters\n");
}


void tTxCli::Do(void)
{
char sname[32], svalue[32];
uint8_t param_idx;
bool rx_param_changed;

    if (!com) return;

    //com->puts(".");

    while (com->available()) {
      char c = com->getc();

      uint32_t t_now = millis32();
      if (t_now - t_last_ms > 2000) clear();
      t_last_ms = t_now;

      if (c != '\n' && c != ',' && c != ';') {
        com->putc(c);
        addc(c);
        continue;
      }
      com->puts(">\n");

      if (strcmp(buf, "h") == 0)     print_help();
      if (strcmp(buf, "help") == 0)  print_help();
      if (strcmp(buf, "?") == 0)     print_help();
      if (strcmp(buf, "pl") == 0)    print_param_list(0);
      if (strcmp(buf, "pl c") == 0)  print_param_list(1);
      if (strcmp(buf, "pl tx") == 0) print_param_list(2);
      if (strcmp(buf, "pl rx") == 0) print_param_list(3);

      if (cmd_param_opt(sname)) { // po name
        if (!param_get_idx(&param_idx, sname)) {
            com->puts("err: invalid parameter name\n");
        } else {
            com->puts("  ");
            com->puts(SetupParameter[param_idx].name);
            com->puts(" options:\n");
            print_param_opt_list(param_idx);
        }
      }

      if (cmd_param_set(sname, svalue)) { // p name,  p name = value
          if (!param_get_idx(&param_idx, sname)) {
              com->puts("err: invalid parameter name\n");
          } else if (!connected() && sname[0] == 'R'){
              com->puts("warn: receiver not connected\n");
          } else if (svalue[0] == '?') {
              print_param(param_idx);
              print_param_opt_list(param_idx);
          } else if (svalue[0] == '\0') {
              com->puts("err: no value specified\n");
          } else if (!param_set_val(&rx_param_changed, svalue, param_idx)) {
              com->puts("err: invalid value\n");
          } else {
              print_param(param_idx);
              if (rx_param_changed) task_pending = CLI_TASK_RX_PARAM_SET;
          }
      }

      if (strcmp(buf, "pstore") == 0) {
          task_pending = CLI_TASK_PARAM_STORE;
          if (!connected()) {
              com->puts("warn: receiver not connected\n");
              com->puts("  Tx parameters stored\n");
          } else {
              com->puts("  parameters stored\n");
          }
      }

      if (strcmp(buf, "bind") == 0) {
          task_pending = CLI_TASK_BIND;
          com->puts("  Tx entered bind mode\n");
      }

      if (strcmp(buf, "reload") == 0) {
          if (!connected()) {
              com->puts("warn: receiver not connected\n");
          } else {
              task_pending = CLI_TASK_RX_RELOAD;
              com->puts("  Rx setupdata reloaded\n");
          }
      }

      clear();
    }
}


#endif // TX_CLI_H



