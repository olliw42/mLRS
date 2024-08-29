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
#include <string.h>
#include "setup_tx.h"


extern volatile uint32_t millis32(void);
extern bool connected(void);
extern tStats stats;
extern tConfigId config_id;


//-------------------------------------------------------
// param helper routines
//-------------------------------------------------------

typedef enum {
    PARAM_FORMAT_DEFAULT = 0,
    PARAM_FORMAT_CLI,
    PARAM_FORMAT_DISPLAY,
} PARAM_FORMAT_ENUM;


// helper, extracts value as string from optstr for parameters of type LIST
// "50 Hz,31 Hz,19 Hz" etc
bool _param_get_listval_fromoptstr(char* const s, uint8_t param_idx, uint8_t value, uint8_t format)
{
int8_t seps[24];
uint8_t nr, n;

    const char* optstr = SetupParameter[param_idx].optstr;

    if (format == PARAM_FORMAT_CLI) {
         if (param_idx == PARAM_INDEX_RF_BAND) { // RF Band
             optstr = SETUP_OPT_RF_BAND_LONGSTR;
         }
         if ((SetupParameter[param_idx].ptr == &Setup.Tx[0].Diversity) ||
             (SetupParameter[param_idx].ptr == &Setup.Rx.Diversity)) {
             optstr = SETUP_OPT_DIVERSITY_LONGSTR;
         }
    } else
    if (format == PARAM_FORMAT_DISPLAY) {
        if (param_idx == PARAM_INDEX_RF_BAND) { // RF Band
            optstr = SETUP_OPT_RF_BAND_DISPSTR;
        }
        if ((SetupParameter[param_idx].ptr == &Setup.Tx[0].Diversity) ||
            (SetupParameter[param_idx].ptr == &Setup.Rx.Diversity)) {
            optstr = SETUP_OPT_DIVERSITY_DISPSTR;
        }
        if (SetupParameter[param_idx].ptr == &Setup.Rx.SerialLinkMode) {
            optstr = SETUP_OPT_SERIAL_LINK_MODE_DISPLAYSTR;
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


// helper, returns allowed mask for LIST
uint16_t param_get_allowed_mask(uint8_t param_idx)
{
    return (SetupParameter[param_idx].allowed_mask_ptr) ? *(SetupParameter[param_idx].allowed_mask_ptr) : UINT16_MAX;
}


// helper, determines number of options in optstr (= maximal number of options)
uint8_t param_get_opt_num(uint8_t param_idx)
{
    const char* optstr = SetupParameter[param_idx].optstr;

    uint8_t num = 0;
    for (uint8_t n = 0; n < strlen(optstr); n++) {
        if (optstr[n] == ',') num++;
    }
    return num + 1;
}


// helper, determines number of allowed options in optstr
uint8_t param_get_allowed_opt_num(uint8_t param_idx)
{
    if (SetupParameter[param_idx].type != SETUP_PARAM_TYPE_LIST) return UINT8_MAX;

    uint16_t allowed_mask = param_get_allowed_mask(param_idx);

    uint8_t num = 0;
    for (uint8_t i = 0; i < param_get_opt_num(param_idx); i++) {
        if (allowed_mask & (1 << i)) num++;
    }
    return num;
}


// helper, finds index from name
bool param_get_idx(uint8_t* const param_idx, char* const name)
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


// helper, gets parameter value as formatted string, different formats can be specified
bool param_get_val_formattedstr(char* const s, uint8_t param_idx, uint8_t format = PARAM_FORMAT_DEFAULT)
{
    switch (SetupParameter[param_idx].type) {
    case SETUP_PARAM_TYPE_UINT8:
        break;
    case SETUP_PARAM_TYPE_INT8:{
        int8_t i8 = *(int8_t*)(SetupParameterPtr(param_idx));
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
        uint8_t u8 = *(uint8_t*)(SetupParameterPtr(param_idx));
        return _param_get_listval_fromoptstr(s, param_idx, u8, format);
        }break;
    case SETUP_PARAM_TYPE_STR6:
        strstrbufcpy(s, (char*)SetupParameterPtr(param_idx), 6);
        return true;
    }
    s[0] = '\0';
    return false;
}


// helper, sets parameter value for integer-valued parameters
bool param_set_val_fromint(bool* const rx_param_changed, int32_t value, uint8_t param_idx)
{
    *rx_param_changed = false;

    switch (SetupParameter[param_idx].type) {
    case SETUP_PARAM_TYPE_UINT8:
        break;
    case SETUP_PARAM_TYPE_INT8:{
        // check
        int32_t i8 = SetupParameter[param_idx].min.INT8_value;
        if (value < i8) return false;
        i8 = SetupParameter[param_idx].max.INT8_value;
        if (value > i8) return false;
        // set
        tParamValue vv;
        vv.i8 = value;
        *rx_param_changed = setup_set_param(param_idx, vv);
        return true;
        }break;
    case SETUP_PARAM_TYPE_UINT16:
        break;
    case SETUP_PARAM_TYPE_INT16:
        break;
    case SETUP_PARAM_TYPE_LIST:{
        // check
        if (value < 0) return false;
        if (value >= param_get_opt_num(param_idx)) return false;
        if ((param_get_allowed_mask(param_idx) & (1 << value)) == 0) return false;
        // set
        tParamValue vv;
        vv.u8 = value;
        *rx_param_changed = setup_set_param(param_idx, vv);
        return true;
        }break;
    case SETUP_PARAM_TYPE_STR6:
        // not an integer-valued parameter, so return false
        break;
    }

    return false;
}


// helper, sets parameter value from an input string
bool param_set_str6val(bool* const rx_param_changed, char* const svalue, uint8_t param_idx)
{
    *rx_param_changed = false;

    switch (SetupParameter[param_idx].type) {
    case SETUP_PARAM_TYPE_UINT8:
    case SETUP_PARAM_TYPE_INT8:
    case SETUP_PARAM_TYPE_UINT16:
    case SETUP_PARAM_TYPE_INT16:
    case SETUP_PARAM_TYPE_LIST:
        // not a str6-valued parameter, so return false
        break;
    case SETUP_PARAM_TYPE_STR6:
        // check
        if (strlen(svalue) != 6) return false;
        for (uint8_t i = 0; i < 6; i++) {
            if (!is_valid_bindphrase_char(svalue[i])) return false;
        }
        // set
        *rx_param_changed = setup_set_param_str6(param_idx, svalue);
        return true;
        break;
    }

    return false;
}


// helper, sets parameter value from an input string
bool param_set_val_fromstr(bool* const rx_param_changed, char* const svalue, uint8_t param_idx)
{
    *rx_param_changed = false;

    switch (SetupParameter[param_idx].type) {
    case SETUP_PARAM_TYPE_UINT8:
    case SETUP_PARAM_TYPE_INT8:
    case SETUP_PARAM_TYPE_UINT16:
    case SETUP_PARAM_TYPE_INT16:
    case SETUP_PARAM_TYPE_LIST:{
        int32_t value = atoi(svalue);
        return param_set_val_fromint(rx_param_changed, value, param_idx);
        }break;
    case SETUP_PARAM_TYPE_STR6:
        return param_set_str6val(rx_param_changed, svalue, param_idx);
    }

    return false;
}


//-------------------------------------------------------
// CLI class
//-------------------------------------------------------

class tTxCli
{
  public:
    void Init(tSerialBase* const _comport);
    void Set(uint8_t new_line_end);
    void Do(void);
    uint8_t Task(void);
    int32_t GetTaskValue(void) { return task_value; }

  private:
    typedef enum {
        CLI_STATE_NORMAL = 0,
        CLI_STATE_STATS,
    } CLI_STATE_ENUM;

    void addc(uint8_t c);
    void clear(void);
    void print_help(void);
    void print_param(uint8_t idx);
    void print_param_list(uint8_t flag);
    void print_param_opt_list(uint8_t idx);
    void print_device_version(void);
    void print_frequencies(void);
    void stream(void);

    bool is_cmd(const char* const cmd);
    bool is_cmd_param_set(char* const name, char* const svalue);
    bool is_cmd_set_value(const char* const cmd, int32_t* const value);

    uint16_t put_cnt;
    void delay_off(void) { put_cnt = 0; }
    void delay_clear(void) { put_cnt = 1; }
//    void delay(void) { if (put_cnt > 768) { delay_ms(40); put_cnt -= 512; } } // 115200 -> 512 bytes = 44 ms
    void delay(void) { if (put_cnt > 256) { delay_ms(15); put_cnt -= 128; } } // 115200 -> 128 bytes = 11 ms, usb txbuf is small

    void putc(char c) { com->putc(c); if (put_cnt) put_cnt++; delay(); }
    void puts(const char* s) { com->puts(s); if (put_cnt) put_cnt += strlen(s); delay(); }
    void putsn(const char* s) { com->puts(s); com->puts(ret); if (put_cnt) put_cnt += strlen(s)+strlen(ret); delay(); }

    void print_config_id(void);

    tSerialBase* com;

    bool initialized;

    uint8_t line_end;
    char ret[4];

    char buf[128];
    uint8_t pos;
    uint32_t tlast_ms;

    uint8_t task_pending;
    int32_t task_value;

    uint8_t state;
};


void tTxCli::Init(tSerialBase* const _comport)
{
    com = _comport;

    initialized = (com != nullptr) ? true : false;

    line_end = CLI_LINE_END_CRLF;
    strcpy(ret, "\r\n");

    pos = 0;
    buf[pos] = '\0';
    tlast_ms = 0;

    task_pending = TX_TASK_NONE;
    task_value = 0;

    state = CLI_STATE_NORMAL;

    put_cnt = 0;
}


void tTxCli::Set(uint8_t new_line_end)
{
    if (new_line_end == line_end) return;

    line_end = new_line_end;

    switch (line_end) {
    case CLI_LINE_END_CRLF: strcpy(ret, "\r\n"); break;
    case CLI_LINE_END_LF: strcpy(ret, "\n"); break;
    case CLI_LINE_END_CR: strcpy(ret, "\r"); break;
    }
}


uint8_t tTxCli::Task(void)
{
    uint8_t task = task_pending;
    task_pending = TX_TASK_NONE;
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


// cmd;
bool tTxCli::is_cmd(const char* const cmd)
{
    return (strcmp(buf, cmd) == 0) ? true : false;
}


// name = value or p name
bool tTxCli::is_cmd_set_value(const char* const cmd, int32_t* const value)
{
char s[64];
uint8_t n;

    uint8_t cmd_len = strlen(cmd);
    uint8_t buf_len = strlen(buf);
    if (buf_len < cmd_len + 2) return false;
    if (strncmp(buf, cmd, cmd_len) != 0) return false;

    // cleanify: extract '=number'
    n = 0;
    for (uint8_t i = cmd_len; i < buf_len; i++) {
        if (buf[i] != ' ') s[n++] = buf[i];
    }
    s[n] = '\0';

    if (n < 1) return false;
    if (s[0] != '=') return false;

    // cleanify: extract 'number'
    s[0] = '0'; // trick to make it easy
    for (uint8_t i = 0; i < strlen(s); i++) if (!isdigit(s[i])) return false;

    *value = atoi(s);

    return true;
}


void tTxCli::print_config_id(void)
{
    puts("ConfigId:");
    putc('0'+Config.ConfigId);
    putsn("");
}


// p name = value or p name
bool tTxCli::is_cmd_param_set(char* const name, char* const svalue)
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
        while (_param_get_listval_fromoptstr(s, idx, i, PARAM_FORMAT_CLI)) {
            if (allowed_mask & (1 << i)) {
                puts("  "); putc(i + '0'); puts(" = "); putsn(s);
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
    uint8_t allowed_num = param_get_allowed_opt_num(idx);

    puts("  ");
    puts(SetupParameter[idx].name);
    puts(" = ");
    if (allowed_num == 0) {
        putsn("- (unavailable)"); // this parameter is not available on this device
        return;
    }
    char s[32];
    param_get_val_formattedstr(s, idx, PARAM_FORMAT_CLI);
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
        uint8_t u8 = *(uint8_t*)(SetupParameterPtr(idx));
        puts(" ["); putc(u8 + '0'); puts("]");
        if (allowed_num == 1) puts("(unchangeable)"); // unmodifiable unalterable immutable unchangeable
        }break;
    case SETUP_PARAM_TYPE_STR6:
        if (Config.FrequencyBand != SETUP_FREQUENCY_BAND_2P4_GHZ) break;
        switch (except_from_bindphrase(s)) {
        case 0: puts(" /--"); break;
        case 1: puts(" /e1"); break;
        case 2: puts(" /e6"); break;
        case 3: puts(" /e11"); break;
        case 4: puts(" /e13"); break;
        }
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
        //delay_ms(10);
    }
}


void tTxCli::stream(void)
{
    uint32_t tnow_ms = millis32();

    if (state == CLI_STATE_STATS) {
        if (tnow_ms - tlast_ms >= 500) {
            tlast_ms = tnow_ms;

            puts(u8toBCD_s(stats.GetLQ_serial()));
            puts("(");
            puts(u8toBCD_s(stats.valid_frames_received.GetLQ()));
            puts("),");
            puts(u8toBCD_s(stats.received_LQ_serial));
            puts(", ");

            puts(s8toBCD_s(stats.last_rssi1));
            puts(",");
            puts(s8toBCD_s(stats.received_rssi));
            puts(", ");
            puts(s8toBCD_s(stats.last_snr1));
            puts("; ");

            puts(u16toBCD_s(stats.bytes_transmitted.GetBytesPerSec()));
            puts(", ");
            puts(u16toBCD_s(stats.bytes_received.GetBytesPerSec()));
            putsn(";");
        }
    }
}


void tTxCli::print_device_version(void)
{
    putsn("  Tx: " DEVICE_NAME ", " VERSIONONLYSTR);

    puts("  Rx: ");
    if (connected()) {
        if (SetupMetaData.rx_available) { // is always true when connected, except when some link task is going on or in bind
            puts(SetupMetaData.rx_device_name);
            puts(", ");
            char s[32];
            version_to_str(s, SetupMetaData.rx_firmware_version);
            putsn(s);
        } else {
            putsn("- (unexpected error)");
        }
    } else {
        putsn("receiver not connected");
    }
}


void tTxCli::print_frequencies(void)
{
char s[32];

    for (uint8_t i = 0; i < fhss.Cnt(); i++) {
        puts(u8toBCD_s(i));
        puts("  ch: ");
        puts(u8toBCD_s(fhss.ChList(i)));
        puts("  f_reg: ");
        puts(u32toBCD_s(fhss.FhssList(i)));
        puts("  f: ");
        u32toBCDstr(fhss.GetFreq_x1000(i), s);
        remove_leading_zeros(s);
        puts(s);
#if defined DEVICE_HAS_SX126x || defined DEVICE_HAS_DUAL_SX126x_SX128x || defined DEVICE_HAS_DUAL_SX126x_SX126x || defined  DEVICE_HAS_SX127x
        putsn(" kHz");
#else
        putsn(" MHz");
#endif
        //delay_ms(20);
    }
}


void tTxCli::print_help(void)
{
    putsn("  help, h, ?  -> this help page");
    putsn("  v           -> print device and version");
    putsn("  pl          -> list all parameters");
    putsn("  pl c        -> list common parameters");
    putsn("  pl tx       -> list Tx parameters");
    putsn("  pl rx       -> list Rx parameters");

    putsn("  p name          -> get parameter value");
    putsn("  p name = value  -> set parameter value");
    putsn("  p name = ?      -> get parameter value and list of allowed values");
    putsn("  pstore      -> store parameters");

    putsn("  setconfigid -> select config id");
    putsn("  bind        -> start binding");
    putsn("  reload      -> reload all parameter settings");
    putsn("  stats       -> starts streaming statistics");
    putsn("  listfreqs   -> lists frequencies used in fhss scheme");

    putsn("  systemboot  -> call system bootloader");

#ifdef USE_ESP_WIFI_BRIDGE
    putsn("  esppt       -> enter serial passthrough");
    putsn("  espboot     -> reboot ESP and enter serial passthrough");
#endif
#ifdef USE_HC04_MODULE
    putsn("  hc04 pt       -> enter serial passthrough");
    putsn("  hc04 setpin   -> set pin of HC04");
#endif
}


void tTxCli::Do(void)
{
char sname[32], svalue[32];
int32_t value;
uint8_t param_idx;
bool rx_param_changed;

    if (!initialized) return;

    //puts(".");

    uint32_t tnow_ms = millis32();
    if (pos && (tnow_ms - tlast_ms > 2000)) { putsn(">"); putsn("  timeout"); clear(); }

    if (state != CLI_STATE_NORMAL) {
        if (com->available()) { com->getc(); state = CLI_STATE_NORMAL; putsn("  streaming stats stopped"); return; }
        delay_off();
        stream();
    }

    delay_clear();

    while (com->available()) {
        char c = com->getc();
        tlast_ms = tnow_ms;

        if (c != '\n' && c != '\r' && c != ',' && c != ';') {
            putc(c);
            addc(c);
            continue;
        }
        putsn(">");

        //-- basic commands
        if (is_cmd("h"))     { print_help(); } else
        if (is_cmd("help"))  { print_help(); } else
        if (is_cmd("?"))     { print_help(); } else
        if (is_cmd("v"))     { print_device_version(); } else
        if (is_cmd("pl"))    { print_config_id(); print_param_list(0); } else
        if (is_cmd("pl c"))  { print_config_id(); print_param_list(1); } else
        if (is_cmd("pl tx")) { print_config_id(); print_param_list(2); } else
        if (is_cmd("pl rx")) { print_config_id(); print_param_list(3);

        } else
        if (is_cmd_param_set(sname, svalue)) { // p name, p name = value
            if (!param_get_idx(&param_idx, sname)) {
                putsn("err: invalid parameter name");
            } else if (!connected() && setup_param_is_rx(param_idx)) {
                putsn("warn: receiver not connected");
            } else if (svalue[0] == '?') {
                print_config_id();
                print_param(param_idx);
                print_param_opt_list(param_idx);
            } else if (svalue[0] == '\0') {
                putsn("err: no value specified");
            } else if (!param_set_val_fromstr(&rx_param_changed, svalue, param_idx)) {
                putsn("err: invalid value");
            } else {
                print_config_id();
                print_param(param_idx);
                if (rx_param_changed) task_pending = TX_TASK_RX_PARAM_SET;
            }

        } else
        if (is_cmd("pstore")) {
            task_pending = TX_TASK_PARAM_STORE;
            print_config_id();
            if (!connected()) {
                putsn("warn: receiver not connected");
                putsn("  Tx parameters stored");
            } else {
                putsn("  parameters stored");
            }

        } else
        if (is_cmd("bind")) {
            task_pending = TX_TASK_BIND;
            putsn("  Tx entered bind mode");

        } else
        if (is_cmd("reload")) {
            task_pending = TX_TASK_PARAM_RELOAD;
            print_config_id();
            if (!connected()) {
                putsn("warn: receiver not connected");
                putsn("  Tx parameters reloaded");
            } else {
                putsn("  parameters reloaded");
            }

        } else
        if (is_cmd_set_value("setconfigid", &value)) { // setconfigid = value
            if (value < 0 || value > 9) {
                putsn("err: invalid config id number (0-9)");
            } else {
            print_config_id();
            if (value == Config.ConfigId) {
                putsn("  no change required");
            } else {
                task_pending = TX_TASK_CLI_CHANGE_CONFIG_ID;
                task_value = value;
                puts("  change ConfigId to ");putc('0'+value);putsn("");
            }
            }

        } else
        if (is_cmd("stats")) {
            state = CLI_STATE_STATS;
            putsn("  starts streaming stats");
            putsn("  send any character to stop");

        //-- miscellaneous
        } else
        if (is_cmd("listfreqs")) {
          print_frequencies();

        //-- System Bootloader
        } else
        if (is_cmd("systemboot")) {
            task_pending = TX_TASK_SYSTEM_BOOT;

        //-- ESP handling
#ifdef USE_ESP_WIFI_BRIDGE
        } else
        if (is_cmd("esppt")) {
            // enter esp passthrough, can only be exited by re-powering
            task_pending = TX_TASK_ESP_PASSTHROUGH;
        } else
        if (is_cmd("espboot")) {
            // enter esp flashing, can only be exited by re-powering
            task_pending = TX_TASK_FLASH_ESP;
#endif

        //-- HC04 module handling
#ifdef USE_HC04_MODULE
        } else
        if (is_cmd("hc04 pt")) {
            // enter hc04 passthrough, can only be exited by re-powering
            task_pending = TX_TASK_HC04_PASSTHROUGH;
        } else
        if (is_cmd_set_value("hc04 setpin", &value)) { // setpin = value
            if (value < 1000 || value > 9999) {
                putsn("err: invalid pin number");
            } else {
                char pin_str[32];
                u16toBCDstr(value, pin_str);
                remove_leading_zeros(pin_str);
                puts("HC04 Pin: ");putsn(pin_str);
                task_pending = TX_TASK_CLI_HC04_SETPIN;
                task_value = value;
            }
#endif

        //-- invalid command
        } else {
            putsn((!pos) ? "  empty cmd" : "  invalid cmd");
        }

        clear();
    }
}


#endif // TX_CLI_H



