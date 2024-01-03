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
#include "math.h"
#include "setup_tx.h"


extern volatile uint32_t millis32(void);
extern TxStatsBase txstats;
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
bool _param_get_listval_fromoptstr(char* s, uint8_t param_idx, uint8_t value, uint8_t format)
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


// helper, gets parameter value as formatted string, different formats can be specified
bool param_get_val_formattedstr(char* s, uint8_t param_idx, uint8_t format = PARAM_FORMAT_DEFAULT)
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
bool param_set_val_fromint(bool* rx_param_changed, int32_t value, uint8_t param_idx)
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
bool param_set_str6val(bool* rx_param_changed, char* svalue, uint8_t param_idx)
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
bool param_set_val_fromstr(bool* rx_param_changed, char* svalue, uint8_t param_idx)
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

typedef enum {
    CLI_TASK_NONE = 0,
    CLI_TASK_RX_PARAM_SET,
    CLI_TASK_PARAM_STORE,
    CLI_TASK_BIND,
    CLI_TASK_PARAM_RELOAD,
    CLI_TASK_BOOT,
    CLI_TASK_FLASH_ESP,
    CLI_TASK_CHANGE_CONFIG_ID,
} CLI_TASK_ENUM;


class tTxCli
{
  public:
    void Init(tSerialBase* _com = nullptr);
    void Set(uint8_t new_line_end = CLI_LINE_END_CR);
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

    bool is_cmd(const char* cmd);
    bool is_cmd_param_set(char* name, char* svalue);
    bool is_cmd_set_value(const char* cmd, int32_t* value);

    void putc(char c) { com->putc(c); }
    void puts(const char* s) { com->puts(s); }
    void putsn(const char* s) { com->puts(s); com->puts(ret); }

    void print_config_id(void);

    tSerialBase* com;

    uint8_t line_end;
    char ret[4];

    char buf[128];
    uint8_t pos;
    uint32_t tlast_ms;

    uint8_t task_pending;
    int32_t task_value;

    uint8_t state;
};


void tTxCli::Init(tSerialBase* _com)
{
    com = _com;

    line_end = CLI_LINE_END_CR;
    strcpy(ret, "\r");

    pos = 0;
    buf[pos] = '\0';
    tlast_ms = 0;

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



// cmd;
bool tTxCli::is_cmd(const char* cmd)
{
    return (strcmp(buf, cmd) == 0) ? true : false;
}


// name = value or p name
bool tTxCli::is_cmd_set_value(const char* cmd, int32_t* value)
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

    if (n != 2) return false;
    if (s[0] != '=') return false;
    if (s[1] < '0' || s[1] > '9') return false;

    *value = s[1] - '0';

    return true;
}


void tTxCli::print_config_id(void)
{
    puts("ConfigId:");
    putc('0'+Config.ConfigId);
    putsn("");
}


// p name = value or p name
bool tTxCli::is_cmd_param_set(char* name, char* svalue)
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
        delay_ms(10);
    }
}


void tTxCli::stream(void)
{
    uint32_t tnow_ms = millis32();

    if (state == CLI_STATE_STATS) {
        if (tnow_ms - tlast_ms >= 500) {
            tlast_ms = tnow_ms;

            puts(u8toBCD_s(txstats.GetLQ_serial()));
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


#ifdef DEVICE_HAS_SX126x
#define CLI_REG_TO_FREQ(f_reg)  roundf( (float)f_reg * ((double)SX126X_FREQ_XTAL_HZ * 1.0E-3 / (double)(1 << 25)) )
#elif defined DEVICE_HAS_SX127x
#define SX12XX_FREQ_MHZ_TO_REG(f_mhz)  SX127X_FREQ_MHZ_TO_REG(f_mhz)
#define CLI_REG_TO_FREQ(f_reg)  roundf( (float)f_reg * ((double)SX127X_FREQ_XTAL_HZ * 1.0E-3 / (double)(1 << 19)) )
#else
#define CLI_REG_TO_FREQ(f_reg)  roundf( (float)f_reg * (1.0E-6 / (double)(1 << 18)) )
#endif

void tTxCli::print_frequencies(void)
{
char s[32];

    for (uint8_t i = 0; i < fhss.Cnt(); i++) {
        puts(u8toBCD_s(i));
        puts("  ch: ");
        puts(u8toBCD_s(fhss.ChList(i)));
        puts("  freg: ");
        puts(u32toBCD_s(fhss.FhssList(i)));
        puts("  f: ");
        u32toBCDstr(CLI_REG_TO_FREQ(fhss.FhssList(i)), s);
        remove_leading_zeros(s);
        puts(s);
#if defined DEVICE_HAS_SX126x || defined  DEVICE_HAS_SX127x
        putsn(" kHz");
#else
        putsn(" MHz");
#endif
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
    delay_ms(10);
    putsn("  p name          -> get parameter value");
    putsn("  p name = value  -> set parameter value");
    putsn("  p name = ?      -> get parameter value and list of allowed values");
    putsn("  pstore      -> store parameters");
    delay_ms(10);
    putsn("  bind        -> start binding");
    putsn("  reload      -> reload all parameter settings");
    putsn("  stats       -> starts streaming statistics");
    putsn("  listfreqs   -> lists frequencies used in fhss scheme");
    delay_ms(10);

    putsn("  ptser       -> enter serial passthrough");
    putsn("  systemboot  -> call system bootloader");

#ifdef USE_ESP_WIFI_BRIDGE
    putsn("  espboot     -> reboot ESP and enter serial passthrough");
    putsn("  espcli      -> GPIO0 = low and enter serial passthrough");
#endif
}


// TODO: should really be abstracted out, but for the moment let's be happy to have it here
void passthrough_do(tSerialBase* ser, tSerialBase* ser2)
{
uint16_t led_blink = 0;

    if (!ser) return;
    if (!ser2) return;

    LED_RED_OFF;
    LED_GREEN_ON;

    while (1) {
        if (doSysTask) {
            doSysTask = 0;
            DECc(led_blink, SYSTICK_DELAY_MS(100));
            if (!led_blink) { LED_GREEN_TOGGLE; LED_RED_TOGGLE; }
        }

        if (ser->available()) {
            char c = ser->getc();
            ser2->putc(c);
        }
        if (ser2->available()) {
            char c = ser2->getc();
            ser->putc(c);
        }
    }
}


// TODO: should really be abstracted out, but for the moment let's be happy to have it here
void flashesp_do(tSerialBase* com)
{
#ifdef USE_ESP_WIFI_BRIDGE
#ifdef USE_ESP_WIFI_BRIDGE_RST_GPIO0
    esp_gpio0_low();
    esp_reset_low();
    delay_ms(100);
    esp_reset_high();
    delay_ms(100);
    esp_gpio0_high();
    delay_ms(100);
#endif
#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL
    serial.SetBaudRate(115200);
    passthrough_do(com, &serial);
#endif
#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
    serial2.SetBaudRate(115200);
    passthrough_do(com, &serial2);
#endif
#endif
}


void tTxCli::Do(void)
{
char sname[32], svalue[32];
int32_t value;
uint8_t param_idx;
bool rx_param_changed;

    if (!com) return;

    //puts(".");

    uint32_t tnow_ms = millis32();
    if (pos && (tnow_ms - tlast_ms > 2000)) { putsn(">"); putsn("  timeout"); clear(); }

    if (state != CLI_STATE_NORMAL) {
        if (com->available()) { com->getc(); state = CLI_STATE_NORMAL; putsn("  streaming stats stopped"); return; }
        stream();
    }

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
                if (rx_param_changed) task_pending = CLI_TASK_RX_PARAM_SET;
            }

        } else
        if (is_cmd("pstore")) {
            task_pending = CLI_TASK_PARAM_STORE;
            print_config_id();
            if (!connected()) {
                putsn("warn: receiver not connected");
                putsn("  Tx parameters stored");
            } else {
                putsn("  parameters stored");
            }

        } else
        if (is_cmd("bind")) {
            task_pending = CLI_TASK_BIND;
            putsn("  Tx entered bind mode");

        } else
        if (is_cmd("reload")) {
            task_pending = CLI_TASK_PARAM_RELOAD;
            print_config_id();
            if (!connected()) {
                putsn("warn: receiver not connected");
                putsn("  Tx parameters reloaded");
            } else {
                putsn("  parameters reloaded");
            }

        } else
        if (is_cmd_set_value("setconfigid", &value)) { // setconfigid = value
            print_config_id();
            if (value == Config.ConfigId) {
                putsn("  no change required");
            } else {
                task_pending = CLI_TASK_CHANGE_CONFIG_ID;
                task_value = value;
                puts("  change ConfigId to ");putc('0'+value);putsn("");
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
            task_pending = CLI_TASK_BOOT;

        //-- ESP handling
        } else
        if (is_cmd("ptser")) {
            // enter passthrough to serial, can only be exited by re-powering
            serial.SetBaudRate(115200);
            passthrough_do(com, &serial);
#ifdef USE_ESP_WIFI_BRIDGE
        } else
        if (is_cmd("espboot")) {
            task_pending = CLI_TASK_FLASH_ESP;
        } else
        if (is_cmd("espcli")) {
            // enter esp cli, can only be exited by re-powering
#ifdef USE_ESP_WIFI_BRIDGE_RST_GPIO0
            esp_gpio0_low();
            delay_ms(100);
#endif
#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL
            serial.SetBaudRate(115200);
            passthrough_do(com, &serial);
#endif
#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
            serial2.SetBaudRate(115200);
            passthrough_do(com, &serial2);
#endif
#endif

        //-- invalid command
        } else {
            putsn((!pos) ? "  empty cmd" : "  invalid cmd");
        }

        clear();
    }
}


#endif // TX_CLI_H



