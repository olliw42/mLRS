//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// COMMON TYPES
//*******************************************************

#include <string.h>
#include "../modules/stm32ll-lib/src/stdstm32.h"
#include "common_types.h"
#include "setup_types.h"
#include "crsf_protocol.h"


//-- rssi & snr

uint8_t rssi_u7_from_i8(int8_t rssi_i8)
{
    if (rssi_i8 == RSSI_INVALID) return RSSI_U7_INVALID;

    // constrain to -1...-127
    if (rssi_i8 > RSSI_MAX) return RSSI_U7_MAX;
    if (rssi_i8 < RSSI_MIN) return RSSI_U7_MIN;

    return -rssi_i8;
}


int8_t rssi_i8_from_u7(uint8_t rssi_u7)
{
    if (rssi_u7 == RSSI_U7_INVALID) return RSSI_INVALID;

    return -rssi_u7;
}


// this would be the naive thing
// scale is "inverted" to make it that it is higher the better
//   rssi = (127 + stats.last_rx_rssi);
//   remrssi = (127 + stats.received_rssi);
//
// https://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html#monitoring-the-link-quality
// for Sik radios holds signal_dBm approx rssi_SiK/1.9 - 127  => 150 approx -48dBm, 70 approx -90dBm
// so we could here a SiK scale
//   rssi_SiK = ( signal_dBm + 127 ) * 1.9
//
//   int32_t rssi_SiK = ( ((int32_t)stats.last_rx_rssi + 127) * 19000 ) / 10000;
//   if (rssi_SiK < 0) rssi_SiK = 0;
//   if (rssi_SiK > 250) rssi_SiK = 250;
//   rssi = rssi_SiK;
//
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp#L483-L510
//   -120 ... -50 -> 0 .. 254
uint8_t rssi_i8_to_ap(int8_t rssi_i8)
{
    if (rssi_i8 == RSSI_INVALID) return UINT8_MAX;
    if (rssi_i8 > -50) return 254; // max value
    if (rssi_i8 < -120) return 0;

    int32_t r = (int32_t)rssi_i8 - (-120);
    constexpr int32_t m = (int32_t)(-50) - (-120);

    return (r * 254 + m/2) / m;
}


//   -120 ... -50 -> 172 .. 1877
uint16_t rssi_i8_to_ap_sbus(int8_t rssi_i8)
{
    if (rssi_i8 == RSSI_INVALID) return 0;
    if (rssi_i8 > -50) return 1877; // max value
    if (rssi_i8 < -120) return 172; // min value

    int32_t r = (int32_t)rssi_i8 - (-120);
    constexpr int32_t m = (int32_t)(-50) - (-120);

    return (r * 1705 + m/2) / m + 172;
}


//-- rc data

uint16_t clip_rc(int32_t x)
{
    if (x <= 1) return 1;
    if (x >= 2047) return 2047;
    return x;
}


uint16_t rc_from_sbus(uint16_t sbus_ch)
{
    return clip_rc( (((int32_t)(sbus_ch) - 992) * 2047) / 1966 + 1024 );
}


uint16_t rc_from_crsf(uint16_t crsf_ch)
{
    return clip_rc( (((int32_t)(crsf_ch) - 992) * 2047) / 1966 + 1024 );
}


uint16_t rc_to_sbus(uint16_t rc_ch)
{
    return (((int32_t)(rc_ch) - 1024) * 1920) / 2047 + 1000;
}


uint16_t rc_to_crsf(uint16_t rc_ch)
{
    return (((int32_t)(rc_ch) - 1024) * 1920) / 2047 + 1000;
}


uint16_t rc_to_mavlink(uint16_t rc_ch)
{
    return (((int32_t)(rc_ch) - 1024) * 1200) / 2047 + 1500; // 1200 = 1920 * 5/8
}


//-- crsf

uint8_t crsf_cvt_power(int8_t power_dbm)
{
    if (power_dbm <= 3) return CRSF_POWER_0_mW; // 0 dBm
    if (power_dbm <= 12) return CRSF_POWER_10_mW; // 10 dBm
    if (power_dbm <= 15) return CRSF_POWER_25_mW; // 14 dBm
    if (power_dbm <= 18) return CRSF_POWER_50_mW; // 17 dBm
    if (power_dbm <= 22) return CRSF_POWER_100_mW; // 20 dBm
    if (power_dbm <= 25) return CRSF_POWER_250_mW; // 24 dBm
    if (power_dbm <= 28) return CRSF_POWER_500_mW; // 27 dBm
    if (power_dbm <= 31) return CRSF_POWER_1000_mW; // 30 dBm
    if (power_dbm <= 33) return CRSF_POWER_2000_mW; // 33 dBm
    return UINT8_MAX; // makes it red in otx
}


uint8_t crsf_cvt_mode(uint8_t mode)
{
    if (mode == MODE_19HZ) return 19;
    if (mode == MODE_31HZ) return 31;
    if (mode == MODE_50HZ) return CRSF_RFMODE_50HZ;
    return UINT8_MAX;
}


uint8_t crsf_cvt_fps(uint8_t mode)
{
    if (mode == MODE_19HZ) return 2; // *10 in OpenTx !
    if (mode == MODE_31HZ) return 3;
    if (mode == MODE_50HZ) return 5;
    return UINT8_MAX;
}


uint8_t crsf_cvt_rssi(int8_t rssi_i8)
{
    if (rssi_i8 == RSSI_INVALID) return 0;
    return -rssi_i8;
}


//-- bind phrase & version

void sanitize_bindphrase(char* bindphrase)
{
    for (uint8_t i = 0; i < 6; i++) {

        if (bindphrase[i] >= 'a' && bindphrase[i] <= 'z' ) {
        } else
        if (bindphrase[i] >= '0' && bindphrase[i] <= '9' ) {
        } else
        if (bindphrase[i] == '_') {
        } else
        if (bindphrase[i] == '#') {
        } else
        if (bindphrase[i] == '-') {
        } else
        if (bindphrase[i] == '.') {
        } else {
          bindphrase[i] = '_';
        }
    }

    bindphrase[6] = '\0';
}


uint32_t u32_from_bindphrase(char* bindphrase)
{
    uint64_t v = 0;
    uint64_t base = 1;

    for (uint8_t i = 0; i < 6; i++) {
        uint8_t n = 0;

        if (bindphrase[i] >= 'a' && bindphrase[i] <= 'z' ) {
            n = bindphrase[i] - 'a';
        } else
        if (bindphrase[i] >= '0' && bindphrase[i] <= '9' ) {
            n = 26 + bindphrase[i] - '0';
        } else
        if (bindphrase[i] == '_') {
            n = 36;
        } else
        if (bindphrase[i] == '#') {
            n = 37;
        } else
        if (bindphrase[i] == '-') {
            n = 38;
        } else
        if (bindphrase[i] == '.') {
            n = 39;
        }

        v += n * base;
        base *= 40;
    }

    return (uint32_t)v;
}


void remove_leading_zeros(char* s)
{
uint16_t i, len;

    len = strlen(s);
    for (i = 0; i < len - 1; i++) {
        if (s[i] != '0') break;
    }
    memmove(&s[0], &s[i], len - i + 1);
}


void power_optstr_from_power_list(char* Power_optstr, int16_t* power_list, uint8_t num, uint8_t slen)
{
    memset(Power_optstr, 0, slen);

    char optstr[32+2] = {0};

    for (uint8_t i = 0; i < num; i++) {
        char s[32];
        if (power_list[i] == INT16_MAX) break;

        if (power_list[i] <= 0) {
            strcpy(s, "min,");
        } else{
            u16toBCDstr(power_list[i], s);
            remove_leading_zeros(s);
            strcat(s, " mW,");
        }
        if (strlen(optstr) + strlen(s) <= slen) { // we are going to cut off the last char, hence <=
            strcat(optstr, s);
        }
    }

    uint16_t len = strlen(optstr);
    if (len > 0) {
        optstr[len - 1] = '\0'; // cut off last ','
    }

    strcpy(Power_optstr, optstr);
}


void power_optstr_from_rfpower_list(char* Power_optstr, const rfpower_t* rfpower_list, uint8_t num, uint8_t slen)
{
int16_t power_list[16];

    for (uint8_t i = 0; i < num; i++) power_list[i] = rfpower_list[i].mW;

    power_optstr_from_power_list(Power_optstr, power_list, num, slen);
}


// u16 firmware version format is: 4.6.6  = 16.64.64

uint16_t version_to_u16(uint32_t version)
{
    uint32_t major = version / 10000;
    version -= major * 10000;
    uint32_t minor = version / 100;
    version -= minor * 100;
    uint32_t patch = version;

    return (major << 12) + (minor << 6) + patch;
}


uint32_t version_from_u16(uint16_t version_u16)
{
    uint32_t major = (version_u16 & 0xF000) >> 12;
    uint32_t minor = (version_u16 & 0x0FC0) >> 6;
    uint32_t patch = (version_u16 & 0x003F);

    return major * 10000 + minor * 100 + patch;
}


//-- auxiliary functions

void strbufstrcpy(char* res, const char* src, uint16_t len)
{
    memset(res, '\0', len);
    for (uint16_t i = 0; i < len; i++) {
        if (src[i] == '\0') return;
        res[i] = src[i];
    }
}


void strstrbufcpy(char* res, const char* src, uint16_t len)
{
    memset(res, '\0', len + 1); // this ensures that res is terminated with a '\0'
    for (uint16_t i = 0; i < len; i++) {
        if (src[i] == '\0') return;
        res[i] = src[i];
    }
}


bool strbufeq(char* s1, const char* s2, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (s1[i] == '\0' && s2[i] == '\0') return true;
        if (s1[i] == '\0') return false;
        if (s2[i] == '\0') return false;
        if (s1[i] != s2[i]) return false;
    }
    return true;
}
