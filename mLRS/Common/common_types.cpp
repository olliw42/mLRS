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
#include "protocols/crsf_protocol.h"


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
//   rssi = (127 + stats.last_rssi);
//   remrssi = (127 + stats.received_rssi);
//
// https://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html#monitoring-the-link-quality
// for Sik radios holds signal_dBm approx rssi_SiK/1.9 - 127  => 150 approx -48dBm, 70 approx -90dBm
// so we could here a SiK scale
//   rssi_SiK = ( signal_dBm + 127 ) * 1.9
//
//   int32_t rssi_SiK = ( ((int32_t)stats.last_rssi + 127) * 19000 ) / 10000;
//   if (rssi_SiK < 0) rssi_SiK = 0;
//   if (rssi_SiK > 250) rssi_SiK = 250;
//   rssi = rssi_SiK;
//
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp#L483-L510
//   -120 ... -50 -> 0 .. 254
//
// ArduPilot wants to have it in range 0..254, which it scales to 0.0..1.0, 255 gets 0.0
// TODO: we should take into account in the scaling a LNA as well as the db min for a board & setting
uint8_t rssi_i8_to_ap(int8_t rssi_i8)
{
    if (rssi_i8 == RSSI_INVALID) return UINT8_MAX;
    if (rssi_i8 > -50) return 254; // max value
    if (rssi_i8 < -120) return 0;

    int32_t r = (int32_t)rssi_i8 - (-120);
    constexpr int32_t m = (int32_t)(-50) - (-120);

    return (r * 254 + m/2) / m;
}


// 0 ... 253 -> 0 .. -253 dBm, 254 = no link connection, 255 = unknown
uint8_t rssi_i8_to_mavradio(int8_t rssi_i8, bool connected)
{
    if (rssi_i8 == RSSI_INVALID) return UINT8_MAX;
    if (!connected) return 254;
    if (rssi_i8 >= 0) return 0; // max rssi value
    return -rssi_i8;
}


// convert rssi to internal rc range, then further converted to output, -120 ... -50 -> 172 .. 1877
uint16_t rssi_i8_to_rc(int8_t rssi_i8)
{
    if (rssi_i8 == RSSI_INVALID) return 0;
    if (rssi_i8 > -50) return 1877; // max value
    if (rssi_i8 < -120) return 172; // min value

    int32_t r = (int32_t)rssi_i8 - (-120);
    constexpr int32_t m = (int32_t)(-50) - (-120);

    return (r * 1705 + m/2) / m + 172;
}


// convert lq to internal rc range, then further converted to output, 0 ... 100 -> 172 .. 1877 = 1000 .. 2000 us
uint16_t lq_to_rc(uint8_t lq)
{
    if (lq >= 100) return 1877; // max value

    return ((uint32_t)lq * 1705 + 50) / 100 + 172;
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
    return (((int32_t)(rc_ch) - 1024) * 1920) / 2047 + 992;
}


uint16_t rc_to_crsf(uint16_t rc_ch)
{
    return (((int32_t)(rc_ch) - 1024) * 1920) / 2047 + 992;
}


uint16_t rc_to_mavlink(uint16_t rc_ch)
{
    return (((int32_t)(rc_ch) - 1024) * 1200) / 2047 + 1500; // 1200 = 1920 * 5/8
}


int16_t rc_to_mavlink_13bcentered(uint16_t rc_ch)
{
//    return ((int32_t)(rc_ch) - 1024) * 4;
    return (((int32_t)(rc_ch) - 1024) * 15) / 4; // let's mimic rc_to_mavlink()
}


//-- CRSF

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
    if (mode == MODE_50HZ) return CRSF_RFMODE_50_HZ;
    if (mode == MODE_FLRC_111HZ) return 111;
    if (mode == MODE_FSK_50HZ) return CRSF_RFMODE_50_HZ;
    return UINT8_MAX;
}


uint8_t crsf_cvt_fps(uint8_t mode)
{
    if (mode == MODE_19HZ) return 2; // *10 in OpenTx !
    if (mode == MODE_31HZ) return 3;
    if (mode == MODE_50HZ) return 5;
    if (mode == MODE_FLRC_111HZ) return 11;
    if (mode == MODE_FSK_50HZ) return 5;
    return UINT8_MAX;
}


// receiver side: flight control stacks appear to expect rssi as positive value
uint8_t crsf_cvt_rssi_rx(int8_t rssi_i8)
{
    if (rssi_i8 == RSSI_INVALID) return 0;
    return -rssi_i8;
}


// tx module side: OpenTx/EdgeTx radios can handle rssi as negative value, and this seems to be preferred
uint8_t crsf_cvt_rssi_tx(int8_t rssi_i8)
{
    if (rssi_i8 == RSSI_INVALID) return 0;
    if (rssi_i8 > RSSI_MAX) return RSSI_MAX; // limit to -1
    return rssi_i8;
}


uint8_t crsf_cvt_rssi_percent(int8_t rssi, int16_t receiver_sensitivity_dbm)
{
    if (rssi == RSSI_INVALID) return 255;
    if (rssi >= -50) return 100;
    if (rssi <= receiver_sensitivity_dbm) return 0;

    int32_t r = (int32_t)rssi - receiver_sensitivity_dbm;
    int32_t m = (int32_t)(-50) - receiver_sensitivity_dbm;

    return (100 * r + 49)/m;
}


// CRSF crc8 is CRC-8/DVB-S2
// check 0xBC, poly 0xD5, init 0x00, not inverted, xor out 0x00
// see https://crccalc.com/
// 0xD5 = 0x1D5 = 0xEA in Koopman notation
// https://users.ece.cmu.edu/~koopman/crc/index.html
// it is not listed in the "Best CRCs" table, but here https://users.ece.cmu.edu/~koopman/crc/crc8.html
// brute force:
//    crc ^= a;
//    for (uint8_t i = 0; i < 8; i++) {
//        crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
//    }
// we are not short of flash, so we use lookup table
#define CRSF_CRC8_LOOKUP_TABLE
#ifndef CRSF_CRC8_LOOKUP_TABLE

uint8_t crsf_crc8_calc(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80) { crc = (crc << 1) ^ 0xD5; } else { crc = (crc << 1); }
    }
    return crc;
}


uint8_t crsf_crc8_update(uint8_t crc, const uint8_t* buf, uint16_t len)
{
    while (len--) {
        crc = crsf_crc8_calc(crc, *buf++);
    }
    return crc;
}

#else
#pragma GCC push_options
#pragma GCC optimize ("O3")

// generated from https://crccalc.com/, for CRC-8/DVB-S2
// matches table give in CRSF specs, https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#crc
const uint8_t crsf_crc8_table[256] = {
    0x00 , 0xd5 , 0x7f , 0xaa , 0xfe , 0x2b , 0x81 , 0x54,
    0x29 , 0xfc , 0x56 , 0x83 , 0xd7 , 0x02 , 0xa8 , 0x7d,
    0x52 , 0x87 , 0x2d , 0xf8 , 0xac , 0x79 , 0xd3 , 0x06,
    0x7b , 0xae , 0x04 , 0xd1 , 0x85 , 0x50 , 0xfa , 0x2f,
    0xa4 , 0x71 , 0xdb , 0x0e , 0x5a , 0x8f , 0x25 , 0xf0,
    0x8d , 0x58 , 0xf2 , 0x27 , 0x73 , 0xa6 , 0x0c , 0xd9,
    0xf6 , 0x23 , 0x89 , 0x5c , 0x08 , 0xdd , 0x77 , 0xa2,
    0xdf , 0x0a , 0xa0 , 0x75 , 0x21 , 0xf4 , 0x5e , 0x8b,
    0x9d , 0x48 , 0xe2 , 0x37 , 0x63 , 0xb6 , 0x1c , 0xc9,
    0xb4 , 0x61 , 0xcb , 0x1e , 0x4a , 0x9f , 0x35 , 0xe0,
    0xcf , 0x1a , 0xb0 , 0x65 , 0x31 , 0xe4 , 0x4e , 0x9b,
    0xe6 , 0x33 , 0x99 , 0x4c , 0x18 , 0xcd , 0x67 , 0xb2,
    0x39 , 0xec , 0x46 , 0x93 , 0xc7 , 0x12 , 0xb8 , 0x6d,
    0x10 , 0xc5 , 0x6f , 0xba , 0xee , 0x3b , 0x91 , 0x44,
    0x6b , 0xbe , 0x14 , 0xc1 , 0x95 , 0x40 , 0xea , 0x3f,
    0x42 , 0x97 , 0x3d , 0xe8 , 0xbc , 0x69 , 0xc3 , 0x16,
    0xef , 0x3a , 0x90 , 0x45 , 0x11 , 0xc4 , 0x6e , 0xbb,
    0xc6 , 0x13 , 0xb9 , 0x6c , 0x38 , 0xed , 0x47 , 0x92,
    0xbd , 0x68 , 0xc2 , 0x17 , 0x43 , 0x96 , 0x3c , 0xe9,
    0x94 , 0x41 , 0xeb , 0x3e , 0x6a , 0xbf , 0x15 , 0xc0,
    0x4b , 0x9e , 0x34 , 0xe1 , 0xb5 , 0x60 , 0xca , 0x1f,
    0x62 , 0xb7 , 0x1d , 0xc8 , 0x9c , 0x49 , 0xe3 , 0x36,
    0x19 , 0xcc , 0x66 , 0xb3 , 0xe7 , 0x32 , 0x98 , 0x4d,
    0x30 , 0xe5 , 0x4f , 0x9a , 0xce , 0x1b , 0xb1 , 0x64,
    0x72 , 0xa7 , 0x0d , 0xd8 , 0x8c , 0x59 , 0xf3 , 0x26,
    0x5b , 0x8e , 0x24 , 0xf1 , 0xa5 , 0x70 , 0xda , 0x0f,
    0x20 , 0xf5 , 0x5f , 0x8a , 0xde , 0x0b , 0xa1 , 0x74,
    0x09 , 0xdc , 0x76 , 0xa3 , 0xf7 , 0x22 , 0x88 , 0x5d,
    0xd6 , 0x03 , 0xa9 , 0x7c , 0x28 , 0xfd , 0x57 , 0x82,
    0xff , 0x2a , 0x80 , 0x55 , 0x01 , 0xd4 , 0x7e , 0xab,
    0x84 , 0x51 , 0xfb , 0x2e , 0x7a , 0xaf , 0x05 , 0xd0,
    0xad , 0x78 , 0xd2 , 0x07 , 0x53 , 0x86 , 0x2c , 0xf9,
};


uint8_t crsf_crc8_calc(uint8_t crc, uint8_t data)
{
    return crsf_crc8_table[crc ^ data];
}


uint8_t crsf_crc8_update(uint8_t crc, const void* buf, uint16_t len)
{
    const uint8_t* pbuf = (const uint8_t*)buf; // to avoid warning ISO C++ forbids incrementing a pointer of type 'const void*'
    while (len--) {
        crc = crsf_crc8_table[crc ^ *pbuf++];
    }
    return crc;
}

#pragma GCC pop_options
#endif


//-- DroneCAN

const uint16_t power_table_dBm_to_mW[] = {
    1, // 0 dBm
    1, // 1 dBm
    2, // 2 dBm
    2, // 3 dBm
    2, // 4 dBm
    3, // 5 dBm
    4, // 6 dBm
    5, // 7 dBm
    6, // 8 dBm
    8, // 9 dBm
    10, // 10 dBm
    12, // 11 dBm
    16, // 12 dBm
    20, // 13 dBm
    25, // 14 dBm
    32, // 15 dBm
    40, // 16 dBm
    50, // 17 dBm
    63, // 18 dBm
    80, // 19 dBm
    100, // 20 dBm
    125, // 21 dBm
    158, // 22 dBm
    200, // 23 dBm
    250, // 24 dBm
    316, // 25 dBm
    400, // 26 dBm
    500, // 27 dBm
    630, // 28 dBm
    800, // 29 dBm
    1000, // 30 dBm
    1250, // 31 dBm
    1500, // 32 dBm
    2000, // 33 dBm
};

uint8_t dronecan_cvt_power(int8_t power_dbm)
{
    if (power_dbm < 0) return 0;
    if (power_dbm > 31) return 250; // 1250 / 5;
    return power_table_dBm_to_mW[power_dbm] / 5;
}


uint16_t cvt_power(int8_t power_dbm)
{
    if (power_dbm < 0) return 0;
    if (power_dbm > 33) return 2000;
    return power_table_dBm_to_mW[power_dbm];
}


//-- bind phrase & power & version

bool is_valid_bindphrase_char(char c)
{
    return ((c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9' ) ||
            (c == '_') || (c == '#') || (c == '-') || (c == '.'));
}


void sanitize_bindphrase(char* const bindphrase, const char* const bindphrase_default)
{
    uint8_t invalid_cnt = 0;
    for (uint8_t i = 0; i < 6; i++) {
        if (bindphrase[i] == 0xFF) invalid_cnt++;
        if (!is_valid_bindphrase_char(bindphrase[i])) bindphrase[i] = '_';
    }

    if (invalid_cnt == 6 && bindphrase_default != nullptr) {
        memcpy(bindphrase, bindphrase_default, 6);
    }

    bindphrase[6] = '\0';
}


uint32_t u32_from_bindphrase(char* const bindphrase)
{
    uint64_t v = 0;
    uint64_t base = 1;

    for (uint8_t i = 0; i < 6; i++) {

        char* cptr = strchr(bindphrase_chars, bindphrase[i]);
        uint8_t n = (cptr) ? cptr - bindphrase_chars : 0; // must not happen that c is not found, but play it safe

        v += n * base;
        base *= 40;
    }

    return (uint32_t)v;
}


uint8_t except_from_bindphrase(char* const bindphrase)
{
    char c = bindphrase[5]; // take last char

    if (c >= '0' && c <= '9') {
        return (c - '0') % 5; // no, #1, #6, #11, #13 = 5 cases = EXCEPT_NUM
    }

    char* cptr = strchr(bindphrase_chars, c);
    uint8_t n = (cptr) ? cptr - bindphrase_chars : 0; // must not happen that c is not found, but play it safe

    return n % 5; // no, #1, #6, #11, #13 = 5 cases = EXCEPT_NUM
}


void bindphrase_from_u32(char* const bindphrase, uint32_t bindphrase_u32)
{
    uint32_t base = 40*40*40*40*40; // 40^5

    for (uint8_t i = 0; i < 6; i++) {

        uint32_t v = bindphrase_u32 / base;
        bindphrase[5 - i] = (v < 40) ? bindphrase_chars[v] : '0'; // must not happen, but play it safe

        bindphrase_u32 -= v * base;
        base /= 40;
    }
}


void remove_leading_zeros(char* const s)
{
uint16_t i, len;

    len = strlen(s);
    for (i = 0; i < len - 1; i++) {
        if (s[i] != '0') break;
    }
    memmove(&s[0], &s[i], len - i + 1);
}


void power_optstr_from_power_list(char* const Power_optstr, int16_t* const power_list, uint8_t num, uint8_t slen)
{
    memset(Power_optstr, 0, slen);

    char optstr[44+2] = {};

    for (uint8_t i = 0; i < num; i++) {
        char s[44+2];
        if (power_list[i] == INT16_MAX) break;

        if (power_list[i] <= 0) {
            strcpy(s, "min,");
        } else
        if (power_list[i] < 1000) {
            u16toBCDstr(power_list[i], s);
            remove_leading_zeros(s);
            strcat(s, " mW,");
        } else {
            u16toBCDstr((power_list[i] + 50) / 100, s);
            remove_leading_zeros(s);
            uint8_t l = strlen(s);
            s[l] = s[l-1]; s[l-1] = '.'; s[l+1] = '\0';
            strcat(s, " W,");
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


void power_optstr_from_rfpower_list(char* const Power_optstr, const rfpower_t* const rfpower_list, uint8_t num, uint8_t slen)
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


void version_to_str(char* const s, uint32_t version)
{
char ss[32];

    uint32_t major = version / 10000;
    version -= major * 10000;
    uint32_t minor = version / 100;
    version -= minor * 100;
    uint32_t patch = version;

    strcpy(s, "v");
    u8toBCDstr(major, ss);
    remove_leading_zeros(ss);
    strcat(s, ss);
    strcat(s, ".");
    u8toBCDstr(minor, ss);
    remove_leading_zeros(ss);
    strcat(s, ss);
    strcat(s, ".");
    u8toBCDstr(patch, ss);
    ss[0] = ss[1]; ss[1] = ss[2]; ss[2] = '\0'; // remove_leading_zeros(ss);
    strcat(s, ss);
}


//-- auxiliary functions


// copy a string into a buffer with max len chars
void strbufstrcpy(char* const res, const char* const src, uint16_t len)
{
    memset(res, '\0', len);
    for (uint16_t i = 0; i < len; i++) {
        if (src[i] == '\0') return;
        res[i] = src[i];
    }
}


// copy a buffer into a string with max len chars (i.e. len + 1 size)
void strstrbufcpy(char* const res, const char* const src, uint16_t len)
{
    memset(res, '\0', len + 1); // this ensures that res is terminated with a '\0'
    for (uint16_t i = 0; i < len; i++) {
        if (src[i] == '\0') return;
        res[i] = src[i];
    }
}


bool strbufeq(char* const s1, const char* const s2, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (s1[i] == '\0' && s2[i] == '\0') return true;
        if (s1[i] == '\0') return false;
        if (s2[i] == '\0') return false;
        if (s1[i] != s2[i]) return false;
    }
    return true;
}
