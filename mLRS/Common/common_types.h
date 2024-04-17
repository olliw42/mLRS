//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// COMMON TYPES
//*******************************************************
#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H
#pragma once


#include <inttypes.h>


#define ARRAY_LEN(x)  sizeof(x)/sizeof(x[0])

#ifndef PACKED
  #define PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) // that's for __GNUC__
#endif


typedef enum {
    POWER_MIN       = INT8_MIN,
    POWER_m18_DBM   = -18, // 16 uW
    POWER_m10_DBM   = -10, // 100 uW
    POWER_m9_DBM    = -9, // 126 uW
    POWER_0_DBM     = 0, // 1 mW
    POWER_3_DBM     = 3, // 2 mW
    POWER_10_DBM    = 10, // 10 mW
    POWER_12_DBM    = 12, // 16 mW
    POWER_12p5_DBM  = 13, // 18 mW
    POWER_17_DBM    = 17, // 50 mW
    POWER_20_DBM    = 20, // 100 mW
    POWER_22_DBM    = 22, // 158 mW
    POWER_23_DBM    = 23, // 200 mW
    POWER_24_DBM    = 24, // 251 mW
    POWER_27_DBM    = 27, // 501 mW
    POWER_30_DBM    = 30, // 1000 mW
    POWER_33_DBM    = 33, // 2000 mW
    POWER_MAX       = INT8_MAX,
} POWER_ENUM;


typedef struct
{
    int8_t dbm;
    int16_t mW;
} rfpower_t;


typedef enum {
    ANTENNA_1 = 0,
    ANTENNA_2 = 1,
} ANTENNA_ENUM;


//-- common hardware abstraction classes

class tSerialBase
{
  public:
    virtual void InitOnce(void) {}
    virtual void Init(void) {}
    virtual void SetBaudRate(uint32_t baud) {}
    virtual void putc(char c) {}
    virtual bool available(void) { return false; }
    virtual char getc(void) { return '\0'; }
    virtual void flush(void) {}
    virtual uint16_t bytes_available(void) { return 0; }

    void putbuf(void* buf, uint16_t len) { for (uint16_t i = 0; i < len; i++) putc(((char*)buf)[i]); }
    void puts(const char* s) { while (*s) { putc(*s); s++; }; }
};


class tI2cBase
{
  public:
    virtual void Init(void) { initialized = false; }
    virtual bool put_buf_blocking(uint8_t device_adr, uint8_t* buf, uint16_t len) { return false; }

    bool initialized;
};


class tInternalDacBase
{
  public:
    virtual void Init(void) = 0;
    virtual void put_channel1(uint16_t value) {}
    virtual void put_channel2(uint16_t value) {}
    virtual void put_channel12(uint16_t value1, uint16_t value2) {}
};


//-- rssi & snr

typedef enum : int8_t {
    RSSI_INVALID      = 127,
    RSSI_MAX          = -1,
    RSSI_MIN          = -127,
} RSSI_ENUM;


typedef enum : uint8_t {
    RSSI_U7_INVALID   = 0,
    RSSI_U7_MAX       = 1, // -1
    RSSI_U7_MIN       = 127, // -127
} RSSI_U7_ENUM;


typedef enum : int8_t {
    SNR_INVALID       = 127,
} SNR_ENUM;


uint8_t rssi_u7_from_i8(int8_t rssi_i8);
int8_t rssi_i8_from_u7(uint8_t rssi_u7);
uint8_t rssi_i8_to_ap(int8_t rssi_i8);
uint8_t rssi_i8_to_mavradio(int8_t rssi_i8, bool connected);
uint16_t rssi_i8_to_ap_sbus(int8_t rssi_i8);

uint16_t lq_to_sbus_crsf(uint8_t lq);


//-- rc data

#define RC_DATA_LEN     18

#define RC_DATA_MIN     1
#define RC_DATA_CENTER  1024
#define RC_DATA_MAX     2047

typedef struct
{
    uint16_t ch[RC_DATA_LEN]; // 1 .. 1024 .. 2047 = -120% .. 120%, 11 bits
} tRcData;

// clip a value for rcData to range
uint16_t clip_rc(int32_t x);

uint16_t rc_from_sbus(uint16_t sbus_ch);
uint16_t rc_from_crsf(uint16_t crsf_ch);
uint16_t rc_to_sbus(uint16_t rc_ch);
uint16_t rc_to_crsf(uint16_t rc_ch);
uint16_t rc_to_mavlink(uint16_t rc_ch);
int16_t rc_to_mavlink_13bcentered(uint16_t rc_ch);


//-- crsf

uint8_t crsf_cvt_power(int8_t power_dbm);
uint8_t crsf_cvt_mode(uint8_t mode);
uint8_t crsf_cvt_fps(uint8_t mode);
uint8_t crsf_cvt_rssi_rx(int8_t rssi_i8);
uint8_t crsf_cvt_rssi_tx(int8_t rssi_i8);
uint8_t crsf_cvt_rssi_percent(int8_t rssi_i8, int16_t receiver_sensitivity_dbm);

uint8_t crsf_crc8_calc(uint8_t crc, uint8_t data);
uint8_t crsf_crc8_update(uint8_t crc, const void* buf, uint16_t len);


//-- bind phrase & power & version

const char bindphrase_chars[] = "abcdefghijklmnopqrstuvwxyz0123456789_#-.";

#define BINDPHRASE_CHARS_LEN  (sizeof(bindphrase_chars) - 1)  // -1 since it's a null-terminated string

bool is_valid_bindphrase_char(char c);
void sanitize_bindphrase(char* bindphrase, const char* bindphrase_default);
uint32_t u32_from_bindphrase(char* bindphrase);
uint8_t except_from_bindphrase(char* bindphrase);

void power_optstr_from_power_list(char* Power_optstr, int16_t* power_list, uint8_t num, uint8_t slen);
void power_optstr_from_rfpower_list(char* Power_optstr, const rfpower_t* rfpower_list, uint8_t num, uint8_t slen);

uint16_t version_to_u16(uint32_t version);
uint32_t version_from_u16(uint16_t version_u16);
void version_to_str(char* s, uint32_t version);


//-- display & keys

typedef enum {
    KEY_UP = 0,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_CENTER,
} KEY_ENUM;


//-- auxiliary functions

void strbufstrcpy(char* res, const char* src, uint16_t len);
void strstrbufcpy(char* res, const char* src, uint16_t len);
bool strbufeq(char* s1, const char* s2, uint16_t len);

void remove_leading_zeros(char* s);


#endif // COMMON_TYPES_H
