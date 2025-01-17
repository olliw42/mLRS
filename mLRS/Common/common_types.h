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
#include <string.h>


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
    virtual bool full(void) { return false; }
    virtual void putbuf(uint8_t* const buf, uint16_t len) {}
    virtual bool available(void) { return false; }
    virtual char getc(void) { return '\0'; }
    virtual void flush(void) {}
    virtual uint16_t bytes_available(void) { return 0; }
    virtual bool has_systemboot(void) { return false; }

    void putc(char c) { putbuf((uint8_t*)&c, 1); }
    void puts(const char* const s) { putbuf((uint8_t*)s, strlen(s)); }
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
uint16_t rssi_i8_to_rc(int8_t rssi_i8);

uint16_t lq_to_rc(uint8_t lq);


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


//-- CRSF

uint8_t crsf_cvt_power(int8_t power_dbm);
uint8_t crsf_cvt_mode(uint8_t mode);
uint8_t crsf_cvt_fps(uint8_t mode);
uint8_t crsf_cvt_rssi_rx(int8_t rssi_i8);
uint8_t crsf_cvt_rssi_tx(int8_t rssi_i8);
uint8_t crsf_cvt_rssi_percent(int8_t rssi, int16_t receiver_sensitivity_dbm);

#define CRSF_CRC8_INIT  0
uint8_t crsf_crc8_calc(uint8_t crc, uint8_t data);
uint8_t crsf_crc8_update(uint8_t crc, const void* buf, uint16_t len);


//-- DroneCAN

uint8_t dronecan_cvt_power(int8_t power_dbm);
uint16_t cvt_power(int8_t power_dbm);


//-- modes and so on

void frequency_band_str_to_strbuf(char* const s, uint8_t frequency_band, uint8_t len);
void mode_str_to_strbuf(char* const s, uint8_t mode, uint8_t len);


//-- bind phrase & power & version

const char bindphrase_chars[] = "abcdefghijklmnopqrstuvwxyz0123456789_#-.";

#define BINDPHRASE_CHARS_LEN  (sizeof(bindphrase_chars) - 1)  // -1 since it's a null-terminated string

bool is_valid_bindphrase_char(char c);
void sanitize_bindphrase(char* const bindphrase, const char* const bindphrase_default);
uint32_t u32_from_bindphrase(char* const bindphrase);
uint8_t except_from_bindphrase(char* const bindphrase);
void bindphrase_from_u32(char* const bindphrase, uint32_t bindphrase_u32);

void power_optstr_from_power_list(char* const Power_optstr, int16_t* const power_list, uint8_t num, uint8_t slen);
void power_optstr_from_rfpower_list(char* const Power_optstr, const rfpower_t* const rfpower_list, uint8_t num, uint8_t slen);

uint16_t version_to_u16(uint32_t version);
uint32_t version_from_u16(uint16_t version_u16);
void version_to_str(char* const s, uint32_t version);


//-- tx tasks

typedef enum {
    TX_TASK_NONE = 0,
    TX_TASK_RX_PARAM_SET,
    TX_TASK_PARAM_STORE,
    TX_TASK_BIND,
    TX_TASK_PARAM_RELOAD,
    TX_TASK_SYSTEM_BOOT,
    TX_TASK_RESTART_CONTROLLER,
    TX_TASK_FLASH_ESP,
    TX_TASK_ESP_PASSTHROUGH,
    TX_TASK_CLI_CHANGE_CONFIG_ID,
    TX_TASK_HC04_PASSTHROUGH,
    TX_TASK_CLI_HC04_SETPIN,
} TX_TASK_ENUM;


//-- display & keys

typedef enum {
    KEY_UP = 0,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_CENTER,
} KEY_ENUM;


//-- auxiliary functions

void strbufstrcpy(char* const res, const char* const src, uint16_t len);
void strstrbufcpy(char* const res, const char* const src, uint16_t len);
bool strbufeq(char* const s1, const char* const s2, uint16_t len);

void remove_leading_zeros(char* const s);


#endif // COMMON_TYPES_H
