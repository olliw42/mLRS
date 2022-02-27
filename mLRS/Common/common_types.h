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


#ifndef PACKED
#  define PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) //that's for __GNUC__
#endif


typedef enum {
    POWER_MIN       = INT8_MIN,
    POWER_m18_DBM   = -18, // 16 uW
    POWER_m10_DBM   = -10, // 100 uW
    POWER_0_DBM     = 0, // 1 mW
    POWER_10_DBM    = 10, // 10 mW
    POWER_12_DBM    = 12, // 16 mW
    POWER_12p5_DBM  = 13, // 18 mW
    POWER_17_DBM    = 17, // 50 mW
    POWER_20_DBM    = 20, // 100 mW
    POWER_22_DBM    = 22, // 159 mW
    POWER_23_DBM    = 24, // 200 mW
    POWER_24_DBM    = 24, // 251 mW
    POWER_27_DBM    = 27, // 501 mW
    POWER_30_DBM    = 30, // 1000 mW
    POWER_MAX       = INT8_MAX,
} POWER_ENUM;


typedef struct {
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
    virtual void Init(void) {};
    virtual void putc(char c) {}
    virtual bool available(void) { return 0; }
    virtual char getc(void) { return '\0'; }
    virtual void flush(void) {};
    virtual uint16_t bytes_available(void) { return 0; }
    virtual const uint16_t rx_buf_size(void) { return 1; }

    void putbuf(void* buf, uint16_t len) { for (uint16_t i = 0; i < len; i++) putc(((char*)buf)[i]); }
    void puts(const char* s) { while (*s) { putc(*s); s++; }; }

    uint16_t rx_free(void) { return rx_buf_size() - bytes_available(); }
    uint8_t rx_free_percent(void) { return (100 * rx_free() + rx_buf_size()/2) / rx_buf_size(); }
};


class tI2cBase
{
  public:
    virtual void Init(void) { initialized = false; }
    virtual bool put_buf_blocking(uint8_t device_adr, uint8_t* buf, uint16_t len) { return false; }

    bool initialized;
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


//-- auxiliary functions

// clip a value for rcData to range

uint16_t clip_rc(int32_t x);


#endif // COMMON_TYPES_H
