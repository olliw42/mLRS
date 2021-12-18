//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// M Bridge
//********************************************************
#ifndef MBRIDGE_H
#define MMBRIDGE_H
#pragma once

#include <inttypes.h>


#ifndef PACKED
  #define MBRDIGE_PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) // that's for __GNUC__
#else
  #define MBRDIGE_PACKED  PACKED
#endif

#define MBRIDGE_STX1                        'O'
#define MBRIDGE_STX2                        'W'

#define MBRIDGE_SERIALPACKET_RX_SIZE_MAX    17 // up to 17 bytes payload when received from transmitter
#define MBRIDGE_SERIALPACKET_TX_SIZE_MAX    16 // up to 16 bytes payload when send to transmitter
#define MBRIDGE_SERIALPACKET_TX_SIZE_LIM    11 // only up to 11 bytes if a channel or command packet was received
#define MBRIDGE_SERIALPACKET_STX            0x00

#define MBRIDGE_CHANNELPACKET_SIZE          22 // 22 bytes payload, only received from transmitter
#define MBRIDGE_CHANNELPACKET_STX           0xFF

#define MBRIDGE_COMMANDPACKET_RX_SIZE       22 // 22 bytes payload when received from transmitter
#define MBRIDGE_COMMANDPACKET_TX_SIZE       12 // 12 bytes payload when send to transmitter
#define MBRIDGE_COMMANDPACKET_STX           0xA0 // 0b101x
#define MBRIDGE_COMMANDPACKET_STX_MASK      0xE0

#define MBRIDGE_TMO_US                      50
#define MBRIDGE_RX2TX_DELAY_US              50


// do not confuse with sbus, it is similar to sbus packet format, but not sbus values
typedef union {
  uint8_t c[MBRIDGE_CHANNELPACKET_SIZE]; // 154 + 20 + 2 = 176 bits = 22 bytes
  MBRDIGE_PACKED(
  struct {
      uint16_t ch0  : 11; // 14 channels a 11 bits per channel = 154 bits, 0 .. 1024 .. 2047
      uint16_t ch1  : 11;
      uint16_t ch2  : 11;
      uint16_t ch3  : 11;
      uint16_t ch4  : 11;
      uint16_t ch5  : 11;
      uint16_t ch6  : 11;
      uint16_t ch7  : 11;
      uint16_t ch8  : 11;
      uint16_t ch9  : 11;
      uint16_t ch10 : 11;
      uint16_t ch11 : 11;
      uint16_t ch12 : 11;
      uint16_t ch13 : 11;
      uint16_t ch14 : 10; // 2 channels a 10 bits per channel = 20 bits, 0 .. 512 .. 1023
      uint16_t ch15 : 10;
      uint16_t ch16 : 1; // 2 channels a 1 bits per channel = 2 bits, 0..1
      uint16_t ch17 : 1;
  });
} tMBridgeChannelBuffer;


class tMBridgeBase
{
  public:

    typedef enum {
      TXSTATE_IDLE = 0,
      TXSTATE_RECEIVE_STX2,
      TXSTATE_RECEIVE_LEN,
      TXSTATE_RECEIVE_SERIALPACKET,
      TXSTATE_RECEIVE_CHANNELPACKET,
      TXSTATE_RECEIVE_COMMANDPACKET,
      TXSTATE_TRANSMIT_START,
      TXSTATE_TRANSMIT_PUTCHARS, // used only in polling mode
      TXSTATE_TRANSMIT_CLOSE, // used only in polling mode
    } TXSTATE_ENUM;

    typedef enum {
      TXTYPE_NONE = 0,
      TXTYPE_SERIALPACKET,
      TXTYPE_CHANNELPACKET,
      TXTYPE_COMMANDPACKET,
    } TXTYPE_ENUM;;

    uint8_t tx_state;
    uint8_t tx_type;
    uint8_t tx_len;
    uint8_t tx_cnt;
    uint16_t tlast_us;

    tMBridgeChannelBuffer channels;
    uint8_t channels_updated;

    uint8_t cmd_receive_cmd;
    uint8_t cmd_receive_buf[MBRIDGE_COMMANDPACKET_RX_SIZE];
    uint8_t cmd_received;

    uint8_t cmd_send_cmd;
    uint8_t cmd_send_buf[MBRIDGE_COMMANDPACKET_TX_SIZE];
    uint8_t cmd_send_available;

    // we need a timer with us resolution
    virtual uint16_t tim_us(void); // do not name it micros(), and give it an unusual name

    // interface to the uart hardware peripheral used for the bridge
    virtual void transmit_enable(bool flag);

    virtual bool mb_rx_available(void);
    virtual char mb_getc(void);
    virtual void mb_putc(char c);

    // mimics a serial interface to the main code, usually two fifo
    virtual bool serial_rx_available(void);
    virtual char serial_getc(void);
    virtual void serial_putc(char c);

    // send/receive commands interface to the main code, usually two fifo, can be empty
    bool cmd_from_transmitter(uint8_t* cmd, uint8_t* payload);
    void cmd_to_transmitter(uint8_t cmd, uint8_t* payload, uint8_t len);

    // init
    void Init(void);

    // for in-loop processing
    void SpinOnce(void);

    // for in-isr processing
    void parse_nextchar(uint8_t c);
    void transmit_start(void);

    // auxiliary helper
    uint8_t transmit_serial(void);
    uint8_t transmit_cmd(void);
    void transmit_close(void);
};


#endif // MBRIDGE_H

