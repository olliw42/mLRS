//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// IN
//********************************************************


#include <string.h>
#include "in.h"


typedef enum {
  IN_STATE_IDLE = 0,
  IN_STATE_RECEIVING,
} IN_STATE_ENUM;


void InBase::Init(void)
{
  _config = UINT8_MAX;

  _t_last_us = 0;
  _state = IN_STATE_IDLE;
}


void InBase::Configure(uint8_t new_config)
{
  if (new_config == _config) return;
  _config = new_config;

  switch (_config) {
  case IN_CONFIG_SBUS:
      config_sbus(false);
      break;
  case IN_CONFIG_SBUS_INVERTED:
      config_sbus(true);
      break;
  }
}


bool InBase::Update(tRcData* rc)
{
  switch (_config) {
  case IN_CONFIG_SBUS:
  case IN_CONFIG_SBUS_INVERTED:
      return parse_sbus(rc);
  }

  return false;
}


//-------------------------------------------------------
// SBus
//-------------------------------------------------------
// see also CommonRx out.cpp

#define SBUS_CHANNELPACKET_SIZE      22

typedef union {
  uint8_t c[SBUS_CHANNELPACKET_SIZE];
  PACKED(
  struct {
    uint16_t ch0  : 11; // 11 bits per channel * 16 channels = 22 bytes
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
    uint16_t ch14 : 11;
    uint16_t ch15 : 11;
  });
} tSBusFrameBuffer;


bool InBase::parse_sbus(tRcData* rc)
{
  uint16_t t_now_us = tim_1us();
  bool updated = false;

  while (available()) {
    char c = getc();

    if (_state == IN_STATE_IDLE) { // scan for new frame
      if (c == 0x0F) { // new frame
        _buf_pos = 0;
        _state = IN_STATE_RECEIVING;
      }
    }

    if (_state == IN_STATE_RECEIVING) {
      _buf[_buf_pos] = c;
      _buf_pos++;
      if (_buf_pos >= 25) {
        get_sbus_data(rc);
        updated = true;
        _state = IN_STATE_IDLE;
        break; // is this what we want, or shouldn't we catch all?
      }
    }

    _t_last_us = t_now_us;
  }

  if (_state == IN_STATE_RECEIVING) {
    if ((t_now_us - _t_last_us) > 2500) _state = IN_STATE_IDLE;
  }

  return updated;
}


void InBase::get_sbus_data(tRcData* rc)
{
tSBusFrameBuffer sbus_buf;

  memcpy(sbus_buf.c, &(_buf[1]), SBUS_CHANNELPACKET_SIZE);

  // OpenTx produces 173 ... 992 ... 1811 for -100% ... 100%

  rc->ch[0] = (((int32_t)(sbus_buf.ch0) - 992) * 2047) / 1638 + 1024;
  rc->ch[1] = (((int32_t)(sbus_buf.ch1) - 992) * 2047) / 1638 + 1024;
  rc->ch[2] = (((int32_t)(sbus_buf.ch2) - 992) * 2047) / 1638 + 1024;
  rc->ch[3] = (((int32_t)(sbus_buf.ch3) - 992) * 2047) / 1638 + 1024;
  rc->ch[4] = (((int32_t)(sbus_buf.ch4) - 992) * 2047) / 1638 + 1024;
  rc->ch[5] = (((int32_t)(sbus_buf.ch5) - 992) * 2047) / 1638 + 1024;
  rc->ch[6] = (((int32_t)(sbus_buf.ch6) - 992) * 2047) / 1638 + 1024;
  rc->ch[7] = (((int32_t)(sbus_buf.ch7) - 992) * 2047) / 1638 + 1024;
  rc->ch[8] = (((int32_t)(sbus_buf.ch8) - 992) * 2047) / 1638 + 1024;
  rc->ch[9] = (((int32_t)(sbus_buf.ch9) - 992) * 2047) / 1638 + 1024;

  rc->ch[10] = (((int32_t)(sbus_buf.ch10) - 992) * 2047) / 1638 + 1024;
  rc->ch[11] = (((int32_t)(sbus_buf.ch11) - 992) * 2047) / 1638 + 1024;
  rc->ch[12] = (((int32_t)(sbus_buf.ch12) - 992) * 2047) / 1638 + 1024;
  rc->ch[13] = (((int32_t)(sbus_buf.ch13) - 992) * 2047) / 1638 + 1024;
  rc->ch[14] = (((int32_t)(sbus_buf.ch14) - 992) * 2047) / 1638 + 1024;
  rc->ch[15] = (((int32_t)(sbus_buf.ch15) - 992) * 2047) / 1638 + 1024;

  rc->ch[16] = 1024;
  rc->ch[17] = 1024;
}



//-------------------------------------------------------
// FPort
//-------------------------------------------------------

// FPort, https://github.com/betaflight/betaflight/files/1491056/F.Port.protocol.betaFlight.V2.1.2017.11.21.pdf
// FPort frame: 0x7E , len = 0x19, type = 0x0,  22 bytes channel data , flags byte, rssi byte, crc byte, 0x7E
// 115200 bps, 8N1
// byte stuffing: 0x7E -> 0x7D,0x5E, 0x7D -> 0x7D,0x5D


//-------------------------------------------------------
// IBus
//-------------------------------------------------------

//IBUS frame: 0x20, 0x40, 28 bytes for 14 channels, 2 byte checksum
// 115200 bps, 8N1
// center = 0x05DC. My transmitter sends values between 0x3E8 and 0x7D0


//-------------------------------------------------------
// SUMD
//-------------------------------------------------------

//SUMD, https://www.deviationtx.com/media/kunena/attachments/98/HoTT-SUMD-Spec-REV01-12062012-pdf.pdf
// 115200 bps, 8N1
// 0xA8, 0x01 or 0x00 or 0x81, len byte, u16 x channels bytes, 2 crc byte, telem, crc8



