//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// OUT
//********************************************************


#include <string.h>
#include "out.h"


void OutBase::Init(void)
{
    _config = UINT8_MAX;
    _tx_channel_order = UINT8_MAX;
    _rx_channel_order = UINT8_MAX;
    for (uint8_t n = 0; n < 4; n++) _channel_map[n] = n;
}


void OutBase::Configure(uint8_t new_config)
{
    if (new_config == _config) return;
    _config = new_config;

    switch (_config) {
    case OUT_CONFIG_SBUS:
        config_sbus();
        break;
    }
}


void OutBase::SetChannelOrder(uint8_t tx_channel_order, uint8_t rx_channel_order)
{
  _tx_channel_order = tx_channel_order;
  _rx_channel_order = rx_channel_order;

  switch (_tx_channel_order) {
    case CHANNEL_ORDER_AETR:
      // nothing to do
      break;
    case CHANNEL_ORDER_TAER:
      // TODO
      break;
    case CHANNEL_ORDER_ETAR:
      _channel_map[0] = 2;
      _channel_map[1] = 0;
      _channel_map[2] = 1;
      _channel_map[3] = 3;
      break;
  }
}


void OutBase::SendRcData(tRcData* rc, bool frame_lost, bool failsafe)
{
    uint16_t ch[4] = { rc->ch[0], rc->ch[1], rc->ch[2], rc->ch[3] };
    for (uint8_t n = 0; n < 4; n++) {
      rc->ch[n] = ch[_channel_map[n]];
    }

    // mimic spektrum
    // 1090 ... 1515  ... 1940
    // => x' = (1090-1000) * 2048/1000 + 850/1000 * x
    uint32_t t = 90*2048;
    for (uint8_t n = 0; n < RC_DATE_LEN; n++) {
      uint32_t xs = 850 * rc->ch[n];
      rc->ch[n] = (xs + t) / 1000;
    }

    switch (_config) {
    case OUT_CONFIG_SBUS:
        send_sbus_rcdata(rc, frame_lost, failsafe);
        break;
    }
}


void OutBase::putbuf(uint8_t* buf, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++) putc(buf[i]);
}


//-------------------------------------------------------
// SBus
//-------------------------------------------------------

// SBus frame: 0x0F , 22 bytes channel data , flags byte, 0x00
// 100000 bps, 8E2
// send every 14 ms (normal) or 7 ms (high speed), 10ms or 20ms
// 92 - 1792 => 1000 - 2000

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


typedef enum {
  SBUS_FLAG_CH17 = 0x01,
  SBUS_FLAG_CH18 = 0x02,
  SBUS_FLAG_FRAME_LOST = 0x04,
  SBUS_FLAG_FAILSAFE = 0x08,
} SBUS_FLAG_ENUM;


// rc data in TxFrame is centered such to cover 0..2047, 0..255, 0..1
// ardupilot: sbus 200 -> 1000us, sbus 1800 -> 2000us
// txFrame:   ch0-ch3:    0 .. 1024 .. 2047, 11 bits
//            ch4-ch13:   0 .. 128 .. 255, 8 bits
//            ch14-ch17:  0..1, 1 bit

// let's convert the full range to +-100% or 200..1000..1800:
// sbus =  ch * 1600 / 2048 + 200 for 11 bits
//         ch * 1600 / 256 + 200 for 8 bits

// we also could convert to OpenTx range +-100% = 988us ... 2012 us
// sbus =  ch * 1600 / 2048 + 200 for 11 bits
//         ch * 1600 / 256 + 200 for 8 bits

// Ardupilot:
// translates sbus values 200..1000..1800 into pwm values 1000..1500..2000 us
// => pwm = sbus * 500 / 800 + 875
//    sbus = (pwm - 875) * 800 / 500 = pwm * 800 / 500 - 1400
// thus, if we scale sbus to 200..1800 we get 1000..2000 us on ardupilot
//
// when selecting SBUS external module and connecting JRpin1 to ArduPilot, we get
// 983 ... 1495 .. 2006

// opentx: sbus value = ch value * 8 / 10 + 992, where ch value = -1024...1023
// => sbus values = 173..992..1811

/*
void OutBase::send_sbus_frame(tTxFrame* frame)
{
tSBusFrameBuffer sbus_buf;

  memset(&sbus_buf, 0, sizeof(tSBusFrameBuffer));

  sbus_buf.ch0 = ((uint32_t)(frame->rc1.ch0) * 1600) / 2047 + 200;
  sbus_buf.ch1 = ((uint32_t)(frame->rc1.ch1) * 1600) / 2047 + 200;
  sbus_buf.ch2 = ((uint32_t)(frame->rc1.ch2) * 1600) / 2047 + 200;
  sbus_buf.ch3 = ((uint32_t)(frame->rc1.ch3) * 1600) / 2047 + 200;

  sbus_buf.ch4 = ((uint32_t)(frame->rc2.ch[0]) * 1600) / 255 + 200;
  sbus_buf.ch5 = ((uint32_t)(frame->rc2.ch[1]) * 1600) / 255 + 200;
  sbus_buf.ch6 = ((uint32_t)(frame->rc2.ch[2]) * 1600) / 255 + 200;
  sbus_buf.ch7 = ((uint32_t)(frame->rc2.ch[3]) * 1600) / 255 + 200;
  sbus_buf.ch8 = ((uint32_t)(frame->rc2.ch[4]) * 1600) / 255 + 200;
  sbus_buf.ch9 = ((uint32_t)(frame->rc2.ch[5]) * 1600) / 255 + 200;
  sbus_buf.ch10 = ((uint32_t)(frame->rc2.ch[6]) * 1600) / 255 + 200;
  sbus_buf.ch11 = ((uint32_t)(frame->rc2.ch[7]) * 1600) / 255 + 200;
  sbus_buf.ch12 = ((uint32_t)(frame->rc2.ch[8]) * 1600) / 255 + 200;
  sbus_buf.ch13 = ((uint32_t)(frame->rc2.ch[9]) * 1600) / 255 + 200;

  sbus_buf.ch14 = (frame->rc1.ch14) ? 1800 : 200;
  sbus_buf.ch15 = (frame->rc1.ch15) ? 1800 : 200;

  uint8_t flags = 0;
  if (frame->rc1.ch16) flags |= SBUS_FLAG_CH17;
  if (frame->rc1.ch17) flags |= SBUS_FLAG_CH18;

  putc(0x0F);
  putbuf(sbus_buf.c, SBUS_CHANNELPACKET_SIZE);
  putc(flags);
  putc(0x00);
}
*/

void OutBase::send_sbus_rcdata(tRcData* rc, bool frame_lost, bool failsafe)
{
tSBusFrameBuffer sbus_buf;

  sbus_buf.ch0 = ((uint32_t)(rc->ch[0]) * 1600) / 2047 + 200;
  sbus_buf.ch1 = ((uint32_t)(rc->ch[1]) * 1600) / 2047 + 200;
  sbus_buf.ch2 = ((uint32_t)(rc->ch[2]) * 1600) / 2047 + 200;
  sbus_buf.ch3 = ((uint32_t)(rc->ch[3]) * 1600) / 2047 + 200;
  sbus_buf.ch4 = ((uint32_t)(rc->ch[4]) * 1600) / 2047 + 200;
  sbus_buf.ch5 = ((uint32_t)(rc->ch[5]) * 1600) / 2047 + 200;
  sbus_buf.ch6 = ((uint32_t)(rc->ch[6]) * 1600) / 2047 + 200;
  sbus_buf.ch7 = ((uint32_t)(rc->ch[7]) * 1600) / 2047 + 200;
  sbus_buf.ch8 = ((uint32_t)(rc->ch[8]) * 1600) / 2047 + 200;
  sbus_buf.ch9 = ((uint32_t)(rc->ch[9]) * 1600) / 2047 + 200;
  sbus_buf.ch10 = ((uint32_t)(rc->ch[10]) * 1600) / 2047 + 200;
  sbus_buf.ch11 = ((uint32_t)(rc->ch[11]) * 1600) / 2047 + 200;
  sbus_buf.ch12 = ((uint32_t)(rc->ch[12]) * 1600) / 2047 + 200;
  sbus_buf.ch13 = ((uint32_t)(rc->ch[13]) * 1600) / 2047 + 200;
  sbus_buf.ch14 = ((uint32_t)(rc->ch[14]) * 1600) / 2047 + 200;
  sbus_buf.ch15 = ((uint32_t)(rc->ch[15]) * 1600) / 2047 + 200;

  uint8_t flags = 0;
  if (rc->ch[16] >= 1536) flags |= SBUS_FLAG_CH17;
  if (rc->ch[16] >= 1536) flags |= SBUS_FLAG_CH18;
  if (frame_lost) flags |= SBUS_FLAG_FRAME_LOST;
  if (failsafe) flags |= SBUS_FLAG_FAILSAFE;

  putc(0x0F);
  putbuf(sbus_buf.c, SBUS_CHANNELPACKET_SIZE);
  putc(flags);
  putc(0x00);
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



