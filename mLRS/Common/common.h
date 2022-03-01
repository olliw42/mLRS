//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// common
//*******************************************************
#ifndef COMMON_H
#define COMMON_H
#pragma once


#include "common_conf.h"
#include "sx-drivers\sx12xx_driver.h"
#include "lq_counter.h"
#include "fhss.h"
#include "frame_types.h"
#include "common_stats.h"


//-------------------------------------------------------
// SysTask
//-------------------------------------------------------

volatile uint8_t doSysTask = 0;


void HAL_IncTick(void)
{
    uwTick += uwTickFreq;
    doSysTask = 1;
}


uint32_t millis32(void)
{
    return uwTick;
}


//-------------------------------------------------------
// Frames
//-------------------------------------------------------

STATIC_ASSERT(sizeof(tFrameStatus) == FRAME_HEADER_LEN - 2, "Frame header len missmatch")
STATIC_ASSERT(sizeof(tTxFrame) == FRAME_TX_RX_LEN, "Frame len missmatch TxFrame")
STATIC_ASSERT(sizeof(tRxFrame) == FRAME_TX_RX_LEN, "Frame len missmatch RxFrame")


typedef enum {
    CHECK_OK = 0,
    CHECK_ERROR_SYNCWORD, // 1
    CHECK_ERROR_HEADER,   // 2
    CHECK_ERROR_CRC1,     // 3
    CHECK_ERROR_CRC,      // 4
} CHECK_ENUM;


void pack_tx_frame(tTxFrame* frame, tFrameStats* frame_stats, tRcData* rc, uint8_t* payload, uint8_t payload_len)
{
uint16_t crc;

    if (payload_len > FRAME_TX_PAYLOAD_LEN) payload_len = FRAME_TX_PAYLOAD_LEN; // should never occur, but play it safe

    memset(frame, 0, sizeof(tTxFrame));

    // generate header
    frame->sync_word = Config.FrameSyncWord;
    frame->status.seq_no = frame_stats->seq_no;
    frame->status.ack = frame_stats->ack;
    frame->status.frame_type = FRAME_TYPE_TX;
    frame->status.antenna = frame_stats->antenna;
    frame->status.transmit_antenna = frame_stats->transmit_antenna;
    frame->status.rssi_u7 = rssi_u7_from_i8(frame_stats->rssi);
    frame->status.LQ = frame_stats->LQ;
    frame->status.LQ_serial_data = frame_stats->LQ_serial_data;
    frame->status.payload_len = payload_len;

    // pack rc data
    // rcData: 0 .. 1024 .. 2047, 11 bits
    frame->rc1.ch0  = rc->ch[0]; // 0 .. 1024 .. 2047, 11 bits
    frame->rc1.ch1  = rc->ch[1];
    frame->rc1.ch2  = rc->ch[2];
    frame->rc1.ch3  = rc->ch[3];

    frame->rc2.ch4  = rc->ch[4]; // 0 .. 1024 .. 2047, 11 bits
    frame->rc2.ch5  = rc->ch[5];
    frame->rc2.ch6  = rc->ch[6];
    frame->rc2.ch7  = rc->ch[7];

    frame->rc2.ch8  = rc->ch[8] / 8; // 0 .. 128 .. 255, 8 bits
    frame->rc2.ch9  = rc->ch[9] / 8;
    frame->rc2.ch10 = rc->ch[10] / 8;
    frame->rc2.ch11 = rc->ch[11] / 8;

    frame->rc1.ch12 = (rc->ch[12] >= 1536) ? 2 : ((rc->ch[12] <= 512) ? 0 : 1); // 0 .. 1 .. 2, bits, 3-way
    frame->rc1.ch13 = (rc->ch[13] >= 1536) ? 2 : ((rc->ch[13] <= 512) ? 0 : 1);
    frame->rc2.ch14 = (rc->ch[14] >= 1536) ? 2 : ((rc->ch[14] <= 512) ? 0 : 1);
    frame->rc2.ch15 = (rc->ch[15] >= 1536) ? 2 : ((rc->ch[15] <= 512) ? 0 : 1);

    // pack the payload
    for (uint8_t i = 0; i < payload_len; i++) {
      frame->payload[i] = payload[i];
    }

    // finalize, crc
    fmav_crc_init(&crc);
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_HEADER_LEN + FRAME_TX_RCDATA1_LEN);
    frame->crc1 = crc;

    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame + FRAME_HEADER_LEN + FRAME_TX_RCDATA1_LEN, FRAME_TX_RX_LEN - FRAME_HEADER_LEN - FRAME_TX_RCDATA1_LEN - 2);
    frame->crc = crc;
}


// returns 0 if OK !!
uint8_t check_tx_frame(tTxFrame* frame)
{
uint16_t crc;

    if (frame->sync_word != Config.FrameSyncWord) return CHECK_ERROR_SYNCWORD;
    if (frame->status.frame_type != FRAME_TYPE_TX) return CHECK_ERROR_HEADER;
    if (frame->status.payload_len > FRAME_TX_PAYLOAD_LEN) return CHECK_ERROR_HEADER;

    fmav_crc_init(&crc);
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_HEADER_LEN + FRAME_TX_RCDATA1_LEN);
    if (crc != frame->crc1) return CHECK_ERROR_CRC1;

    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame + FRAME_HEADER_LEN + FRAME_TX_RCDATA1_LEN, FRAME_TX_RX_LEN - FRAME_HEADER_LEN - FRAME_TX_RCDATA1_LEN - 2);
    if (crc != frame->crc) return CHECK_ERROR_CRC;

    return CHECK_OK;
}


void rcdata_rc1_from_txframe(tRcData* rc, tTxFrame* frame)
{
    rc->ch[0] = frame->rc1.ch0;
    rc->ch[1] = frame->rc1.ch1;
    rc->ch[2] = frame->rc1.ch2;
    rc->ch[3] = frame->rc1.ch3;

    rc->ch[12] = (frame->rc1.ch12 > 1) ? 2047 : ((frame->rc1.ch12 < 1) ? 0 : 1024);
    rc->ch[13] = (frame->rc1.ch13 > 1) ? 2047 : ((frame->rc1.ch13 < 1) ? 0 : 1024);
}


void rcdata_from_txframe(tRcData* rc, tTxFrame* frame)
{
    rc->ch[0] = frame->rc1.ch0;
    rc->ch[1] = frame->rc1.ch1;
    rc->ch[2] = frame->rc1.ch2;
    rc->ch[3] = frame->rc1.ch3;

    rc->ch[4] = frame->rc2.ch4;
    rc->ch[5] = frame->rc2.ch5;
    rc->ch[6] = frame->rc2.ch6;
    rc->ch[7] = frame->rc2.ch7;

    rc->ch[8] = frame->rc2.ch8 * 8;
    rc->ch[9] = frame->rc2.ch9 * 8;
    rc->ch[10] = frame->rc2.ch10 * 8;
    rc->ch[11] = frame->rc2.ch11 * 8;

    rc->ch[12] = (frame->rc1.ch12 > 1) ? 2047 : ((frame->rc1.ch12 < 1) ? 0 : 1024);
    rc->ch[13] = (frame->rc1.ch13 > 1) ? 2047 : ((frame->rc1.ch13 < 1) ? 0 : 1024);
    rc->ch[14] = (frame->rc2.ch14 > 1) ? 2047 : ((frame->rc2.ch14 < 1) ? 0 : 1024);
    rc->ch[15] = (frame->rc2.ch15 > 1) ? 2047 : ((frame->rc2.ch15 < 1) ? 0 : 1024);

    rc->ch[16] = 1024;
    rc->ch[17] = 1024;
}


void pack_rx_frame(tRxFrame* frame, tFrameStats* frame_stats, uint8_t* payload, uint8_t payload_len)
{
uint16_t crc;

    if (payload_len > FRAME_RX_PAYLOAD_LEN) payload_len = FRAME_RX_PAYLOAD_LEN; // should never occur, but play it safe

    memset(frame, 0, sizeof(tRxFrame));

    frame->sync_word = Config.FrameSyncWord;
    frame->status.seq_no = frame_stats->seq_no;
    frame->status.ack = frame_stats->ack;
    frame->status.frame_type = FRAME_TYPE_RX;
    frame->status.antenna = frame_stats->antenna;
    frame->status.transmit_antenna = frame_stats->transmit_antenna;
    frame->status.rssi_u7 = rssi_u7_from_i8(frame_stats->rssi);
    frame->status.LQ = frame_stats->LQ;
    frame->status.LQ_serial_data = frame_stats->LQ_serial_data;
    frame->status.payload_len = payload_len;

    for (uint8_t i = 0; i < payload_len; i++) {
      frame->payload[i] = payload[i];
    }

    fmav_crc_init(&crc);
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_TX_RX_LEN - 2);
    frame->crc = crc;
}


// returns 0 if OK !!
uint8_t check_rx_frame(tRxFrame* frame)
{
uint16_t crc;

    if (frame->sync_word != Config.FrameSyncWord) return CHECK_ERROR_SYNCWORD;

    if (frame->status.frame_type != FRAME_TYPE_RX) return CHECK_ERROR_HEADER;
    if (frame->status.payload_len > FRAME_RX_PAYLOAD_LEN) return CHECK_ERROR_HEADER;

    fmav_crc_init(&crc);
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_TX_RX_LEN - 2);
    if (crc != frame->crc) return CHECK_ERROR_CRC;

    return CHECK_OK;
}


//-------------------------------------------------------
// Serial Classes
//-------------------------------------------------------

// this is the serial port, is always uartb
class tSerialPort : public tSerialBase
{
#ifndef DEVICE_HAS_NO_SERIAL
  public:
    void Init(void) override { uartb_init(); }
    void putc(char c) override { uartb_putc(c); }
    bool available(void) override { return uartb_rx_available(); }
    char getc(void) override { return uartb_getc(); }
    void flush(void) override { uartb_rx_flush(); uartb_tx_flush(); }
    uint16_t bytes_available(void) override { return uartb_rx_bytesavailable(); }
    const uint16_t rx_buf_size(void) override { return UARTB_RXBUFSIZE; }
    bool tx_is_empty(void) override { return uartb_tx_isempty(); }

#endif
};


// is always uartc
class tDebugPort : public tSerialBase
{
#if (!defined DEVICE_HAS_NO_DEBUG && defined DEBUG_ENABLED)
  public:
    void Init(void) { uartc_init(); }
    void putc(char c) override { uartc_putc(c); }
#endif
};


//-------------------------------------------------------
// Common Variables
//-------------------------------------------------------

tSerialPort serial;
tDebugPort dbg;

tRcData rcData;

tTxFrame txFrame;
tRxFrame rxFrame;

tTxFrame txFrame2;
tRxFrame rxFrame2;

#ifdef DEVICE_HAS_SX126x
Sx126xDriver sx;
#elif defined DEVICE_HAS_SX127x
Sx127xDriver sx;
#else
Sx128xDriver sx;
#endif

#ifdef DEVICE_HAS_DIVERSITY
#ifdef DEVICE_HAS_SX126x
Sx126xDriver2 sx2;
#elif defined DEVICE_HAS_SX127x
Sx127xDriver2 sx2;
#else
Sx128xDriver2 sx2;
#endif
#else
SxDriverDummy sx2;
#endif

Stats stats;

FhssBase fhss;



#endif // COMMON_H
