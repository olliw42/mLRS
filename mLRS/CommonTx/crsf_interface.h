//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// CRSF Interface Header
//********************************************************
#ifndef CRSF_INTERFACE_H
#define CRSF_INTERFACE_H
#pragma once


#if (defined USE_CRSF) && (defined DEVICE_HAS_JRPIN5)

#include "..\Common\thirdparty.h"
#include "jr_pin5_interface.h"
#include "..\Common\crsf_protocol.h"


uint16_t micros(void);


//-------------------------------------------------------
// Interface Implementation

class tTxCrsf : public tPin5BridgeBase
{
  public:
    void Init(void);
    bool IsEmpty(void);
    bool IsChannelData(void);
    void SendLinkStatistics(tCrsfLinkStatistics* payload); // in OpenTx this triggers telemetryStreaming
    void SendLinkStatisticsTx(tCrsfLinkStatisticsTx* payload);
    void SendLinkStatisticsRx(tCrsfLinkStatisticsRx* payload);

    // for in-isr processing
    void parse_nextchar(uint8_t c, uint16_t tnow_us) override;
    bool transmit_start(void) override; // returns true if transmission should be started

    uint8_t frame[128];

    volatile bool frame_received;

    volatile uint8_t tx_available; // this signals if something needs to be send to radio
    uint8_t tx_frame[128];

    uint8_t crc8(const uint8_t* buf);
};

tTxCrsf crsf;


// we do not add a delay here before we transmit
// the logic analyzer shows this gives a 30-35 us gap nevertheless, which is perfect

void uart_rx_callback(uint8_t c)
{
  LED_RIGHT_GREEN_ON;

  if (crsf.state >= tPin5BridgeBase::STATE_TRANSMIT_START) { // recover in case something went wrong
      crsf.state = tPin5BridgeBase::STATE_IDLE;
  }

  uint16_t tnow_us = micros();
  crsf.parse_nextchar(c, tnow_us);

  if (crsf.transmit_start()) { // check if a transmission waits, put it into buf and return true to start
      uart_tx_start();
  }

  LED_RIGHT_GREEN_OFF;
}


void uart_tc_callback(void)
{
  crsf.transmit_enable(false); // switches on rx
  crsf.state = tPin5BridgeBase::STATE_IDLE;
}


bool tTxCrsf::transmit_start(void)
{
  if (state < STATE_TRANSMIT_START) return false; // we are in receiving

  if (!tx_available || (state != STATE_TRANSMIT_START)) {
    state = STATE_IDLE;
    return false;
  }

  transmit_enable(true); // switches of rx

  for (uint8_t i = 0; i < tx_available; i++) {
      uint8_t c = tx_frame[i];
      mb_putc(c);
  }

  tx_available = 0;
  state = STATE_TRANSMITING;
  return true;
}


void tTxCrsf::Init(void)
{
  tPin5BridgeBase::Init();

  frame_received = false;
  tx_available = 0;
}


//-------------------------------------------------------
// CRSF Bridge

// a frame is sent every 4 ms, frame length is max 64 bytes
// a byte is 25 us
// gaps between frames are 1 ms or so
#define CRSF_TMO_US        500


// CRSF frame format:
// adress len type payload crc
// len is the length including type, payload, crc

void tTxCrsf::parse_nextchar(uint8_t c, uint16_t tnow_us)
{
  if (state != STATE_IDLE) {
      uint16_t dt = tnow_us - tlast_us;
      if (dt > CRSF_TMO_US) state = STATE_IDLE;
  }

  tlast_us = tnow_us;

  switch (state) {
  case STATE_IDLE:
      if (c == CRSF_ADDRESS_MODULE) {
        cnt = 0;
        frame[cnt++] = c;
        state = STATE_RECEIVE_CRSF_LEN;
      }
      break;

  case STATE_RECEIVE_CRSF_LEN:
      frame[cnt++] = c;
      len = c;
      state = STATE_RECEIVE_CRSF_PAYLOAD;
      break;
  case STATE_RECEIVE_CRSF_PAYLOAD:
      frame[cnt++] = c;
      if (cnt >= len + 1) {
        state = STATE_RECEIVE_CRSF_CRC;
      }
    break;
  case STATE_RECEIVE_CRSF_CRC:
      frame[cnt++] = c;
      // let's just ignore it
      frame_received = true;
      state = STATE_TRANSMIT_START;
      break;
  }
}


uint8_t tTxCrsf::crc8(const uint8_t* buf)
{
  return crc8_update(0, &(buf[2]), buf[1] - 1, 0xD5);
}


bool tTxCrsf::IsEmpty(void)
{
  return (tx_available == 0);
}


bool tTxCrsf::IsChannelData(void)
{
  return (frame[2] == CRSF_FRAME_ID_CHANNELS);
}


void tTxCrsf::SendLinkStatistics(tCrsfLinkStatistics* payload)
{
  constexpr uint8_t len = CRSF_LINK_STATISTICS_LEN;
  tx_frame[0] = CRSF_ADDRESS_RADIO;
  tx_frame[1] = (4-2) + len;
  tx_frame[2] = CRSF_FRAME_ID_LINK_STATISTICS;
  memcpy(&(tx_frame[3]), payload, len);
  tx_frame[3 + len] = crc8(tx_frame);

  tx_available = 4 + len;
}


void tTxCrsf::SendLinkStatisticsTx(tCrsfLinkStatisticsTx* payload)
{
  constexpr uint8_t len = CRSF_LINK_STATISTICS_TX_LEN;
  tx_frame[0] = CRSF_ADDRESS_RADIO;
  tx_frame[1] = (4-2) + len;
  tx_frame[2] = CRSF_FRAME_ID_LINK_STATISTICS_TX;
  memcpy(&(tx_frame[3]), payload, len);
  tx_frame[3 + len] = crc8(tx_frame);

  tx_available = 4 + len;
}


void tTxCrsf::SendLinkStatisticsRx(tCrsfLinkStatisticsRx* payload)
{
  constexpr uint8_t len = CRSF_LINK_STATISTICS_RX_LEN;
  tx_frame[0] = CRSF_ADDRESS_RADIO;
  tx_frame[1] = (4-2) + len;
  tx_frame[2] = CRSF_FRAME_ID_LINK_STATISTICS_RX;
  memcpy(&(tx_frame[3]), payload, len);
  tx_frame[3 + len] = crc8(tx_frame);

  tx_available = 4 + len;
}


//-------------------------------------------------------
// convenience helper

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

// on crsf rssi
// rssi = 255 -> red in otx
//      = 130 -> -126 dB
//      = 129 -> -127 dB
//      = 128 -> -128 dB
//      = 127 ->  127dB
//      = 126 ->  126dB
// hmhm ...

void crsf_send_LinkStatistics(void)
{
tCrsfLinkStatistics lstats;

  lstats.uplink_rssi1 = (stats.last_rx_antenna == ANTENNA_1) ? ((stats.last_rx_rssi1 == -1) ? -2 : stats.last_rx_rssi1) : 255;
  lstats.uplink_rssi2 = (stats.last_rx_antenna == ANTENNA_2) ? ((stats.last_rx_rssi2 == -1) ? -2 : stats.last_rx_rssi2) : 255;
  lstats.uplink_LQ = txstats.GetLQ();
  lstats.uplink_snr = (stats.last_rx_antenna == ANTENNA_1) ? stats.last_rx_snr1 : stats.last_rx_snr2;
  lstats.active_antenna = stats.last_rx_antenna;
  lstats.mode = (Setup.Mode == MODE_19HZ) ? 19 : 1;
  lstats.uplink_transmit_power = crsf_cvt_power(sx.RfPower_dbm());
  lstats.downlink_rssi = (stats.received_rssi == -1) ? -2 : stats.received_rssi;
  lstats.downlink_LQ = stats.received_LQ;
  lstats.downlink_snr = 0;
  crsf.SendLinkStatistics(&lstats);
}


void crsf_send_LinkStatisticsTx(void)
{
tCrsfLinkStatisticsTx lstats;

  lstats.uplink_rssi = (stats.last_rx_antenna == ANTENNA_1) ? stats.last_rx_rssi1 : stats.last_rx_rssi2; // ignored by OpenTx
  lstats.uplink_rssi_percent = 12; // TODO
  lstats.uplink_LQ = txstats.GetLQ(); // ignored by OpenTx
  lstats.uplink_snr = (stats.last_rx_antenna == ANTENNA_1) ? stats.last_rx_snr1 : stats.last_rx_snr2; // ignored by OpenTx
  lstats.downlink_transmit_power = UINT8_MAX; // we only can guess it // crsf_cvt_power(sx.RfPower_dbm());
  lstats.uplink_fps = (Setup.Mode == MODE_19HZ) ? 19 : 50;
  crsf.SendLinkStatisticsTx(&lstats);
}


void crsf_send_LinkStatisticsRx(void)
{
tCrsfLinkStatisticsRx lstats;

  lstats.downlink_rssi = stats.received_rssi; // ignored by OpenTx
  lstats.downlink_rssi_percent = 13; // TODO
  lstats.downlink_LQ = stats.received_LQ; // ignored by OpenTx
  lstats.downlink_snr = 0; // ignored by OpenTx
  lstats.uplink_transmit_power = crsf_cvt_power(sx.RfPower_dbm());
  crsf.SendLinkStatisticsRx(&lstats);
}


// CRSF:    172 .. 992 .. 1811, 11 bits
// rcData:  0 .. 1024 .. 2047, 11 bits
void fill_rcdata_from_crsf(tRcData* rc, uint8_t* frame)
{
tCrsfChannelBuffer buf;

  memcpy(buf.c, &(frame[3]), CRSF_CHANNELPACKET_SIZE);

  rc->ch[0] = clip_rc( (((int32_t)(buf.ch0) - 992) * 2047) / 1638 + 1024 );
  rc->ch[1] = clip_rc( (((int32_t)(buf.ch1) - 992) * 2047) / 1638 + 1024 );
  rc->ch[2] = clip_rc( (((int32_t)(buf.ch2) - 992) * 2047) / 1638 + 1024 );
  rc->ch[3] = clip_rc( (((int32_t)(buf.ch3) - 992) * 2047) / 1638 + 1024 );
  rc->ch[4] = clip_rc( (((int32_t)(buf.ch4) - 992) * 2047) / 1638 + 1024 );
  rc->ch[5] = clip_rc( (((int32_t)(buf.ch5) - 992) * 2047) / 1638 + 1024 );
  rc->ch[6] = clip_rc( (((int32_t)(buf.ch6) - 992) * 2047) / 1638 + 1024 );
  rc->ch[7] = clip_rc( (((int32_t)(buf.ch7) - 992) * 2047) / 1638 + 1024 );
  rc->ch[8] = clip_rc( (((int32_t)(buf.ch8) - 992) * 2047) / 1638 + 1024 );
  rc->ch[9] = clip_rc( (((int32_t)(buf.ch9) - 992) * 2047) / 1638 + 1024 );

  rc->ch[10] = clip_rc( (((int32_t)(buf.ch10) - 992) * 2047) / 1638 + 1024 );
  rc->ch[11] = clip_rc( (((int32_t)(buf.ch11) - 992) * 2047) / 1638 + 1024 );
  rc->ch[12] = clip_rc( (((int32_t)(buf.ch12) - 992) * 2047) / 1638 + 1024 );
  rc->ch[13] = clip_rc( (((int32_t)(buf.ch13) - 992) * 2047) / 1638 + 1024 );
  rc->ch[14] = clip_rc( (((int32_t)(buf.ch14) - 992) * 2047) / 1638 + 1024 );
  rc->ch[15] = clip_rc( (((int32_t)(buf.ch15) - 992) * 2047) / 1638 + 1024 );
}



#endif // if (defined USE_CRSF) && (defined DEVICE_HAS_JRPIN5)

#endif // CRSF_INTERFACE_H








/*

void tTxCrsf::SpinOnce(void)
{
uint8_t c;

  if ((state != STATE_IDLE)) {
    uint16_t dt = tim_us() - tlast_us;
    if (dt > CRSF_TMO_US) state = STATE_IDLE;
  }

  switch (state) {
  case STATE_IDLE:
    if (!mb_rx_available()) break;
    tlast_us = tim_us();
    c = mb_getc();
    if (c == CRSF_ADDRESS_MODULE) {
      cnt = 0;
      frame[cnt++] = c;
      state = STATE_RECEIVE_CRSF_LEN;
    }
    break;
  case STATE_RECEIVE_CRSF_LEN:
    if (!mb_rx_available()) break;
    tlast_us = tim_us();
    c = mb_getc();
    frame[cnt++] = c;
    len = c;
    state = STATE_RECEIVE_CRSF_PAYLOAD;
    break;
  case STATE_RECEIVE_CRSF_PAYLOAD:
    if (!mb_rx_available()) break;
    tlast_us = tim_us();
    c = mb_getc();
    frame[cnt++] = c;
    if (cnt >= len + 1) {
      state = STATE_RECEIVE_CRSF_CRC;
    }
    break;
  case STATE_RECEIVE_CRSF_CRC:
    if (!mb_rx_available()) break;
    tlast_us = tim_us();
    c = mb_getc();
    frame[cnt++] = c;
    updated = true;
    state = STATE_IDLE;
    break;
  }
}

 */
