//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// MBridge Interface Header
//********************************************************
#ifndef MBRIDGE_INTERFACE_H
#define MBRIDGE_INTERFACE_H
#pragma once


#if (defined USE_MBRIDGE) && (defined DEVICE_HAS_JRPIN5)

#include "jr_pin5_interface.h"
#include "mbridge_protocol.h"
#include "..\Common\fifo.h"


uint16_t micros(void);


//-------------------------------------------------------
// Interface Implementation

class tMBridge : public tPin5BridgeBase, public tSerialBase
{
  public:
    void Init(void);
    bool ChannelsUpdated(tRcData* rc);
    void GetCommand(uint8_t* cmd, uint8_t* payload);
    void SendCommand(uint8_t cmd, uint8_t* payload, uint8_t payload_len);

    // for in-isr processing
    void parse_nextchar(uint8_t c, uint16_t tnow_us) override;
    bool transmit_start(void) override; // returns true if transmission should be started
    uint8_t send_serial(void);
    void send_command(void);

    uint8_t type;

    volatile bool channels_received;
    tMBridgeChannelBuffer channels;
    void fill_rcdata(tRcData* rc);

    volatile bool cmd_received;
    uint8_t cmd_r2m_frame[MBRIDGE_R2M_COMMAND_FRAME_LEN_MAX];

    volatile uint8_t cmd_m2r_available;
    uint8_t cmd_m2r_frame[MBRIDGE_M2R_COMMAND_FRAME_LEN_MAX];

    // front end to communicate with mbridge
    void putc(char c) { sx_rx_fifo.Put(c); }
    void putbuf(void* buf, uint16_t len) { sx_rx_fifo.PutBuf(buf, len); }
    bool available(void) { return sx_tx_fifo.Available(); }
    char getc(void) { return sx_tx_fifo.Get(); }
    void flush(void) { sx_tx_fifo.Flush(); }

    // backend
    // mimics a serial interface to the main code, usually two fifo
    void serial_putc(char c) { sx_tx_fifo.Put(c); }
    bool serial_rx_available(void) { return sx_rx_fifo.Available(); }
    char serial_getc(void) { return sx_rx_fifo.Get(); }

    FifoBase<char,256> sx_tx_fifo;
    FifoBase<char,256> sx_rx_fifo;
};

tMBridge mbridge;


// we do not add a delay here as with SpinOnce()
// the logic analyzer shows this gives a 30-35 us gap nevertheless, which is perfect

void uart_rx_callback(uint8_t c)
{
  LED_RIGHT_GREEN_ON;

  if (mbridge.state >= tPin5BridgeBase::STATE_TRANSMIT_START) { // recover in case something went wrong
      mbridge.state = tPin5BridgeBase::STATE_IDLE;
  }

  uint16_t tnow_us = micros();
  mbridge.parse_nextchar(c, tnow_us);

  if (mbridge.transmit_start()) {
      uart_tx_start();
  }

  LED_RIGHT_GREEN_OFF;
}


void uart_tc_callback(void)
{
  mbridge.transmit_enable(false);
  mbridge.state = tPin5BridgeBase::STATE_IDLE;
}


bool tMBridge::transmit_start(void)
{
uint8_t available = 0;

  if (state < STATE_TRANSMIT_START) return false; // we are in receiving

  if (state != STATE_TRANSMIT_START) {
      state = STATE_IDLE;
      return false;
  }

  if (cmd_m2r_available) {
      send_command(); // uses cmd_m2r_available
      available = cmd_m2r_available;
      cmd_m2r_available = 0;
  } else {
      available = send_serial();
  }

  if (!available) {
      state = STATE_IDLE;
      return false;
  }

  transmit_enable(true);

  state = STATE_TRANSMITING;
  return true;
}


uint8_t tMBridge::send_serial(void)
{
  uint8_t count = 0;
  uint8_t payload[MBRIDGE_M2R_SERIAL_PAYLOAD_LEN_MAX];
  for (uint8_t i = 0; i < MBRIDGE_M2R_SERIAL_PAYLOAD_LEN_MAX; i++) {
      if (!serial_rx_available()) break;
      payload[count++] = serial_getc();
  }
  if (count > 0) {
      mb_putc(0x00); // we can send anything we want which is not a command, send 0xoo so it is easy to recognize
      for (uint8_t i = 0; i < count; i++) {
          uint8_t c = payload[i];
          mb_putc(c);
      }
  }
  return count;
}


void tMBridge::send_command(void)
{
  for (uint8_t i = 0; i < cmd_m2r_available; i++) {
      uint8_t c = cmd_m2r_frame[i];
      mb_putc(c);
  }
}


#define MBRIDGE_TMO_US  250


void tMBridge::parse_nextchar(uint8_t c, uint16_t tnow_us)
{
  if (state != STATE_IDLE) {
      uint16_t dt = tnow_us - tlast_us;
      if (dt > MBRIDGE_TMO_US) state = STATE_IDLE; // timeout error
  }

  tlast_us = tnow_us;

  switch (state) {
  case STATE_IDLE:
      if (c == MBRIDGE_STX1) state = STATE_RECEIVE_MBRIDGE_STX2;
      break;

  case STATE_RECEIVE_MBRIDGE_STX2:
      if (c == MBRIDGE_STX2) state = STATE_RECEIVE_MBRIDGE_LEN; else state = STATE_IDLE; // error
      break;
  case STATE_RECEIVE_MBRIDGE_LEN:
      cnt = 0;
      if (c == MBRIDGE_CHANNELPACKET_STX) {
          len = MBRIDGE_CHANNELPACKET_SIZE;
          type = MBRIDGE_TYPE_CHANNELPACKET;
          state = STATE_RECEIVE_MBRIDGE_CHANNELPACKET;
      } else
      if (c >= MBRIDGE_COMMANDPACKET_STX) {
          uint8_t cmd = c & (~MBRIDGE_COMMANDPACKET_MASK);
          cmd_r2m_frame[cnt++] = cmd;
          len = mbridge_cmd_payload_len(cmd);
          type = MBRIDGE_TYPE_COMMANDPACKET;
          state = STATE_RECEIVE_MBRIDGE_COMMANDPACKET;
      } else
      if (c > MBRIDGE_R2M_SERIAL_PAYLOAD_LEN_MAX) {
          state = STATE_IDLE; // error
      } else
      if (c > 0) {
          len = c;
          type = MBRIDGE_TYPE_SERIALPACKET;
          state = STATE_RECEIVE_MBRIDGE_SERIALPACKET;
      } else {
          type = MBRIDGE_TYPE_NONE;
          state = STATE_TRANSMIT_START; // tx_len = 0, no payload
      }
      break;
  case STATE_RECEIVE_MBRIDGE_SERIALPACKET:
      serial_putc(c);
      cnt++;
      if (cnt >= len) state = STATE_TRANSMIT_START;
      break;
  case STATE_RECEIVE_MBRIDGE_CHANNELPACKET:
      channels.c[cnt++] = c;
      if (cnt >= len) {
          channels_received = true;
          state = STATE_TRANSMIT_START;
      }
      break;
  case STATE_RECEIVE_MBRIDGE_COMMANDPACKET:
      cmd_r2m_frame[cnt++] = c;
      if (cnt >= len + 1) {
          cmd_received = true;
          state = STATE_TRANSMIT_START;
      }
      break;
  }
}


// mBridge: ch0-15    11 bits, 1 .. 1024 .. 2047 for +-120%
//          ch16-17:  1 bit, 0 .. 1
// rcData:            11 bit, 1 .. 1024 .. 2047 for +-120%
void tMBridge::fill_rcdata(tRcData* rc)
{
  rc->ch[0] = channels.ch0;
  rc->ch[1] = channels.ch1;
  rc->ch[2] = channels.ch2;
  rc->ch[3] = channels.ch3;
  rc->ch[4] = channels.ch4;
  rc->ch[5] = channels.ch5;
  rc->ch[6] = channels.ch6;
  rc->ch[7] = channels.ch7;
  rc->ch[8] = channels.ch8;
  rc->ch[9] = channels.ch9;
  rc->ch[10] = channels.ch10;
  rc->ch[11] = channels.ch11;
  rc->ch[12] = channels.ch12;
  rc->ch[13] = channels.ch13;
  rc->ch[14] = channels.ch14;
  rc->ch[15] = channels.ch15;
  rc->ch[16] = (channels.ch16) ? 1876 : 172; // +-100%
  rc->ch[17] = (channels.ch17) ? 1876 : 172; // +-100%
}


STATIC_ASSERT(sizeof(tMBridgeLinkStats) == MBRIDGE_CMD_TX_LINK_STATS_LEN, "tMBridgeLinkStats len missmatch")


//-------------------------------------------------------
// MBridge user interface

void tMBridge::Init(void)
{
  tSerialBase::Init();
  tPin5BridgeBase::Init();

  type = MBRIDGE_TYPE_NONE;
  channels_received = false;
  cmd_received = false;
  cmd_m2r_available = 0;

  sx_tx_fifo.Init();
  sx_rx_fifo.Init();
}


bool tMBridge::ChannelsUpdated(tRcData* rc)
{
  if (!channels_received) return false;

  channels_received = false;

  fill_rcdata(rc);
  return true;
}


void tMBridge::GetCommand(uint8_t* cmd, uint8_t* payload)
{
  if ((cmd_r2m_frame[0] & MBRIDGE_COMMANDPACKET_MASK) != MBRIDGE_COMMANDPACKET_STX) return; // not a command, should not happen, but play it safe

  *cmd = cmd_r2m_frame[0] & (~MBRIDGE_COMMANDPACKET_MASK) ;

  uint8_t payload_len = mbridge_cmd_payload_len(*cmd);
  memcpy(payload, &(cmd_r2m_frame[1]), payload_len);
}


void tMBridge::SendCommand(uint8_t cmd, uint8_t* payload, uint8_t payload_len)
{
  memset(cmd_m2r_frame, 0, MBRIDGE_M2R_COMMAND_FRAME_LEN_MAX);

  if (payload_len != mbridge_cmd_payload_len(cmd)) return; // should never happen but play it safe

  cmd_m2r_frame[0] = MBRIDGE_COMMANDPACKET_STX + (cmd & (~MBRIDGE_COMMANDPACKET_MASK));
  memcpy(&(cmd_m2r_frame[1]), payload, payload_len);

  cmd_m2r_available = payload_len + 1;
}


//-------------------------------------------------------
// convenience helper

void mbridge_send_LinkStats(void)
{
tMBridgeLinkStats lstats = {0};

  lstats.LQ = txstats.GetLQ(); // it's the same as GetLQ_serial_data() // = LQ_valid_received; // number of valid packets received on transmitter side
  lstats.rssi1_instantaneous = (stats.last_rx_antenna == ANTENNA_1) ? stats.last_rx_rssi1 : RSSI_INVALID;
  lstats.rssi2_instantaneous = (stats.last_rx_antenna == ANTENNA_2) ? stats.last_rx_rssi2 : RSSI_INVALID;
  lstats.snr_instantaneous = stats.GetLastRxSnr();
  lstats.receive_antenna = stats.last_rx_antenna;
  lstats.transmit_antenna = stats.last_tx_antenna;
#ifdef USE_DIVERSITY
  lstats.diversity = 1;
#else
  lstats.diversity = 0;
#endif

  lstats.rssi1_filtered = RSSI_INVALID;
  lstats.rssi2_filtered = RSSI_INVALID;
  lstats.snr_filtered = SNR_INVALID;

  // receiver side of things

  lstats.receiver_LQ = stats.received_LQ; // valid_crc1_received, number of rc data packets received on receiver side
  lstats.receiver_LQ_serial = stats.received_LQ_serial_data; // valid_frames_received, number of completely valid packets received on receiver side
  lstats.receiver_rssi_instantaneous = stats.received_rssi;
  lstats.receiver_receive_antenna = stats.received_antenna;
  lstats.receiver_transmit_antenna = stats.received_transmit_antenna;
  lstats.receiver_diversity = 0; // TODO: this we do not know currently

  lstats.receiver_rssi_filtered = RSSI_INVALID;

  // further stats acquired on transmitter side

  lstats.LQ_fresh_serial_packets_transmitted = stats.fresh_serial_data_transmitted.GetLQ();
  lstats.bytes_per_sec_transmitted = stats.GetTransmitBandwidthUsage();

  lstats.LQ_valid_received = stats.valid_frames_received.GetLQ(); // number of completely valid packets received per sec
  lstats.LQ_fresh_serial_packets_received = stats.fresh_serial_data_received.GetLQ();
  lstats.bytes_per_sec_received = stats.GetReceiveBandwidthUsage();

  lstats.LQ_received = stats.frames_received.GetLQ(); // number of packets received per sec, not practically relevant

  mbridge.SendCommand(MBRIDGE_CMD_TX_LINK_STATS, (uint8_t*)&lstats, sizeof(tMBridgeLinkStats));
}


#else

tSerialBase mbridge;

#endif // if (defined USE_MBRIDGE) && (defined DEVICE_HAS_JRPIN5)

#endif // MBRIDGE_INTERFACE_H
