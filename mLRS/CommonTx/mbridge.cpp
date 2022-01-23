//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// M Bridge
//********************************************************

#include "mbridge.h"
#include <string.h>


#define MBRIDGE_TMO_US  250


// method implementations

void tMBridgeBase::Init(void)
{
  state = STATE_IDLE;
  len = 0;
  cnt = 0;
  tlast_us = 0;

  type = MBRIDGE_TYPE_NONE;
  channels_received = false;
  cmd_received = false;
  cmd_tx_available = 0;
}


void tMBridgeBase::parse_nextchar(uint8_t c, uint16_t tnow_us)
{
  if (state != STATE_IDLE) {
    uint16_t dt = tnow_us - tlast_us;
    if (dt > MBRIDGE_TMO_US) state = STATE_IDLE; // timeout error
  }

  tlast_us = tnow_us;

  switch (state) {
  case STATE_IDLE:
      if (c == MBRIDGE_STX1) state = STATE_MBRIDGE_RECEIVE_STX2;
      break;

  case STATE_MBRIDGE_RECEIVE_STX2:
      if (c == MBRIDGE_STX2) state = STATE_MBRIDGE_RECEIVE_LEN; else state = STATE_IDLE; // error
      break;

  case STATE_MBRIDGE_RECEIVE_LEN:
      cnt = 0;
      if (c == MBRIDGE_CHANNELPACKET_STX) {
          len = MBRIDGE_CHANNELPACKET_SIZE;
          type = MBRIDGE_TYPE_CHANNELPACKET;
          state = STATE_MBRIDGE_RECEIVE_CHANNELPACKET;
      } else
      if (c >= MBRIDGE_COMMANDPACKET_STX) {
          cmd_rx_frame[cnt++] = (c & ~MBRIDGE_COMMANDPACKET_MASK);
          len = MBRIDGE_RX_COMMAND_PAYLOAD_LEN;
          type = MBRIDGE_TYPE_COMMANDPACKET;
          state = STATE_MBRIDGE_RECEIVE_COMMANDPACKET;
      } else
      if (c > MBRIDGE_RX_SERIAL_PAYLOAD_LEN_MAX) {
          state = STATE_IDLE; // error
      } else
      if (c > 0) {
          len = c;
          type = MBRIDGE_TYPE_SERIALPACKET;
          state = STATE_MBRIDGE_RECEIVE_SERIALPACKET;
      } else {
          type = MBRIDGE_TYPE_NONE;
          state = STATE_TRANSMIT_START; // tx_len = 0, no payload
      }
      break;

  case STATE_MBRIDGE_RECEIVE_SERIALPACKET:
      serial_putc(c);
      cnt++;
      if (cnt >= len) state = STATE_TRANSMIT_START;
      break;

  case STATE_MBRIDGE_RECEIVE_CHANNELPACKET:
      channels.c[cnt++] = c;
      if (cnt >= len) {
          channels_received = true;
          state = STATE_TRANSMIT_START;
      }
      break;

  case STATE_MBRIDGE_RECEIVE_COMMANDPACKET:
      cmd_rx_frame[cnt++] = c;
      if (cnt >= len + 1) {
          cmd_received = true;
          state = STATE_TRANSMIT_START;
      }
      break;
  }
}


//-- in-isr processing:
// in uart rx isr
//   parse_nextchar(c)
//   if tx_state == TXSTATE_TRANSMIT_START:
//       transmit_start()
//       uart tx start
// in uart tc isr
//   transmit_enable(DISABLE)
//   tx_state = TXSTATE_IDLE

bool tMBridgeBase::transmit_start(void)
{
uint8_t tx_available = 0;

  if (state < STATE_TRANSMIT_START) return false; // we are in receiving

  if (cmd_tx_available) {
      tx_available = cmd_tx_available;
      cmd_tx_available = 0;
      send_command();
  } else {
      tx_available = send_serial();
  }

  if (!tx_available) {
    state = STATE_IDLE;
    return false;
  }

  transmit_enable(false);

  state = STATE_TRANSMITING;
  return true;
}


uint8_t tMBridgeBase::send_serial(void)
{
  // send up to 16 bytes
  // if we received a channel or command packet we only do 11
  uint8_t count = (type >= MBRIDGE_TYPE_CHANNELPACKET) ? MBRIDGE_TX_SERIAL_PAYLOAD_LEN_LIM : MBRIDGE_TX_SERIAL_PAYLOAD_LEN_MAX;
  uint8_t actual_count = 0;
  for (uint8_t i = 0; i < count; i++) {
      if (!serial_rx_available()) break;
      if (i == 0) {
          mb_putc(MBRIDGE_SERIALPACKET_STX); // send type byte
      }
      uint8_t c = serial_getc();
      mb_putc(c);
      actual_count++;
  }
  return actual_count;
}


void tMBridgeBase::send_command(void)
{
  for (uint8_t i = 0; i < MBRIDGE_TX_COMMAND_FRAME_LEN; i++) {
      uint8_t c = cmd_tx_frame[i];
      mb_putc(c);
  }
}


void tMBridgeBase::GetCommand(uint8_t* cmd, uint8_t* payload)
{
  *cmd = cmd_rx_frame[0];
  memcpy(payload, &(cmd_rx_frame[1]), MBRIDGE_RX_COMMAND_PAYLOAD_LEN);
}


void tMBridgeBase::SendCommand(uint8_t cmd, uint8_t* payload, uint8_t payload_len)
{
  memset(cmd_tx_frame, 0, MBRIDGE_TX_COMMAND_FRAME_LEN);
  if (payload_len > MBRIDGE_TX_COMMAND_PAYLOAD_LEN) payload_len = MBRIDGE_TX_COMMAND_PAYLOAD_LEN;

  cmd_tx_frame[0] = MBRIDGE_COMMANDPACKET_STX + (cmd &~ MBRIDGE_COMMANDPACKET_MASK);
  memcpy(&(cmd_tx_frame[1]), payload, payload_len);

  cmd_tx_available = MBRIDGE_TX_COMMAND_FRAME_LEN;
}





