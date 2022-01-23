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


// method implementations

void tMBridgeBase::Init(void)
{
  state = STATE_IDLE;
  type = TYPE_NONE;
  len = 0;
  cnt = 0;
  tlast_us = 0;
  channels_updated = 0;
  cmd_received = 0;
  cmd_send_available = 0;
}


void tMBridgeBase::parse_nextchar(uint8_t c)
{
  switch (state) {
  case STATE_IDLE:
      tlast_us = tim_us();
      if (c == MBRIDGE_STX1) state = STATE_RECEIVE_STX2;
      break;
  case STATE_RECEIVE_STX2: {
      tlast_us = tim_us();
      if (c == MBRIDGE_STX2) state = STATE_RECEIVE_LEN; else state = STATE_IDLE; // error
      uint16_t dt = tim_us() - tlast_us;
      if (dt > MBRIDGE_TMO_US) state = STATE_IDLE; // timeout error
      }break;
  case STATE_RECEIVE_LEN: {
      tlast_us = tim_us();
      cnt = 0;
      if (c == MBRIDGE_CHANNELPACKET_STX) {
          len = MBRIDGE_CHANNELPACKET_SIZE;
          type = TYPE_CHANNELPACKET;
          state = STATE_RECEIVE_CHANNELPACKET;
      } else if (c >= MBRIDGE_COMMANDPACKET_STX) {
          cmd_receive_cmd = (c & ~MBRIDGE_COMMANDPACKET_STX_MASK);
          len = MBRIDGE_COMMANDPACKET_RX_SIZE;
          type = TYPE_COMMANDPACKET;
          state = STATE_RECEIVE_COMMANDPACKET;
      } else if (c > MBRIDGE_SERIALPACKET_RX_SIZE_MAX) {
          state = STATE_IDLE; // error
      } else if (c > 0) {
          len = c;
          type = TYPE_SERIALPACKET;
          state = STATE_RECEIVE_SERIALPACKET;
      } else {
          type = TYPE_NONE;
          state = STATE_TRANSMIT_START; // tx_len = 0, no payload
      }
      uint16_t dt = tim_us() - tlast_us;
      if (dt > MBRIDGE_TMO_US) state = STATE_IDLE; // timeout error
      }break;

  case STATE_RECEIVE_SERIALPACKET: {
      tlast_us = tim_us();
      serial_putc(c);
      cnt++;
      if (cnt >= len) state = STATE_TRANSMIT_START;
      uint16_t dt = tim_us() - tlast_us;
      if (dt > MBRIDGE_TMO_US) state = STATE_IDLE; // timeout error
      }break;

  case STATE_RECEIVE_CHANNELPACKET: {
      tlast_us = tim_us();
      channels.c[cnt] = c;
      cnt++;
      if (cnt >= len) {
          channels_updated = 1;
          state = STATE_TRANSMIT_START;
      }
      uint16_t dt = tim_us() - tlast_us;
      if (dt > MBRIDGE_TMO_US) state = STATE_IDLE; // timeout error
      }break;

  case STATE_RECEIVE_COMMANDPACKET: {
      tlast_us = tim_us();
      cmd_receive_buf[cnt] = c;
      cnt++;
      if (cnt >= len) {
          cmd_received = 1;
          state = STATE_TRANSMIT_START;
      }
      uint16_t dt = tim_us() - tlast_us;
      if (dt > MBRIDGE_TMO_US) state = STATE_IDLE; // timeout error
      }break;
  }
}


void tMBridgeBase::transmit_start(void)
{
uint8_t count = 0;

  transmit_enable(true);
  if (cmd_send_available) {
      count = transmit_cmd();
  } else {
      count = transmit_serial();
  }
  if (count) {
      state = STATE_TRANSMIT_CLOSE;
  } else {
      transmit_enable(false);
      state = STATE_IDLE;
  }
}


uint8_t tMBridgeBase::transmit_serial(void)
{
  // send up to 16 bytes
  // if we received a channel or command packet we only do 11
  uint8_t count = (type >= TYPE_CHANNELPACKET) ? MBRIDGE_SERIALPACKET_TX_SIZE_LIM : MBRIDGE_SERIALPACKET_TX_SIZE_MAX;
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


uint8_t tMBridgeBase::transmit_cmd(void)
{
  cmd_send_available = 0;
  // send type byte
  mb_putc(MBRIDGE_COMMANDPACKET_STX + (cmd_send_cmd &~ MBRIDGE_COMMANDPACKET_STX_MASK));
  // send 12 bytes
  for (uint8_t i = 0; i < MBRIDGE_COMMANDPACKET_TX_SIZE; i++) {
      uint8_t c = cmd_send_buf[i];
      mb_putc(c);
  }
  return MBRIDGE_COMMANDPACKET_TX_SIZE;
}


void tMBridgeBase::transmit_close(void)
{
  transmit_enable(true);
  state = STATE_IDLE;
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


//-- in-loop processing:
// call SpinOnce() repeatedly

void tMBridgeBase::SpinOnce(void)
{
  switch (state) {
  case STATE_IDLE:
  case STATE_RECEIVE_STX2:
  case STATE_RECEIVE_LEN:
  case STATE_RECEIVE_SERIALPACKET:
  case STATE_RECEIVE_CHANNELPACKET:
      if (mb_rx_available()) {
          uint8_t c = mb_getc();
          parse_nextchar(c);
      }
      break;

  case STATE_TRANSMIT_START: {
      uint16_t dt = tim_us() - tlast_us;
      if (dt > MBRIDGE_RX2TX_DELAY_US) state = STATE_TRANSMIT_PUTCHARS;
      }break;

  case STATE_TRANSMIT_PUTCHARS:
      transmit_start();
      break;

  case STATE_TRANSMIT_CLOSE: {
      // wait for bytes to be transmitted
      uint16_t dt = tim_us() - tlast_us;
      if (dt > 500) { // 16 bytes @ 400000 bps = 400 us
          transmit_close();
      }
      }break;
  }
}


bool tMBridgeBase::cmd_from_transmitter(uint8_t* cmd, uint8_t* payload)
{
  if (cmd_received) {
      *cmd = cmd_receive_cmd;
      memcpy(payload, cmd_receive_buf, MBRIDGE_COMMANDPACKET_RX_SIZE);
      cmd_received = 0;
      return true;
  }
  *cmd = 0;
  return false;
}


void tMBridgeBase::cmd_to_transmitter(uint8_t cmd, uint8_t* payload, uint8_t len)
{
  cmd_send_cmd = cmd;
  memset(cmd_send_buf, 0, MBRIDGE_COMMANDPACKET_TX_SIZE);
  if (len > MBRIDGE_COMMANDPACKET_TX_SIZE) len = MBRIDGE_COMMANDPACKET_TX_SIZE;
  memcpy(cmd_send_buf, payload, len);
  cmd_send_available = 1;
}





