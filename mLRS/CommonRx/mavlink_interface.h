//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Mavlink Interface RX Side
//*******************************************************
#ifndef MAVLINK_INTERFACE_H
#define MAVLINK_INTERFACE_H
#pragma once


#define MAVLINK_BUF_SIZE            300 // needs to be larger than max mavlink frame size = 286 bytes

uint8_t f_buf[MAVLINK_BUF_SIZE];
fmav_message_t f_msg;
fmav_result_t f_result;
fmav_status_t f_status;

bool inject_radio_status;


void f_init(void)
{
  fmav_init();

  f_result = {0};
  f_status = {0};

  inject_radio_status = false;
}


void f_send(void)
{
  uint16_t len = fmav_msg_to_frame_buf(f_buf, &f_msg);

  serial.putbuf(f_buf, len);
}


void send_radio_status(void)
{
uint8_t rssi, remrssi, txbuf;

  rssi = rxstats.GetLQ();
  remrssi = stats.received_LQ;

  txbuf = serial.rx_free_percent();

  fmav_msg_radio_status_pack(
      &f_msg,
      51, // sysid, SiK uses 51, 68
      MAV_COMP_ID_TELEMETRY_RADIO,
      rssi, remrssi, txbuf, UINT8_MAX, UINT8_MAX, 0, 0,
      //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
      &f_status);
  f_send();
}


#endif // MAVLINK_INTERFACE_H
