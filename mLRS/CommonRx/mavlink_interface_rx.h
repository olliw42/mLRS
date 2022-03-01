//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Mavlink Interface RX Side
//*******************************************************
#ifndef MAVLINK_INTERFACE_RX_H
#define MAVLINK_INTERFACE_RX_H
#pragma once


#define MAVLINK_BUF_SIZE            300 // needs to be larger than max mavlink frame size = 286 bytes

uint8_t f_buf[MAVLINK_BUF_SIZE];
fmav_message_t f_msg;
fmav_result_t f_result;
fmav_status_t f_status;

bool f_inject_radio_status;


void f_init(void)
{
  fmav_init();

  f_result = {0};
  f_status = {0};

  f_inject_radio_status = false;
}


void f_send(void)
{
  uint16_t len = fmav_msg_to_frame_buf(f_buf, &f_msg);

  serial.putbuf(f_buf, len);
}


// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp#L483-L510
uint8_t f_convert_rssi_to_ap(int8_t rssi)
{
  if (rssi == RSSI_INVALID) return UINT8_MAX;
  if (rssi > -50) return 254; // max value
  if (rssi < -120) return 0;

  int32_t r = (int32_t)rssi - (-120);
  int32_t m = (int32_t)(-50) - (-120);

  return (100 * r + 49) / m;
}


void send_radio_status(void)
{
uint8_t rssi, remrssi, txbuf;

  rssi = rxstats.GetLQ();
  remrssi = stats.received_LQ;

  rssi = f_convert_rssi_to_ap(stats.GetLastRxRssi());
  remrssi = f_convert_rssi_to_ap(stats.received_rssi);

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


// call at 1 Hz
void f_update_1hz(bool connected)
{
  if (connected) f_inject_radio_status = true;
}


void f_handle_link_receive(char c)
{
  // send to serial
  serial.putc(c);

  // parse stream, and inject radio status
  uint8_t res = fmav_parse_to_frame_buf(&f_result, f_buf, &f_status, c);

  if (res == FASTMAVLINK_PARSE_RESULT_OK && f_inject_radio_status) { // we have a complete mavlink frame
    f_inject_radio_status = false;
    send_radio_status();
    LED_RED_TOGGLE; // indicate we send the radio status
  }
}


#endif // MAVLINK_INTERFACE_RX_H
