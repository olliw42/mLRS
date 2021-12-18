//*******************************************************
// MLRS project
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Mavlink Interface
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
#if SETUP_TX_USE_MBRIDGE==1
  bridge.putbuf(f_buf, len);
#else
  serial.putbuf(f_buf, len);
#endif
}


void send_radio_status(void)
{
uint8_t rssi, remrssi;

  // this would be the naive thing
  // scale is "inverted" to make it that it is higher the better
  rssi = (127 + stats.last_rssi);
  remrssi = (127 +stats.received_rssi);

  // https://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html#monitoring-the-link-quality
  // for Sik radios holds signal_dBm approx rssi_SiK/1.9 - 127  => 150 approx -48dBm, 70 approx -90dBm
  // so we fake here a SiK scale
  //   rssi_SiK = ( signal_dBm + 127 ) * 1.9

  int32_t rssi_SiK = ( ((int32_t)stats.last_rssi + 127) * 19000 ) / 10000;
  if (rssi_SiK < 0) rssi_SiK = 0;
  if (rssi_SiK > 250) rssi_SiK = 250;
  rssi = rssi_SiK;

  int32_t remrssi_SiK = ( ((int32_t)stats.received_rssi + 127) * 19000 ) / 10000;
  if (remrssi_SiK < 0) remrssi_SiK = 0;
  if (remrssi_SiK > 250) remrssi_SiK = 250;
  remrssi = remrssi_SiK;

  // argh, what a nonsense the sx12xx rssi & snr is
  // so substitute with LQ
  rssi = txstats.GetLQ();
  remrssi = stats.received_LQ;

  fmav_msg_radio_status_pack(
      &f_msg,
      51, // sysid, SiK uses 51, 68
      MAV_COMP_ID_TELEMETRY_RADIO,
      rssi, remrssi, 0, UINT8_MAX, UINT8_MAX, 0, 0,
      //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
      &f_status);
  f_send();
//  uartc_puts("R");
}


void send_radio_status_v2(void)
{
  fmav_msg_radio_status_v2_pack(
      &f_msg,
      51, // sysid, SiK uses 51, 68
      MAV_COMP_ID_TELEMETRY_RADIO,
      stats.last_rssi, (128 + stats.last_rssi), txstats.GetLQ(), stats.last_snr,
      stats.received_rssi, (128 + stats.received_rssi), stats.received_LQ, INT8_MAX,
      //int8_t rssi_dbm, uint8_t rssi_au, uint8_t LQ, int8_t snr_dbm,
      //int8_t rem_rssi_dbm, uint8_t rem_rssi_au, uint8_t rem_LQ, int8_t rem_snr_dbm,
      &f_status);
  f_send();
}



#endif // MAVLINK_INTERFACE_H
