//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Mavlink Interface TX Side
//*******************************************************
#ifndef MAVLINK_INTERFACE_TX_H
#define MAVLINK_INTERFACE_TX_H
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

  switch (Setup.Tx.SerialDestination) {
    case SERIAL_DESTINATION_SERIAL_PORT:
      serial.putbuf(f_buf, len);
      break;
    case SERIAL_DESTINATION_MBRDIGE:
#ifdef USE_MBRIDGE
      bridge.putbuf(f_buf, len);
#endif
      break;
  }
}


void send_radio_status(void)
{
uint8_t rssi, remrssi, txbuf;
/*
  // this would be the naive thing
  // scale is "inverted" to make it that it is higher the better
  rssi = (127 + stats.last_rx_rssi);
  remrssi = (127 + stats.received_rssi);

  // https://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html#monitoring-the-link-quality
  // for Sik radios holds signal_dBm approx rssi_SiK/1.9 - 127  => 150 approx -48dBm, 70 approx -90dBm
  // so we fake here a SiK scale
  //   rssi_SiK = ( signal_dBm + 127 ) * 1.9

  int32_t rssi_SiK = ( ((int32_t)stats.last_rx_rssi + 127) * 19000 ) / 10000;
  if (rssi_SiK < 0) rssi_SiK = 0;
  if (rssi_SiK > 250) rssi_SiK = 250;
  rssi = rssi_SiK;

  int32_t remrssi_SiK = ( ((int32_t)stats.received_rssi + 127) * 19000 ) / 10000;
  if (remrssi_SiK < 0) remrssi_SiK = 0;
  if (remrssi_SiK > 250) remrssi_SiK = 250;
  remrssi = remrssi_SiK;
*/
  // argh, what a nonsense the sx12xx rssi & snr is
  // so substitute with LQ
  rssi = txstats.GetLQ();
  remrssi = stats.received_LQ;

  txbuf = 100;
  switch (Setup.Tx.SerialDestination) {
    case SERIAL_DESTINATION_SERIAL_PORT:
      txbuf = serial.rx_free_percent();
      break;
    case SERIAL_DESTINATION_MBRDIGE:
      // don't do anything
      break;
  }

  fmav_msg_radio_status_pack(
      &f_msg,
      51, // sysid, SiK uses 51, 68
      MAV_COMP_ID_TELEMETRY_RADIO,
      rssi, remrssi, txbuf, UINT8_MAX, UINT8_MAX, 0, 0,
      //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
      &f_status);
  f_send();
}


// not used
void send_radio_status_v2(void)
{
int8_t rssi = stats.GetLastRxRssi();
int8_t snr = stats.GetLastRxSnr();

  fmav_msg_radio_status_v2_pack(
      &f_msg,
      51, // sysid, SiK uses 51, 68
      MAV_COMP_ID_TELEMETRY_RADIO,
      rssi, (128 + rssi), txstats.GetLQ(), snr,
      stats.received_rssi, (128 + stats.received_rssi), stats.received_LQ, INT8_MAX,
      stats.bytes_transmitted.GetBytesPerSec(), stats.bytes_received.GetBytesPerSec(),
      stats.GetTransmitBandwidthUsage(), stats.GetReceiveBandwidthUsage(),
      //int8_t rssi_dbm, uint8_t rssi_au, uint8_t LQ, int8_t snr_dbm,
      //int8_t rem_rssi_dbm, uint8_t rem_rssi_au, uint8_t rem_LQ, int8_t rem_snr_dbm,
      //uint16_t tx_bps, uint16_t rx_bps, uint8_t tx_used_bandwidth, uint8_t rx_used_bandwidth,
      &f_status);
  f_send();
}


// call at 1 Hz
void f_update_1hz(bool connected)
{
  if (connected) f_inject_radio_status = true;
}


void f_handle_link_receive(char c, tSerialBase* serialport)
{
  if (!serialport) return;

  // send to serial or mbridge
  serialport->putc(c);

  // parse stream, and inject radio status
  uint8_t res = fmav_parse_to_frame_buf(&f_result, f_buf, &f_status, c);

  if (res == FASTMAVLINK_PARSE_RESULT_OK && f_inject_radio_status) { // we have a complete mavlink frame
    f_inject_radio_status = false;
    send_radio_status();
  }
}


#endif // MAVLINK_INTERFACE_TX_H
