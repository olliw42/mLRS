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


static inline bool connected(void);


#define RADIO_STATUS_SYSTEM_ID      51 // SiK uses 51, 68

#define MAVLINK_BUF_SIZE            300 // needs to be larger than max mavlink frame size = 286 bytes

uint8_t f_buf_in[MAVLINK_BUF_SIZE];
fmav_result_t f_result;
fmav_status_t f_status;

fmav_message_t f_msg;
uint8_t f_buf[MAVLINK_BUF_SIZE];

bool f_inject_radio_status;
uint32_t f_radio_status_tlast_ms;


void f_init(void)
{
  fmav_init();

  f_result = {0};
  f_status = {0};

  f_inject_radio_status = false;
  f_radio_status_tlast_ms = millis32() + 1000;
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


void f_generate_radio_status(void)
{
uint8_t rssi, remrssi, txbuf;

  rssi = rxstats.GetLQ();
  remrssi = stats.received_LQ;

  rssi = f_convert_rssi_to_ap(stats.GetLastRxRssi());
  remrssi = f_convert_rssi_to_ap(stats.received_rssi);

  // txbuf = serial.rx_free_percent();
  // instead of the true buffer size, we use a smaller virtual buffer size
  // on which we base the estimate
  // the values are purely phenomenological so far, may need more adaption
  // works quite well for me in avoiding stuck, parameters by mavftp or conventional, and missions
  // this mechanism does NOT work well for the constant stream, in as far as the byte rate fluctuates wildly
  // indeed txbuf goes up and down wildly
  uint32_t buf_size = serial.rx_buf_size();
  switch (Setup.Mode) {
  case MODE_50HZ: if (buf_size > 768) buf_size = 768; break; // ca 4100 bytes/s / 5760 bytes/s
  case MODE_31HZ: if (buf_size > 512) buf_size = 512; break;
  case MODE_19HZ: if (buf_size > 256) buf_size = 256; break;
  }

  uint32_t bytes = serial.bytes_available();
  if (bytes >= buf_size) {
    txbuf = 0;
  } else {
    txbuf = (100 * (buf_size - bytes) + buf_size/2) / buf_size;
  }

/*
  // that's how fast bytes can be transported on the link
  // 82 bytes / 20 ms = 4100, 82 bytes / 53 ms = 1547
  uint32_t rx_rate_max = ((uint32_t)1000 * FRAME_RX_PAYLOAD_LEN) / Config.frame_rate_ms;

  // that's how fast bytes come in from the serial
  uint32_t serial_max = Setup.Rx.SerialBaudrate_bytespersec;

  rx_rate_max = (rx_rate_max * 80) / 100; // leave an overhead of 20%
*/

  fmav_msg_radio_status_pack(
      &f_msg,
      RADIO_STATUS_SYSTEM_ID, // sysid, SiK uses 51, 68
      MAV_COMP_ID_TELEMETRY_RADIO,
      rssi, remrssi, txbuf, UINT8_MAX, UINT8_MAX, 0, 0,
      //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
      &f_status);
}


void f_do(void)
{
  uint32_t tnow_ms = millis32();

  if ((tnow_ms - f_radio_status_tlast_ms) >= 1000) {
    f_radio_status_tlast_ms = tnow_ms;
    if (connected() && Setup.Rx.SendRadioStatus) f_inject_radio_status = true;
  }

  if (f_inject_radio_status) { // && serial.tx_is_empty()) {
    f_inject_radio_status = false;
    f_generate_radio_status();
    f_send();
    return;
  }
}


void f_handle_link_receive(char c)
{
  if (fmav_parse_and_check_to_frame_buf(&f_result, f_buf_in, &f_status, c)) {
    fmav_frame_buf_to_msg(&f_msg, &f_result, f_buf_in);
    f_send();
  }
}


#endif // MAVLINK_INTERFACE_RX_H
