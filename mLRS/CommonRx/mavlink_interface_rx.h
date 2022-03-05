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


#include "..\Common\mavlink\fmav_extension.h"

static inline bool connected(void);

#define RADIO_STATUS_SYSTEM_ID      51 // SiK uses 51, 68

#define MAVLINK_BUF_SIZE            300 // needs to be larger than max mavlink frame size = 286 bytes


class MavlinkBase
{
  public:
    void Init(void);
    void Do(void);
    void putc(char c);
    bool available(void);
    uint8_t getc(void);
    void flush(void);

  private:
    void generate_radio_status(void);
    void send_msg_serial_out(void);

    fmav_status_t status_link_in_serial_out;
    fmav_result_t result_link_in;
    uint8_t buf_link_in[MAVLINK_BUF_SIZE]; // buffer for link in parser
    fmav_message_t msg_serial_out;

    uint8_t _buf[MAVLINK_BUF_SIZE]; // working buffer

    // to count the number of missed mavlink packets received from link
    bool link_in_first_packet_received;
    uint8_t msg_serial_out_seq_last;
    uint32_t link_in_missed_packets;

    // to inject RADIO_STATUS messages
    bool inject_radio_status;
    uint32_t radio_status_tlast_ms;
};


void MavlinkBase::Init(void)
{
  fmav_init();

  result_link_in = {0};
  status_link_in_serial_out = {0};

  inject_radio_status = false;
  radio_status_tlast_ms = millis32() + 1000;

  link_in_first_packet_received = false;
  link_in_missed_packets = 0;
}


void MavlinkBase::Do(void)
{
  uint32_t tnow_ms = millis32();

  if (!connected()) {
    //Init();
    inject_radio_status = false;
    radio_status_tlast_ms = millis32() + 1000;
    link_in_first_packet_received = false;
  }

  if (Setup.Rx.SerialLinkMode != SERIAL_LINK_MODE_MAVLINK) return;

  if ((tnow_ms - radio_status_tlast_ms) >= 1000) {
    radio_status_tlast_ms = tnow_ms;
    if (connected() && Setup.Rx.SendRadioStatus) inject_radio_status = true;
  }

  if (inject_radio_status) { // && serial.tx_is_empty()) {
    inject_radio_status = false;
    generate_radio_status();
    send_msg_serial_out();
  }
}


// houston, we have a problem!
// we do need to do adjust the seq no if we inject messages
// but we only can do this if we have the extra crc, which we won't have for FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN !!
// we then need to reconstruct the extra crc!?
// this is very very costly
// TODO: build up a table of the unknown msgid's

void MavlinkBase::putc(char c)
{
  if (fmav_parse_and_check_to_frame_buf(&result_link_in, buf_link_in, &status_link_in_serial_out, c)) {
    fmav_frame_buf_to_msg(&msg_serial_out, &result_link_in, buf_link_in);

    // we can use seq no to detect missed mavlink packets
    // this assume we do not lose more than 256 packets
    // this also assumes that the seq no is continuous at the tx side, this should be so, but...
    if (!link_in_first_packet_received) {
      link_in_first_packet_received = true;
      msg_serial_out_seq_last = (msg_serial_out.seq - 1);
    }
    uint8_t lost_packets_since_last = msg_serial_out.seq - (msg_serial_out_seq_last + 1);
    msg_serial_out_seq_last = msg_serial_out.seq;
    link_in_missed_packets += lost_packets_since_last;

    // we need to fake the seq no, when we want to inject messages:

    // correct status tx_seq for lost packets, so that it reflects them too
    status_link_in_serial_out.tx_seq += lost_packets_since_last;

    // costly, so do it only if needed
    if (msg_serial_out.seq != status_link_in_serial_out.tx_seq) {
      if (result_link_in.res == FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN) {
        msg_serial_out.crc_extra = fmav_msg_reconstruct_crc_extra(&msg_serial_out); // very costly
      }
      fmav_msg_set_seq(&msg_serial_out, status_link_in_serial_out.tx_seq);
    }
    status_link_in_serial_out.tx_seq++;

    send_msg_serial_out();
  }
}


bool MavlinkBase::available(void)
{
  return serial.available();
}


uint8_t MavlinkBase::getc(void)
{
  return serial.getc();
}


void MavlinkBase::flush(void)
{
  serial.flush();
}


void MavlinkBase::send_msg_serial_out(void)
{
  uint16_t len = fmav_msg_to_frame_buf(_buf, &msg_serial_out);
  serial.putbuf(_buf, len);
}


// ArduPilot's txbuf mechanism
// a variable stream_slowdown_ms is set
// txbuf < 20: stream_slowdown_ms is increased in steps of 60 ms until 2000 ms
// txbuf < 50: stream_slowdown_ms is increased in steps of 20 ms until 2000 ms
// txbuf > 90: stream_slowdown_ms is decreased in steps of 20 ms until 0 ms
// txbuf > 95 && stream_slowdown_ms > 200: stream_slowdown_ms is decreased in steps of 40 ms until 200 ms
// => for txbuf in range [50,89] stream_slowdown_ms is not changed
//
// when params, missions, or mavftp are send, the stream interval are panelized further
// params are send only if txbuf > 50, also baudrate is considered (wrongly)
//
// ArduPilot sends the stream in bursts
// => the current rx buffer filling is not a good indicator, one needs some average
//
// ArduPilot does not route/forward RADIO and RADIO_STATUS messages
//
// messages send by ArduPilot stream, which carries rssi etc
//   RC_CHANNELS_RAW (34), RC_CHANNELS (65)

void MavlinkBase::generate_radio_status(void)
{
uint8_t rssi, remrssi, txbuf, noise;

  rssi = rssi_i8_to_ap(stats.GetLastRxRssi());
  remrssi = rssi_i8_to_ap(stats.received_rssi);

  // we don't have a reasonable noise measurement, but can use this field to report on the snr
  // the snr can be positive and negative however, so we artificially set snr = 10 to zero
  int16_t snr = -stats.GetLastRxSnr() + 10;
  noise = (snr < 0) ? 0 : (snr > 127) ? 127 : snr;

  txbuf = 100;
  if (Setup.Rx.SendRadioStatus == SEND_RADIO_STATUS_ON_W_TXBUF) {
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
  }

  fmav_msg_radio_status_pack(
      &msg_serial_out,
      RADIO_STATUS_SYSTEM_ID, // sysid, SiK uses 51, 68
      MAV_COMP_ID_TELEMETRY_RADIO,
      rssi, remrssi, txbuf, noise, UINT8_MAX, (uint16_t)link_in_missed_packets, 0,
      //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
      &status_link_in_serial_out);
}


#endif // MAVLINK_INTERFACE_RX_H
