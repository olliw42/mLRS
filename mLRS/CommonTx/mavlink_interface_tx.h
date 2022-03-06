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
    void send_msg_serial_out(void);
    void generate_radio_status(void);

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
    if (connected() && Setup.Tx.SendRadioStatus) inject_radio_status = true;
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

    // allow crsf to capture it
    crsf.TelemetryHandleMavlinkMsg(&msg_serial_out);
  }
}


bool MavlinkBase::available(void)
{
  if (!serialport) return false;

  return serialport->available();
}


uint8_t MavlinkBase::getc(void)
{
  if (!serialport) return 0;

  return serialport->getc();
}


void MavlinkBase::flush(void)
{
  if (!serialport) return;

  serialport->flush();
}


void MavlinkBase::send_msg_serial_out(void)
{
  if (!serialport) return;

  uint16_t len = fmav_msg_to_frame_buf(_buf, &msg_serial_out);

  serialport->putbuf(_buf, len);
}


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
  if (Setup.Tx.SendRadioStatus == SEND_RADIO_STATUS_ON_W_TXBUF && serialport) {
    txbuf = serial.rx_free_percent();
  }

  fmav_msg_radio_status_pack(
      &msg_serial_out,
      RADIO_STATUS_SYSTEM_ID, // sysid, SiK uses 51, 68
      MAV_COMP_ID_TELEMETRY_RADIO,
      rssi, remrssi, txbuf, noise, UINT8_MAX, (uint16_t)link_in_missed_packets, 0,
      //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
      &status_link_in_serial_out);
}


#endif // MAVLINK_INTERFACE_TX_H
