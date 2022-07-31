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


#include "../Common/mavlink/fmav_extension.h"

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

    fmav_status_t status_link_in;
    fmav_result_t result_link_in;
    uint8_t buf_link_in[MAVLINK_BUF_SIZE]; // buffer for link in parser
    fmav_status_t status_serial_out;
    fmav_message_t msg_serial_out;

    uint8_t _buf[MAVLINK_BUF_SIZE]; // working buffer

    // to inject RADIO_STATUS messages
    bool inject_radio_status;
    uint32_t radio_status_tlast_ms;

    uint32_t bytes_serial_in;
};


void MavlinkBase::Init(void)
{
    fmav_init();

    result_link_in = {0};
    status_link_in = {0};
    status_serial_out = {0};

    inject_radio_status = false;
    radio_status_tlast_ms = millis32() + 1000;

    bytes_serial_in = 0;
}


void MavlinkBase::Do(void)
{
    uint32_t tnow_ms = millis32();

    if (!connected()) {
        //Init();
        inject_radio_status = false;
        radio_status_tlast_ms = millis32() + 1000;
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


void MavlinkBase::putc(char c)
{
    if (fmav_parse_and_check_to_frame_buf(&result_link_in, buf_link_in, &status_link_in, c)) {
        fmav_frame_buf_to_msg(&msg_serial_out, &result_link_in, buf_link_in);

        send_msg_serial_out();
    }
}


bool MavlinkBase::available(void)
{
    return serial.available();
}


uint8_t MavlinkBase::getc(void)
{
    bytes_serial_in++;

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


// see design_decissions.h for details
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
        // method C
        uint32_t rate_max = ((uint32_t)1000 * FRAME_RX_PAYLOAD_LEN) / Config.frame_rate_ms;
        uint32_t rate_percentage = (bytes_serial_in * 100) / rate_max;
        if (rate_percentage > 80) {
            txbuf = 0; // +60 ms
        } else if (rate_percentage > 70) {
            txbuf = 30; // +20 ms
        } else if (rate_percentage < 45) {
            txbuf = 100; // -40 ms
        } else if (rate_percentage < 55) {
            txbuf = 91; // -20 ms
        } else {
            txbuf = 60; // no change
        }

        if (serial.bytes_available() > 512) txbuf = 0; // keep the buffer low !!

/*dbg.puts("\nM: ");
dbg.puts(u16toBCD_s(stats.GetTransmitBandwidthUsage()*41));dbg.puts(", ");
dbg.puts(u16toBCD_s(bytes_serial_in));dbg.puts(", ");
dbg.puts(u16toBCD_s(serial.bytes_available()));dbg.puts(", ");
dbg.puts(u8toBCD_s(rate_percentage));dbg.puts(", ");
dbg.puts(u8toBCD_s(txbuf));dbg.puts(", ");
if(txbuf<20) dbg.puts("+60 "); else
if(txbuf<40) dbg.puts("+20 "); else
if(txbuf>95) dbg.puts("-40 "); else
if(txbuf>90) dbg.puts("-20 "); else dbg.puts("+-0 ");*/
    }
    bytes_serial_in = 0; // reset, to restart rate measurement

    fmav_msg_radio_status_pack(
        &msg_serial_out,
        RADIO_STATUS_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO, // sysid, SiK uses 51, 68
        rssi, remrssi, txbuf, noise, UINT8_MAX, 0, 0,
        //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
        &status_serial_out);
}


#endif // MAVLINK_INTERFACE_RX_H
