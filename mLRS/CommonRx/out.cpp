//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// OUT
//********************************************************


#include <string.h>
#include "out.h"
#include "../Common/thirdparty/thirdparty.h"
#include "../Common/protocols/sbus_protocol.h"
#include "../Common/protocols/crsf_protocol.h"


OutBase::OutBase(void)
    : channel_order(ChannelOrder::DIRECTION_MLRS_TO_RX) // needed to construct channel_order properly
{
}


void OutBase::Init(tRxSetup* _setup)
{
    config = UINT8_MAX;
    initialized = false;

    link_stats_available = false;
    link_stats_set_tstart = false;
    link_stats_tstart_us = 0;

    rc = {0};

    setup = _setup;
}


void OutBase::Configure(uint8_t new_config)
{
    if (new_config == config) return;

    // first disable the previous setting
    switch (config) {
    case OUT_CONFIG_SBUS:
        config_sbus(false);
        break;
    case OUT_CONFIG_SBUS_INVERTED:
        config_sbus_inverted(false);
        break;
    case OUT_CONFIG_CRSF:
        config_crsf(false);
        break;
    }

    initialized = false;

    config = new_config;

    switch (config) {
    case OUT_CONFIG_SBUS:
        initialized = config_sbus(true);
        break;
    case OUT_CONFIG_SBUS_INVERTED:
        initialized = config_sbus_inverted(true);
        break;
    case OUT_CONFIG_CRSF:
        initialized = config_crsf(true);
        break;
    }
}


void OutBase::Do(uint16_t tnow_us)
{
    if (!initialized) return;

    switch (config) {
    case OUT_CONFIG_SBUS:
    case OUT_CONFIG_SBUS_INVERTED:
        // nothing to do
        break;
    case OUT_CONFIG_CRSF:
        do_crsf(tnow_us);
        break;
    }
}


void OutBase::SetChannelOrder(uint8_t new_channel_order)
{
    channel_order.Set(new_channel_order);
}


void OutBase::SendRcData(tRcData* rc_orig, bool frame_lost, bool failsafe, int8_t rssi)
{
    memcpy(&rc, rc_orig, sizeof(tRcData)); // copy rc data, to not modify it !!
    channel_order.Apply(&rc);

    uint8_t failsafe_mode = setup->FailsafeMode;

    if (failsafe) {
        switch (failsafe_mode) {
        case FAILSAFE_MODE_NO_SIGNAL:
            // we do not output anything, so jump out
            return;
        case FAILSAFE_MODE_LOW_THROTTLE:
            // do below
            break;
        case FAILSAFE_MODE_AS_CONFIGURED:
            for (uint8_t n = 0; n < 12; n++) {
              int32_t v = setup->FailsafeOutChannelValues_Ch1_Ch12[n]; // -120 ... +120
              rc.ch[n] = 1024 + (v * 1023) / 120;
            }
            for (uint8_t n = 12; n < 16; n++) {
              int32_t v = setup->FailsafeOutChannelValues_Ch13_Ch16[n-12]; // -120, 0, +120
              rc.ch[n] = (v == 0) ? 1 : (v == 2) ? 2047 : 1024;
            }
            break;
        case FAILSAFE_MODE_LOW_THROTTLE_ELSE_CENTER:
            // do the centering here, throttle is set below
            for (uint8_t n = 0; n < RC_DATA_LEN; n++) rc.ch[n] = 1024;
            break;
        case FAILSAFE_MODE_CH1CH4_CENTER:
            for (uint8_t n = 0; n < 4; n++) rc.ch[n] = 1024; // center all four
            break;
        }
    }

    // mimic spektrum
    // 1090 ... 1515  ... 1940
    // => x' = (1090-1000) * 2048/1000 + 850/1000 * x
    uint32_t t = 85*2048;
    for (uint8_t n = 0; n < RC_DATA_LEN; n++) {
        uint32_t xs = 850 * rc.ch[n];
        rc.ch[n] = (xs + t) / 1000;
    }

    if (failsafe) {
        switch (failsafe_mode) {
        case FAILSAFE_MODE_LOW_THROTTLE:
        case FAILSAFE_MODE_LOW_THROTTLE_ELSE_CENTER:
            rc.ch[channel_order.ChannelMap(2)] = 0; // that's the minimum we can send, gives 905 on ArduPilot
            break;
        case FAILSAFE_MODE_CH1CH4_CENTER:
            // in this mode do not let sbus report bad signal
            // it's a bit an ArduPilot thing, could be achieved by setting RC_OPTIONS 4
            frame_lost = false;
            failsafe = false;
            break;
        }
    }

    if (setup->OutRssiChannelMode >= OUT_RSSI_CHANNEL_CH5 && setup->OutRssiChannelMode <= OUT_RSSI_CHANNEL_CH12) {
        uint8_t rssi_channel = setup->OutRssiChannelMode + 4;
        rc.ch[rssi_channel-1] = rssi_i8_to_ap_sbus(rssi);
    }

    if (!initialized) return;

    switch (config) {
    case OUT_CONFIG_SBUS:
    case OUT_CONFIG_SBUS_INVERTED:
        send_sbus_rcdata(&rc, frame_lost, failsafe);
        break;
    case OUT_CONFIG_CRSF:
        send_crsf_rcdata(&rc);
        break;
    }
}


void OutBase::SendLinkStatistics(tOutLinkStats* lstats)
{
    switch (config) {
    case OUT_CONFIG_SBUS:
    case OUT_CONFIG_SBUS_INVERTED:
        // nothing to send
        break;
    case OUT_CONFIG_CRSF:
        memcpy(&link_stats, lstats, sizeof(tOutLinkStats));
        link_stats_available = true;
        link_stats_set_tstart = true;
        break;
    }
}


void OutBase::SendLinkStatisticsDisconnected(void)
{
    switch (config) {
    case OUT_CONFIG_SBUS:
    case OUT_CONFIG_SBUS_INVERTED:
        break;
    case OUT_CONFIG_CRSF:
        link_stats.receiver_rssi1 = RSSI_MIN;
        link_stats.receiver_rssi2 = RSSI_MIN;
        link_stats.receiver_LQ = 0;
        link_stats.receiver_snr = 0;
        link_stats.receiver_antenna = 0;
        link_stats.receiver_transmit_antenna = 0;
        link_stats.receiver_power_dbm = 0;
        link_stats.transmitter_rssi = RSSI_MIN;
        link_stats.transmitter_LQ = 0;
        link_stats.transmitter_snr = 0;
        link_stats.transmitter_antenna = 0;
        link_stats.transmitter_transmit_antenna = 0;

        link_stats_available = true;
        link_stats_set_tstart = true;
        break;
    }
}


void OutBase::putbuf(uint8_t* buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) putc(buf[i]);
}


//-------------------------------------------------------
// SBus
//-------------------------------------------------------

void OutBase::send_sbus_rcdata(tRcData* rc, bool frame_lost, bool failsafe)
{
tSBusChannelBuffer sbus_buf;

    // chX = (((int32_t)(rc->ch[X]) - 1024) * 1920) / 2047 + 1000;
    sbus_buf.ch0 = rc_to_sbus(rc->ch[0]);
    sbus_buf.ch1 = rc_to_sbus(rc->ch[1]);
    sbus_buf.ch2 = rc_to_sbus(rc->ch[2]);
    sbus_buf.ch3 = rc_to_sbus(rc->ch[3]);
    sbus_buf.ch4 = rc_to_sbus(rc->ch[4]);
    sbus_buf.ch5 = rc_to_sbus(rc->ch[5]);
    sbus_buf.ch6 = rc_to_sbus(rc->ch[6]);
    sbus_buf.ch7 = rc_to_sbus(rc->ch[7]);
    sbus_buf.ch8 = rc_to_sbus(rc->ch[8]);
    sbus_buf.ch9 = rc_to_sbus(rc->ch[9]);
    sbus_buf.ch10 = rc_to_sbus(rc->ch[10]);
    sbus_buf.ch11 = rc_to_sbus(rc->ch[11]);
    sbus_buf.ch12 = rc_to_sbus(rc->ch[12]);
    sbus_buf.ch13 = rc_to_sbus(rc->ch[13]);
    sbus_buf.ch14 = rc_to_sbus(rc->ch[14]);
    sbus_buf.ch15 = rc_to_sbus(rc->ch[15]);

    uint8_t flags = 0;
    if (rc->ch[16] >= 1450) flags |= SBUS_FLAG_CH17; // 1450 = +50%
    if (rc->ch[17] >= 1450) flags |= SBUS_FLAG_CH18;
    if (frame_lost) flags |= SBUS_FLAG_FRAME_LOST;
    if (failsafe) flags |= SBUS_FLAG_FAILSAFE;

    putc(SBUS_STX);
    putbuf(sbus_buf.c, SBUS_CHANNELPACKET_SIZE);
    putc(flags);
    putc(SBUS_END_STX);
}


//-------------------------------------------------------
// Crsf
//-------------------------------------------------------

void OutBase::send_crsf_rcdata(tRcData* rc)
{
tCrsfChannelBuffer crsf_buf;

    // chX = (((int32_t)(rc->ch[X]) - 1024) * 1920) / 2047 + 1000;
    crsf_buf.ch0 = rc_to_crsf(rc->ch[0]);
    crsf_buf.ch1 = rc_to_crsf(rc->ch[1]);
    crsf_buf.ch2 = rc_to_crsf(rc->ch[2]);
    crsf_buf.ch3 = rc_to_crsf(rc->ch[3]);
    crsf_buf.ch4 = rc_to_crsf(rc->ch[4]);
    crsf_buf.ch5 = rc_to_crsf(rc->ch[5]);
    crsf_buf.ch6 = rc_to_crsf(rc->ch[6]);
    crsf_buf.ch7 = rc_to_crsf(rc->ch[7]);
    crsf_buf.ch8 = rc_to_crsf(rc->ch[8]);
    crsf_buf.ch9 = rc_to_crsf(rc->ch[9]);
    crsf_buf.ch10 = rc_to_crsf(rc->ch[10]);
    crsf_buf.ch11 = rc_to_crsf(rc->ch[11]);
    crsf_buf.ch12 = rc_to_crsf(rc->ch[12]);
    crsf_buf.ch13 = rc_to_crsf(rc->ch[13]);
    crsf_buf.ch14 = rc_to_crsf(rc->ch[14]);
    crsf_buf.ch15 = rc_to_crsf(rc->ch[15]);

    uint8_t crc = 0;

    putc(CRSF_ADDRESS_BROADCAST); // or CRSF_ADDRESS_FLIGHT_CONTROLLER ??? what's better?
    putc(CRSF_CHANNELPACKET_SIZE + 2);

    putc(CRSF_FRAME_ID_CHANNELS);
    crc = crc8_calc(crc, CRSF_FRAME_ID_CHANNELS, 0xD5);

    putbuf(crsf_buf.c, CRSF_CHANNELPACKET_SIZE);
    crc = crc8_update(crc, crsf_buf.c, CRSF_CHANNELPACKET_SIZE, 0xD5);

    putc(crc);
}


void OutBase::send_crsf_linkstatistics(tOutLinkStats* lstats)
{
tCrsfLinkStatistics clstats;

    if (lstats->antenna_config == 3) {
        clstats.uplink_rssi1 = crsf_cvt_rssi_rx(lstats->receiver_rssi1);
        clstats.uplink_rssi2 = crsf_cvt_rssi_rx(lstats->receiver_rssi2);
    } else if (lstats->antenna_config == 2) {
        clstats.uplink_rssi1 = 255;
        clstats.uplink_rssi2 = crsf_cvt_rssi_rx(lstats->receiver_rssi2);
    } else {
        clstats.uplink_rssi1 = crsf_cvt_rssi_rx(lstats->receiver_rssi1);
        clstats.uplink_rssi2 = 255;
    }
    clstats.uplink_LQ = lstats->receiver_LQ;
    clstats.uplink_snr = lstats->receiver_snr;
    clstats.active_antenna = lstats->receiver_antenna;
    clstats.mode = crsf_cvt_mode(lstats->mode);
    clstats.uplink_transmit_power = crsf_cvt_power(lstats->receiver_power_dbm);
    clstats.downlink_rssi = crsf_cvt_rssi_rx(lstats->transmitter_rssi);
    clstats.downlink_LQ = lstats->transmitter_LQ;
    clstats.downlink_snr = lstats->transmitter_snr;

    uint8_t crc = 0;

    putc(CRSF_ADDRESS_BROADCAST);
    putc(CRSF_LINK_STATISTICS_LEN + 2);

    putc(CRSF_FRAME_ID_LINK_STATISTICS);
    crc = crc8_calc(crc, CRSF_FRAME_ID_LINK_STATISTICS, 0xD5);

    putbuf((uint8_t*)&clstats, CRSF_LINK_STATISTICS_LEN);
    crc = crc8_update(crc, &clstats, CRSF_LINK_STATISTICS_LEN, 0xD5);

    putc(crc);
}


void OutBase::do_crsf(uint16_t tnow_us)
{
    if (!link_stats_available) return;

    if (link_stats_set_tstart) {
        link_stats_tstart_us = tnow_us;
        link_stats_set_tstart = false;
    }

    uint16_t dt = tnow_us - link_stats_tstart_us;
    if (dt > 4000) {
        link_stats_available = false;
        send_crsf_linkstatistics(&link_stats);
    }
}


//-------------------------------------------------------
// FPort
//-------------------------------------------------------

// FPort, https://github.com/betaflight/betaflight/files/1491056/F.Port.protocol.betaFlight.V2.1.2017.11.21.pdf
// FPort frame: 0x7E , len = 0x19, type = 0x0,  22 bytes channel data , flags byte, rssi byte, crc byte, 0x7E
// 115200 bps, 8N1
// byte stuffing: 0x7E -> 0x7D,0x5E, 0x7D -> 0x7D,0x5D


//-------------------------------------------------------
// IBus
//-------------------------------------------------------

//IBUS frame: 0x20, 0x40, 28 bytes for 14 channels, 2 byte checksum
// 115200 bps, 8N1
// center = 0x05DC. My transmitter sends values between 0x3E8 and 0x7D0


//-------------------------------------------------------
// SUMD
//-------------------------------------------------------

//SUMD, https://www.deviationtx.com/media/kunena/attachments/98/HoTT-SUMD-Spec-REV01-12062012-pdf.pdf
// 115200 bps, 8N1
// 0xA8, 0x01 or 0x00 or 0x81, len byte, u16 x channels bytes, 2 crc byte, telem, crc8



