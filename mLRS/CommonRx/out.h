//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// OUT
//********************************************************
#ifndef OUT_H
#define OUT_H
#pragma once


#include <inttypes.h>
#include "../Common/common_types.h"
#include "../Common/frame_types.h"
#include "../Common/setup_types.h"
#include "../Common/channel_order.h"


//-------------------------------------------------------
// Generic Out Class
//-------------------------------------------------------

typedef struct
{
    int8_t receiver_rssi1;
    int8_t receiver_rssi2;
    uint8_t receiver_LQ;
    int8_t receiver_snr;
    uint8_t receiver_antenna;
    uint8_t receiver_transmit_antenna;
    int8_t receiver_power_dbm;
    int8_t transmitter_rssi;
    uint8_t transmitter_LQ;
    int8_t transmitter_snr;
    uint8_t transmitter_antenna;
    uint8_t transmitter_transmit_antenna;
    uint8_t mode;
    uint8_t antenna_config;
} tOutLinkStats;


class tOutBase
{
  public:
    tOutBase(void);

    void Init(tRxSetup* const _setup);

    void Configure(uint8_t new_config);

    void Do(void);

    void SendRcData(tRcData* const rc_orig, bool frame_lost, bool failsafe, int8_t rssi, uint8_t lq);
    void SendLinkStatistics(tOutLinkStats* const lstats);
    void SendLinkStatisticsDisconnected(void);

    void SetChannelOrder(uint8_t new_channel_order);

    tRcData* GetRcDataPtr(void) { return &rc; }

    bool IsRelaySecondary(void) { return (config == OUT_CONFIG_CRSF_TX_JRPIN5); }

  private:
    void send_sbus_rcdata(tRcData* const rc, bool frame_lost, bool failsafe);
    void send_crsf_rcdata(tRcData* const rc);
    void send_crsf_linkstatistics(tOutLinkStats* const lstats);
    void do_crsf(void);

    virtual void putbuf(uint8_t* const buf, uint16_t len) {}
    virtual bool config_sbus(bool enable_flag) { return false; }
    virtual bool config_crsf(bool enable_flag) { return false; }
    virtual bool config_sbus_inverted(bool enable_flag) { return false; }

    tChannelOrder channel_order;
    tRxSetup* setup;
    uint8_t config;
    bool initialized;

    bool link_stats_available;
    bool link_stats_set_tstart;
    uint16_t link_stats_tstart_us;
    tOutLinkStats link_stats;

    tRcData rc;

    // for relay operation

    void send_crsf_tx_rcdata(tRcData* rc);
    void do_crsf_tx_jrpin5(void);
    virtual bool config_crsf_tx_jrpin5(bool enable_flag) { return false; }
};


#endif // OUT_H
