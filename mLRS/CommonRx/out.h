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
#include "..\Common\frame_types.h"


//-------------------------------------------------------
// Generic Out Class
//-------------------------------------------------------

typedef enum {
  OUT_CONFIG_SBUS = 0,
} OUT_CONFIG_ENUM;


class OutBase
{
  public:
    void Init(void);

    void Configure(uint8_t new_config);

    void SendRcData(tRcData* rc);

    void SetChannelOrder(uint8_t tx_channel_order, uint8_t rx_channel_order);

  private:
    void send_sbus_rcdata(tRcData* rc);
    void putbuf(uint8_t* buf, uint16_t len);

    virtual void putc(char c);
    virtual void config_sbus(void) = 0;

    uint8_t _config;
    uint8_t _rx_channel_order;
    uint8_t _tx_channel_order;
    uint8_t _channel_map[4];
};



#endif // OUT_H
