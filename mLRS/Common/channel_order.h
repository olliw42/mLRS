//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// ChannelOrder
//********************************************************
#ifndef CHANNEL_ORDER_H
#define CHANNEL_ORDER_H
#pragma once


#include "../Common/common_types.h"


class tChannelOrder
{
  public:
    enum {
        DIRECTION_TX_TO_MLRS = 0,
        DIRECTION_MLRS_TO_RX,
    };

    tChannelOrder(uint8_t direction);

    void Set(uint8_t new_channel_order);
    void Apply(tRcData* rc);
    uint8_t ChannelMap(uint8_t n);

  private:
    bool direction_is_tx;
    uint8_t channel_order;
    uint8_t channel_map[4];
};


#endif // CHANNEL_ORDER_H
