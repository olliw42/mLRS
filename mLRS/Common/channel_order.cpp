//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// ChannelOrder
//********************************************************

#include "../Common/setup_types.h"
#include "channel_order.h"


tChannelOrder::tChannelOrder(uint8_t direction)
{
    direction_is_tx = (direction == DIRECTION_TX_TO_MLRS);
    channel_order = CHANNEL_ORDER_AETR;
    for (uint8_t n = 0; n < 4; n++) channel_map[n] = n;
}


void tChannelOrder::Set(uint8_t new_channel_order)
{
    if (new_channel_order == channel_order) return;
    channel_order = new_channel_order;

    switch (channel_order) {
        case CHANNEL_ORDER_AETR:
            for (uint8_t n = 0; n < 4; n++) channel_map[n] = n;
            break;
        case CHANNEL_ORDER_TAER:
            if (direction_is_tx) {
                channel_map[0] = 1;
                channel_map[1] = 2;
                channel_map[2] = 0;
                channel_map[3] = 3;
            } else {
                channel_map[0] = 2;
                channel_map[1] = 0;
                channel_map[2] = 1;
                channel_map[3] = 3;
            }
            break;
        case CHANNEL_ORDER_ETAR:
            if (direction_is_tx) {
                channel_map[0] = 2;
                channel_map[1] = 0;
                channel_map[2] = 1;
                channel_map[3] = 3;
            } else {
                channel_map[0] = 1;
                channel_map[1] = 2;
                channel_map[2] = 0;
                channel_map[3] = 3;
            }
            break;
        default:
            while(1){} // must not happen
    }
}


void tChannelOrder::Apply(tRcData* const rc)
{
    uint16_t ch[4] = { // take a copy to swap
        rc->ch[0],
        rc->ch[1],
        rc->ch[2],
        rc->ch[3]
    };

    for (uint8_t n = 0; n < 4; n++) {
        rc->ch[n] = ch[channel_map[n]];
    }
}


uint8_t tChannelOrder::ChannelMap(uint8_t n)
{
    return channel_map[n];
}


