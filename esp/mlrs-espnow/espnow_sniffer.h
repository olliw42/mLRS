//*******************************************************
// mLRS ESP-NOW
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Sniffer Mode — standard ESP-NOW API, broadcast-only listener
// Receives the broadcast copy sent by the bridge for passive monitoring.
// Supports ESP32, ESP32C3, ESP32S3.
//*******************************************************
// 11. Mar. 2026
//*******************************************************
#ifndef ESPNOW_SNIFFER_H
#define ESPNOW_SNIFFER_H

#include <esp_now.h>


//-------------------------------------------------------
// ESP-NOW receive callback
//-------------------------------------------------------
// only accepts broadcast-destination frames (the sniffer copy from the bridge).

void espnow_recv_cb(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{
    const uint8_t* sender_mac = info->src_addr;

    // only accept broadcast-destination frames
    if (memcmp(info->des_addr, broadcast_mac, 6) != 0) return;

    // latch to the first sender (the bridge)
    if (!espnow_latched_mac_available) {
        memcpy(espnow_latched_mac, sender_mac, 6);
        espnow_latched_mac_available = true;
    } else if (memcmp(sender_mac, espnow_latched_mac, 6) != 0) {
        return; // ignore other senders
    }

    rxbuf_push(data, len);
}



//-------------------------------------------------------
// scan hooks (called by scan_for_traffic in .ino)
//-------------------------------------------------------

void scan_reset(void)
{
    // sniffer never adds peers, so nothing to delete — just reset latch
    espnow_latched_mac_available = false;
}

bool scan_check(void)
{
    return rxbuf_head != rxbuf_tail;
}


#endif // ESPNOW_SNIFFER_H
