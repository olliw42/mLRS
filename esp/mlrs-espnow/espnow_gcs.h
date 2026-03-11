//*******************************************************
// mLRS ESP-NOW
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// GCS Mode — ESP-NOW API bridge (bidirectional serial <-> ESP-NOW)
// Supports ESP32, ESP32C3, ESP32S3
//*******************************************************
// 11. Mar. 2026
//*******************************************************
#ifndef ESPNOW_GCS_H
#define ESPNOW_GCS_H

#include <esp_now.h>


//-------------------------------------------------------
// ESP-NOW receive callback
//-------------------------------------------------------
// the bridge dual-sends: unicast to GCS + broadcast for sniffers.
// we use info->des_addr to drop the broadcast copy.

void espnow_recv_cb(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{
    const uint8_t* sender_mac = info->src_addr;

    if (!espnow_latched_mac_available) {
        memcpy(espnow_latched_mac, sender_mac, 6);
        espnow_latched_mac_available = true;
    } else if (memcmp(sender_mac, espnow_latched_mac, 6) != 0) {
        return; // ignore other senders
    }

    // drop broadcast copies — we already receive the unicast
    if (memcmp(info->des_addr, broadcast_mac, 6) == 0) {
        return;
    }

    rxbuf_push(data, len);
}



//-------------------------------------------------------
// scan hooks (called by scan_for_traffic in .ino)
//-------------------------------------------------------

void scan_reset(void)
{
    if (espnow_latched_mac_available) {
        esp_now_del_peer(espnow_latched_mac);
        espnow_latched_mac_available = false;
        latched_peer_added = false;
    }
}

bool scan_check(void)
{
    while (SERIAL_PORT.available()) SERIAL_PORT.read(); // dump data while disconnected
    return rxbuf_head != rxbuf_tail;
}


#endif // ESPNOW_GCS_H
