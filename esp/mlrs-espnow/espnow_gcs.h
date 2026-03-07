//*******************************************************
// mLRS ESP-NOW
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// GCS Mode — ESP-NOW API bridge (bidirectional serial <-> ESP-NOW)
// Supports ESP32, ESP32C3, ESP32S3
//*******************************************************
// 7. Mar. 2026
//*******************************************************
#ifndef ESPNOW_GCS_H
#define ESPNOW_GCS_H

#include <esp_now.h>


//-------------------------------------------------------
// MAC latch — once a bridge sends us data we lock to its MAC
//-------------------------------------------------------

volatile bool espnow_latched_mac_available;
uint8_t espnow_latched_mac[6];
bool latched_peer_added;


//-------------------------------------------------------
// ESP-NOW receive callback
//-------------------------------------------------------

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void espnow_recv_cb(const uint8_t* mac, const uint8_t* data, int len)
{
    const uint8_t* sender_mac = mac;
#else
void espnow_recv_cb(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{
    const uint8_t* sender_mac = info->src_addr;
#endif
    if (!espnow_latched_mac_available) {
        memcpy(espnow_latched_mac, sender_mac, 6);
        espnow_latched_mac_available = true;
    } else if (memcmp(sender_mac, espnow_latched_mac, 6) != 0) {
        return; // ignore other senders
    }
    rxbuf_push(data, len);
}


//-------------------------------------------------------
// broadcast peer
//-------------------------------------------------------

uint8_t broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };


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


//-------------------------------------------------------
// wifi / ESP-NOW init (GCS mode specific)
//-------------------------------------------------------

void setup_wifi_mode(void)
{
    esp_now_init();

    // register receive callback and add broadcast peer
    esp_now_register_recv_cb(espnow_recv_cb);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    esp_now_add_peer(&peer);

#ifdef WIFI_CHANNEL
    // fixed channel — skip scanning, required for MSP (send/request) systems
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
#else
    scan_for_traffic();
#endif
}


#endif // ESPNOW_GCS_H
