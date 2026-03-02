//*******************************************************
// mLRS ESP-NOW GCS Bridge
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// GCS-side ESP-NOW bridge companion for mLRS wireless bridge.
// For use with ESP8266, ESP32, ESP32C3 and ESP32S3 modules.
// To use USB on ESP32C3 and ESP32S3, 'USB CDC On Boot' must be enabled in Tools.
//********************************************************
// 2. Mar. 2026
//********************************************************

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <espnow.h>
#else
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#endif


#define BAUD_RATE           115200 // baudrate for serial connection to GCS

// wifi channel — must match WIFI_CHANNEL on the bridge
// required for MSP systems, e.g. INAV
// MSP uses send/request, so the bridge won't transmit until polled
// for MAVLink systems this can be left commented out; scanning will find the bridge
//#define WIFI_CHANNEL        13

//#define USE_SERIAL1                // uncomment to use Serial1 instead of USB Serial for ESP32C3 and ESP32S3
//#define TX_PIN              43     // Serial1 TX pin
//#define RX_PIN              44     // Serial1 RX pin

//#define DEVICE_HAS_SINGLE_LED      // uncomment for single on/off LED
//#define DEVICE_HAS_SINGLE_LED_RGB  // uncomment for single RGB (NeoPixel/WS2812) LED
//#define LED_IO              8      // LED pin (comment out to disable)
//#define LED_ACTIVE_LOW             // uncomment if LED is active low (on = LOW)
//#define RGB_LED_COUNT       1      // number of RGB LEDs (default 1)

#include "leds.h"


#ifdef USE_SERIAL1
  #if !defined(TX_PIN) || !defined(RX_PIN)
    #error TX_PIN and RX_PIN must be defined when USE_SERIAL1 is enabled.
  #endif
  #define SERIAL_PORT  Serial1
#else
  #define SERIAL_PORT  Serial
#endif


//-------------------------------------------------------
// Internals
//-------------------------------------------------------

// ring buffer for esp-now receive callback
#define ESPNOW_RXBUF_SIZE  2048
uint8_t espnow_rxbuf[ESPNOW_RXBUF_SIZE];
volatile uint16_t espnow_rxbuf_head;
volatile uint16_t espnow_rxbuf_tail;

void espnow_rxbuf_push(const uint8_t* data, int len)
{
    for (int i = 0; i < len; i++) {
        uint16_t next = (espnow_rxbuf_head + 1) & (ESPNOW_RXBUF_SIZE - 1);
        if (next == espnow_rxbuf_tail) break; // full, drop
        espnow_rxbuf[espnow_rxbuf_head] = data[i];
        espnow_rxbuf_head = next;
    }
}

int espnow_rxbuf_pop(uint8_t* buf, int maxlen)
{
    int cnt = 0;
    while (espnow_rxbuf_tail != espnow_rxbuf_head && cnt < maxlen) {
        buf[cnt++] = espnow_rxbuf[espnow_rxbuf_tail];
        espnow_rxbuf_tail = (espnow_rxbuf_tail + 1) & (ESPNOW_RXBUF_SIZE - 1);
    }
    return cnt;
}

// mac latch: once a bridge sends us data, we lock to its MAC and register it as a peer
volatile bool espnow_latched_mac_available;
uint8_t espnow_latched_mac[6];
bool latched_peer_added;

#ifdef ESP8266
void espnow_recv_cb(uint8_t* mac, uint8_t* data, uint8_t len)
#elif ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void espnow_recv_cb(const uint8_t* mac, const uint8_t* data, int len)
#else
void espnow_recv_cb(const esp_now_recv_info_t* info, const uint8_t* data, int len)
#endif
{
#ifdef ESP8266
    const uint8_t* sender_mac = mac;
#elif ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    const uint8_t* sender_mac = info->src_addr;
#else
    const uint8_t* sender_mac = mac;
#endif
    if (!espnow_latched_mac_available) {
        memcpy(espnow_latched_mac, sender_mac, 6);
        espnow_latched_mac_available = true;
    } else if (memcmp(sender_mac, espnow_latched_mac, 6) != 0) {
        return; // ignore other senders
    }
    espnow_rxbuf_push(data, len);
}

uint8_t broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// channels used by mLRS
const uint8_t scan_channels[] = { 1, 6, 11, 13 };


bool is_connected;
unsigned long is_connected_tlast_ms;
bool wifi_initialized;
uint8_t buf[250];


//-------------------------------------------------------
// Channel scan
//-------------------------------------------------------

// scan channels until we hear from the bridge
void scan_for_bridge(void)
{
    if (espnow_latched_mac_available) {
        esp_now_del_peer(espnow_latched_mac);
        espnow_latched_mac_available = false;
        latched_peer_added = false;
    }
    while (true) {
        for (int i = 0; i < (int)sizeof(scan_channels); i++) {
#ifdef ESP8266
            wifi_set_channel(scan_channels[i]);
#else
            esp_wifi_set_channel(scan_channels[i], WIFI_SECOND_CHAN_NONE);
#endif

            unsigned long t = millis();
            while (millis() - t < 500) {
                led_tick_scanning();

                while (SERIAL_PORT.available()) SERIAL_PORT.read(); // dump data while disconnected

                if (espnow_rxbuf_head != espnow_rxbuf_tail) {
                    return;
                }
                delay(1);
            }
        }
    }
}


//-------------------------------------------------------
// WiFi / ESP-NOW init
//-------------------------------------------------------

void setup_wifi(void)
{
    // init wifi in STA mode (required for ESP-NOW, no AP association)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

#ifdef ESP8266
    // force 11b only for best reliability
    wifi_set_phy_mode(PHY_MODE_11B);
#else
    // set country to EU to enable channels 1-13 (default may restrict to 1-11)
    wifi_country_t country = { .cc = "EU", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL };
    esp_wifi_set_country(&country);
    // force 11b only for best reliability
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
#endif

    esp_now_init();

    // register receive callback and add broadcast peer
#ifdef ESP8266
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_register_recv_cb(espnow_recv_cb);
    esp_now_add_peer((uint8_t*)broadcast_mac, ESP_NOW_ROLE_COMBO, 0, NULL, 0);
#else
    esp_now_register_recv_cb(espnow_recv_cb);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    esp_now_add_peer(&peer);
#endif

#ifdef WIFI_CHANNEL
    // fixed channel — skip scanning, required for MSP (send/request) systems
#ifdef ESP8266
    wifi_set_channel(WIFI_CHANNEL);
#else
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
#endif
#else
    scan_for_bridge();
#endif
}


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup()
{
    led_init();

#if defined(USE_SERIAL1) && !defined(ESP8266)
    SERIAL_PORT.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
#else
    SERIAL_PORT.begin(BAUD_RATE);
#endif

    espnow_rxbuf_head = 0;
    espnow_rxbuf_tail = 0;
    espnow_latched_mac_available = false;
    latched_peer_added = false;

    is_connected = false;
    is_connected_tlast_ms = 0;
    wifi_initialized = false;
}


void loop()
{
    // defer wifi/esp-now init to first loop iteration
    if (!wifi_initialized) {
        wifi_initialized = true;
        setup_wifi();
        return;
    }

    unsigned long tnow_ms = millis();

    // connection timeout
    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) {
        is_connected = false;
#ifndef WIFI_CHANNEL
        scan_for_bridge();
#endif
    }

    // LED: blink pattern based on connection state
    if (is_connected) {
        led_tick_connected();
    } else {
        led_tick_disconnected();
    }

    // drain received ESP-NOW data to Serial (toward GCS)
    int len = espnow_rxbuf_pop(buf, sizeof(buf));
    if (len > 0) {
        SERIAL_PORT.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = tnow_ms;
    }

    // register latched peer for unicast TX
    if (espnow_latched_mac_available && !latched_peer_added) {
#ifdef ESP8266
        esp_now_add_peer(espnow_latched_mac, ESP_NOW_ROLE_COMBO, 0, NULL, 0);
#else
        esp_now_peer_info_t lpeer = {};
        memcpy(lpeer.peer_addr, espnow_latched_mac, 6);
        esp_now_add_peer(&lpeer);
#endif
        latched_peer_added = true;
    }

    // read Serial (from GCS) and send via ESP-NOW
    int rlen = 0;
    while (SERIAL_PORT.available() && rlen < (int)sizeof(buf)) {
        buf[rlen++] = SERIAL_PORT.read();
    }
    if (rlen > 0) {
        uint8_t* dest = espnow_latched_mac_available ? espnow_latched_mac : broadcast_mac;
        esp_now_send(dest, buf, rlen);
    }

    delay(2);
}
