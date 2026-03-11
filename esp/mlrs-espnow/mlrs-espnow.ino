//*******************************************************
// mLRS ESP-NOW
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Combined GCS bridge / sniffer for ESP-NOW.
// Select mode by defining ESPNOW_GCS or ESPNOW_SNIFFER below.
// For use with ESP32, ESP32C3, and ESP32S3.
// To use USB on ESP32C3 and ESP32S3, 'USB CDC On Boot' must be enabled in Tools.
//********************************************************
// 11. Mar. 2026
//********************************************************

//-------------------------------------------------------
// mode selection — uncomment exactly one
//-------------------------------------------------------

#define ESPNOW_GCS       // bidirectional GCS bridge
//#define ESPNOW_SNIFFER   // passive broadcast listener (receive-only)

// validation
#if defined(ESPNOW_GCS) && defined(ESPNOW_SNIFFER)
  #error "define ESPNOW_GCS or ESPNOW_SNIFFER, not both"
#elif !defined(ESPNOW_GCS) && !defined(ESPNOW_SNIFFER)
  #error "define either ESPNOW_GCS or ESPNOW_SNIFFER"
#endif


//-------------------------------------------------------
// user configuration
//-------------------------------------------------------

#define BAUD_RATE           115200 // baudrate for serial connection

// wifi channel — must match WIFI_CHANNEL on the bridge (GCS mode only)
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


//-------------------------------------------------------
// platform includes
//-------------------------------------------------------

#include <WiFi.h>
#include <esp_wifi.h>

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #error "this sketch requires ESP Arduino Core 3.x or later (esp_now_recv_info_t with des_addr)"
#endif


//-------------------------------------------------------
// serial port
//-------------------------------------------------------

#ifdef USE_SERIAL1
  #if !defined(TX_PIN) || !defined(RX_PIN)
    #error TX_PIN and RX_PIN must be defined when USE_SERIAL1 is enabled.
  #endif
  #define SERIAL_PORT  Serial1
#else
  #define SERIAL_PORT  Serial
#endif


//-------------------------------------------------------
// includes (order matters — headers reference globals above)
//-------------------------------------------------------

#include "leds.h"
#include "ring_buffer.h"

// shared ESP-NOW state (used by both GCS and sniffer headers)
volatile bool espnow_latched_mac_available;
uint8_t espnow_latched_mac[6];
uint8_t broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#ifdef ESPNOW_GCS
bool latched_peer_added;
#endif

#ifdef ESPNOW_SNIFFER
#include "espnow_sniffer.h"
#else
#include "espnow_gcs.h"
#endif


//-------------------------------------------------------
// shared wifi init + channel scan
//-------------------------------------------------------

const uint8_t scan_channels[] = { 1, 6, 11, 13 };

void setup_wifi_common(void)
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // set country to EU to enable channels 1-13
    wifi_country_t country = { .cc = "EU", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL };
    esp_wifi_set_country(&country);

    // force 11b only (matches the bridge)
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
}

// unified channel scan — calls mode-specific scan_reset() and scan_check()
void scan_for_traffic(void)
{
    scan_reset();
    while (true) {
        for (int i = 0; i < (int)sizeof(scan_channels); i++) {
            esp_wifi_set_channel(scan_channels[i], WIFI_SECOND_CHAN_NONE);

            unsigned long t = millis();
            while (millis() - t < 500) {
                led_tick_scanning();
                if (scan_check()) return;
                delay(1);
            }
        }
    }
}

void setup_wifi_mode(void)
{
    esp_now_init();
    esp_now_register_recv_cb(espnow_recv_cb);

#ifdef ESPNOW_GCS
    // GCS needs broadcast peer for TX
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    esp_now_add_peer(&peer);

#ifdef WIFI_CHANNEL
    // fixed channel — skip scanning, required for MSP (send/request) systems
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    return;
#endif
#endif

    scan_for_traffic();
}


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

bool wifi_initialized;
bool is_connected;
unsigned long is_connected_tlast_ms;
uint8_t buf[250];


void setup()
{
    led_init();

#if defined(USE_SERIAL1)
    SERIAL_PORT.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
#else
    SERIAL_PORT.begin(BAUD_RATE);
#endif

    rxbuf_init();

    is_connected = false;
    is_connected_tlast_ms = 0;
    wifi_initialized = false;
}


void loop()
{
    // defer wifi init to first loop iteration
    if (!wifi_initialized) {
        wifi_initialized = true;
        setup_wifi_common();
        setup_wifi_mode();
        return;
    }

    unsigned long tnow_ms = millis();

    // connection timeout — rescan if no frames for 2 seconds
    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) {
        is_connected = false;
#if !defined(WIFI_CHANNEL) || defined(ESPNOW_SNIFFER)
        scan_for_traffic();
#endif
    }

    // LED: pattern based on connection state
    if (is_connected) {
        led_tick_connected();
    } else {
        led_tick_disconnected();
    }

    // drain ring buffer to serial
    int len = rxbuf_pop(buf, sizeof(buf));
    if (len > 0) {
        SERIAL_PORT.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = tnow_ms;
    }

#ifdef ESPNOW_GCS
    // register latched peer for unicast TX
    if (espnow_latched_mac_available && !latched_peer_added) {
        esp_now_peer_info_t lpeer = {};
        memcpy(lpeer.peer_addr, espnow_latched_mac, 6);
        esp_now_add_peer(&lpeer);
        latched_peer_added = true;
    }

    // read serial (from GCS) and send via ESP-NOW
    int rlen = 0;
    while (SERIAL_PORT.available() && rlen < (int)sizeof(buf)) {
        buf[rlen++] = SERIAL_PORT.read();
    }
    if (rlen > 0) {
        uint8_t* dest = espnow_latched_mac_available ? espnow_latched_mac : broadcast_mac;
        esp_now_send(dest, buf, rlen);
    }
#endif

    delay(2);
}
