//*******************************************************
// mLRS Wireless Bridge for ESP32
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Basic but effective & reliable transparent WiFi or Bluetooth <-> serial bridge.
// Minimizes wireless traffic while respecting latency by better packeting algorithm.
//*******************************************************
// 21. Mar. 2026
//*********************************************************/
// inspired by examples from Arduino
// NOTES:
// - For ESP32 and ESP32C3: Partition Scheme needs to be changed to "No OTA (Large APP)" or "No OTA (2MB APP/2MB SPIFFS)" or similar!!
// - Use upload speed 115200 if serial passthrough shall be used for flashing (else 921600 is fine)
// - ArduinoIDE 2.3.2, esp32 by Espressif Systems 3.0.4
// This can be useful: https://github.com/espressif/arduino-esp32/blob/master/libraries
// Dependencies:
// You need to have in File->Preferences->Additional Board managers URLs
// - https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
// - https://arduino.esp8266.com/stable/package_esp8266com_index.json
// Install
// - Boards Manager: esp32 by Espressif Systems (not Arduino ESP32 Boards by Arduino!)
// - Boards Manager: esp8266 by ESP8266 Community
// - Library Manager: Adafruit NeoPixel by Adafruit
// - Library Manager: Preferences by Volodymyr Shymanskyy

/*
Definitions:
- "module" refers to the physical hardware
- "board" refers to the board you need to select in the Arduino IDE menu Tools->Board

For more details on the modules see mlrs-wireless-bridge-boards.h

For a list of supported modules, and board which needs to be selected, see the 
available defines below in the User Configuration - Module section.
Additional comments for some modules:
  - ELRS Tx module ESP82xx backpack, ELRS Tx module ESP32C3 backpack
    ELRS Tx backpacks using ESP82xx or ESP32-C3 must have GPIO0 or GPIO9 controllable, respectively
  - M5Stack M5Stamp C3 Mate
    ATTENTION: when the 5V pin is used, one MUST not also use the USB port, since they are connected internally!!
  - M5Stack M5Stamp C3U Mate
    ATTENTION: when the 5V pin is used, one MUST not also use the USB port, since they are connected internally!!

Troubleshooting:
- If you get error "text section exceeds available space": Set Partition Scheme to "No OTA (Large APP)" or "No OTA (2MB APP/2MB SPIFFS)" or similar
- If flashing is via serial passthrough, you may have to use upload speed 115200
*/

// For ESP8266 this sadly needs to come before User Configuration section
#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#endif


//-------------------------------------------------------
// User configuration
//-------------------------------------------------------

// Module
// uncomment what you want, you must select one (and only one)
// (you also need to set the board in the Arduino IDE accordingly)
//#define MODULE_MATEK_MTX_DB30                 // board: ESP32 PICO-D4
//#define MODULE_ESP82XX_ELRS_TX                // board: Generic ESP8266 Module or Generic ESP8285 Module
//#define MODULE_ESP32C3_ELRS_TX                // board: ESP32C3 Dev Module
//#define MODULE_ESP32_DEVKITC_V4               // board: ESP32 Dev Module
//#define MODULE_NODEMCU_ESP32_WROOM32          // board: ESP32 Dev Module
//#define MODULE_ESP32_PICO_KIT                 // board: ESP32 PICO-D4
//#define MODULE_ADAFRUIT_QT_PY_ESP32_S2        // board: Adafruit QT Py ESP32-S2
//#define MODULE_TTGO_MICRO32                   // board: ESP32 PICO-D4
//#define MODULE_M5STAMP_C3_MATE                // board: ESP32C3 Dev Module
//#define MODULE_M5STAMP_C3U_MATE               // board: ESP32C3 Dev Module
//#define MODULE_M5STAMP_C3U_MATE_FOR_FRSKY_R9M // uses inverted serial
//#define MODULE_M5STAMP_PICO                   // board: ESP32 PICO-D4
//#define MODULE_M5STAMP_PICO_FOR_FRSKY_R9M // uses inverted serial
//#define MODULE_M5STACK_ATOM_LITE              // board: M5Stack-ATOM ??
//#define MODULE_GENERIC
//#define MODULE_DIY_E28DUAL_MODULE02_G491RE    // board: ESP32 PICO-D4, upload speed: 115200

// Serial level
// uncomment, if you need inverted serial for a supported module
// (for the generic module also SERIAL_RXD and SERIAL_TXD need to be defined)
//#define USE_SERIAL_INVERTED

// Wireless protocol
//     0 = WiFi TCP
//     1 = WiFi UDP
//     2 = Wifi UDPSTA
//     3 = Bluetooth (not available for all boards)
//     4 = Wifi UDPCl
//     5 = BLE (not available for all boards)
//     6 = ESP-NOW broadcast
// Note: If GPIO0_IO is defined, then this only sets the default protocol
#define WIRELESS_PROTOCOL  1

// GPIO0 usage
// uncomment if your Tx module supports the RESET and GPIO0 lines on the ESP32/ESP82xx (aka AT mode)
// the number determines the IO pin, usally it is 0
//#define GPIO0_IO  0 // for ESP32, ESP82XX
//#define GPIO0_IO  9 // for ESP32C3


//**********************//
//*** WiFi settings ***//

// For TCP, UDP (only for these two)
// ssid = "" results in a default name, like "mLRS-13427 AP UDP"
// password = "" makes it an open AP
String ssid = ""; // "mLRS AP"; // Wifi name
String password = ""; // "thisisgreat"; // WiFi password (min 8 chars)

IPAddress ip(192, 168, 4, 55); // connect to this IP // MissionPlanner default is 127.0.0.1, so enter 192.168.4.55 in MP

int port_tcp = 5760; // connect to this port per TCP // MissionPlanner default is 5760
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550

// For UDPSTA, UDPCl (only for these two)
// for UDPSTA setting network_ssid = "" results in
// - a default name, like "mLRS-13427 STA UDP"
// - a default password which includes the mLRS bindphrase, like "mLRS-mlrs.0"
// for UDPCl both strings MUST be set to what your Wifi network requires
String network_ssid = ""; // name of your WiFi network
String network_password = ""; // password to access your WiFi network (min 8 chars)

IPAddress ip_udpcl(192, 168, 0, 164); // your network's IP (only for UDPCl) // MissionPlanner default is 127.0.0.1, so enter your home's IP in MP

int port_udpcl = 14550; // listens to this port per UDPCl (only for UDPCl) // MissionPlanner default is 14550

// WiFi channel (only for TCP, UDP)
// choose 1, 6, 11, 13. Channel 13 (2461-2483 MHz) has the least overlap with mLRS 2.4 GHz frequencies.
// Note: Channel 13 is generally not available in the US, where 11 is the maximum.
#define WIFI_CHANNEL  6

// WiFi power (for all TCP, UDP, UDPSTA, UDPCl)
// this sets the power level for the WiFi protocols
// Note: If GPIO0_IO is defined, this sets the power for the medium power option.
#ifndef ESP8266
// Note: In order to find the possible options, right click on WIFI_POWER_19_5dBm and choose "Go To Definiton"
#define WIFI_POWER  WIFI_POWER_2dBm // WIFI_POWER_MINUS_1dBm is the lowest possible, WIFI_POWER_19_5dBm is the max
#else
// Note: WIFI_POWER can be 0 to 20.5
#define WIFI_POWER  6
#endif


//**************************//
//*** Bluetooth settings ***//

// bluetooth_device_name = "" results in a default name, like "mLRS-13427 BT"
String bluetooth_device_name = ""; // name of your Bluetooth device as it will be seen by your operating system


//**************************//
//*** BLE settings ***//

// ble_device_name = "" results in a default name, like "mLRS-13427 BLE"
String ble_device_name = ""; // name of your BLE device as it will be seen by your operating system


//**************************//
//*** General settings ***//

// Baudrate
#define BAUD_RATE  115200

// Serial port usage (only effective for MODULE_GENERIC)
// comment all for default behavior, which is using only Serial port
//#define USE_SERIAL_DBG1 // use Serial for communication and flashing, and Serial1 for debug output
//#define USE_SERIAL1_DBG // use Serial1 for communication, and Serial for debug output and flashing
//#define USE_SERIAL2_DBG // use Serial2 for communication, and Serial for debug output and flashing

// LED pin (only effective for MODULE_GENERIC)
// uncomment if you want a LED, and set the pin number as desired
//#define LED_IO  13


//-------------------------------------------------------
// Version
//-------------------------------------------------------

#define VERSION_STR  "v1.3.09" // to not get version salad, use what the current mLRS version is at the time


//-------------------------------------------------------
// Module details
//-------------------------------------------------------

#include "mlrs-wireless-bridge-boards.h"


//-------------------------------------------------------
// Includes
//-------------------------------------------------------

#if defined GPIO0_IO && (WIRELESS_PROTOCOL != 4) // UDPCl cannot be set via Tx module, so force it
  #define USE_AT_MODE
#endif

#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 0)
    #define USE_WIRELESS_PROTOCOL_TCP
#endif
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 1)
    #define USE_WIRELESS_PROTOCOL_UDP
#endif
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 2)
    #define USE_WIRELESS_PROTOCOL_UDPSTA
#endif
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 4)
    #define USE_WIRELESS_PROTOCOL_UDPCL
#endif

#if defined CONFIG_IDF_TARGET_ESP32 
  #if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    #error Version of your ESP Arduino Core below 3.0.0 !
  #elif ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 7) // 19.Mar.2026
    #pragma message "Warning: Consider upgrading your ESP Arduino Core !" // #warning doesn't show if not verbose, hence #pgrama message
  #endif
#elif defined CONFIG_IDF_TARGET_ESP32C3
  #if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    #error Version of your ESP Arduino Core below 3.0.0 !
  #elif ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 7) // 21.Mar.2026
    #pragma message "Warning: Consider upgrading your ESP Arduino Core !" // #warning doesn't show if not verbose, hence #pragma message
  #endif
#elif defined ESP8266
#else
    #error Must be ESP32, ESP32-C3 or ESP82xx.
#endif

#if defined USE_AT_MODE && (defined CONFIG_IDF_TARGET_ESP32 || defined CONFIG_IDF_TARGET_ESP32C3) && \
    !defined ARDUINO_PARTITION_no_ota
    #error Partition Scheme must be "No OTA (Large APP)", "No OTA (2MB APP/2MB SPIFFS)" or similar!
#endif

#ifndef ESP8266 // not ESP8266
#include <WiFi.h>
#include <esp_mac.h>
#endif
// for some reason checking
// #if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
// does not work here. Also checking e.g. PLATFORM_ESP32_C3 seems not to work.
// This sucks. So we don't try to be nice but let the compiler work it out.
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 3)
  #ifdef CONFIG_IDF_TARGET_ESP32 // classic BT only available on ESP32
    #define USE_WIRELESS_PROTOCOL_BLUETOOTH
    #include <BluetoothSerial.h>
  #endif
#endif
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 5)
  #if defined CONFIG_IDF_TARGET_ESP32 || defined CONFIG_IDF_TARGET_ESP32C3 // BLE available on ESP32 and ESP32C3
    #define USE_WIRELESS_PROTOCOL_BLE
    #include <BLEDevice.h>
    #include <BLEServer.h>
    #include <BLEUtils.h>
    #include <BLE2902.h>
  #endif
#endif
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 6)
  #define USE_WIRELESS_PROTOCOL_ESPNOW
  #ifdef ESP8266
    #include <espnow.h>
  #else
    #include <esp_now.h>
    #include <esp_wifi.h>
  #endif
#endif


//-------------------------------------------------------
// Internals
//-------------------------------------------------------

// TCP, UDP, UDPCl
IPAddress ip_gateway(0, 0, 0, 0);
IPAddress netmask(255, 255, 255, 0);
// TCP
#ifdef USE_WIRELESS_PROTOCOL_TCP
WiFiServer server(port_tcp);
WiFiClient client;
#endif
// UDP, UDPSTA, UDPCl
#if defined USE_WIRELESS_PROTOCOL_UDP || defined USE_WIRELESS_PROTOCOL_UDPSTA || defined USE_WIRELESS_PROTOCOL_UDPCL
WiFiUDP udp;
#endif
// Bluetooth
#ifdef USE_WIRELESS_PROTOCOL_BLUETOOTH
BluetoothSerial SerialBT;
#endif
// BLE
#ifdef USE_WIRELESS_PROTOCOL_BLE
#define BLE_SERVICE_UUID            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // Nordic UART service UUID
#define BLE_CHARACTERISTIC_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHARACTERISTIC_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
BLEServer* ble_server = NULL;
BLECharacteristic* ble_tx_characteristic;
bool ble_device_connected;
bool ble_serial_started;
uint16_t ble_negotiated_mtu;
unsigned long ble_adv_tlast_ms;
extern bool is_connected; // forward declarations, needed for BLE callbacks
extern unsigned long is_connected_tlast_ms; // forward declarations, needed for BLE callbacks

class BLEServerCallbacksHandler : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        ble_device_connected = true;
        ble_negotiated_mtu = pServer->getPeerMTU(pServer->getConnId());
        DBG_PRINTLN("BLE connected");
    }
    void onDisconnect(BLEServer* pServer) {
        ble_device_connected = false;
        ble_serial_started = false;
        DBG_PRINTLN("BLE disconnected");
    }
};

class BLECharacteristicCallbacksHandler : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        if (!ble_serial_started) {
            ble_serial_started = true;
        }
        String rxValue = pCharacteristic->getValue();
        int len = rxValue.length();
        if (len > 0) {
            SERIAL.write((uint8_t*)rxValue.c_str(), len);
            is_connected = true;
            is_connected_tlast_ms = millis();
        }
    }
};

void ble_setup(String device_name) {
    ble_device_connected = false;
    ble_serial_started = false;
    ble_negotiated_mtu = 23;
    ble_adv_tlast_ms = 0;
    // Create BLE Device, set MTU
    BLEDevice::init(device_name.c_str());
    BLEDevice::setMTU(512);
    // Set BLE Power
    BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_DEFAULT);
    // Create BLE server, add callbacks
    ble_server = BLEDevice::createServer();
    ble_server->setCallbacks(new BLEServerCallbacksHandler());
    // Create BLE service
    BLEService* ble_service = ble_server->createService(BLE_SERVICE_UUID);
    // Create BLE characteristics, add callback
    ble_tx_characteristic = ble_service->createCharacteristic(BLE_CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    ble_tx_characteristic->addDescriptor(new BLE2902());
    BLECharacteristic* ble_rx_characteristic = ble_service->createCharacteristic(BLE_CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    ble_rx_characteristic->setCallbacks(new BLECharacteristicCallbacksHandler());
    // Start service
    ble_service->start();
    // Configure advertising
    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    advertising->setMinPreferred(0x12);
    // Start advertising
    advertising->start();
    DBG_PRINTLN("BLE advertising started");
}
#endif // USE_WIRELESS_PROTOCOL_BLE
// ESP-NOW
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW
// ring buffer for esp-now receive callback
#define ESPNOW_RXBUF_SIZE  2048
uint8_t espnow_rxbuf[ESPNOW_RXBUF_SIZE];
volatile uint16_t espnow_rxbuf_head;
volatile uint16_t espnow_rxbuf_tail;
volatile bool espnow_gcs_mac_available; // once a GCS sends us data, we lock to its MAC
uint8_t espnow_gcs_mac[6];
bool espnow_gcs_peer_added;
uint8_t espnow_broadcast_mac[6] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };

void espnow_rxbuf_push(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) {
        uint16_t next = (espnow_rxbuf_head + 1) & (ESPNOW_RXBUF_SIZE - 1);
        if (next == espnow_rxbuf_tail) break; // fifo full, drop
        espnow_rxbuf[espnow_rxbuf_head] = data[i];
        espnow_rxbuf_head = next;
    }
}

int espnow_rxbuf_pop(uint8_t* buf, int maxlen) {
    int cnt = 0;
    while (espnow_rxbuf_tail != espnow_rxbuf_head && cnt < maxlen) {
        buf[cnt++] = espnow_rxbuf[espnow_rxbuf_tail];
        espnow_rxbuf_tail = (espnow_rxbuf_tail + 1) & (ESPNOW_RXBUF_SIZE - 1);
    }
    return cnt;
}

void espnow_recv_callback(const uint8_t* sender_mac, const uint8_t* data, int len) {
    if (!espnow_gcs_mac_available) { // accept only from the first sender we hear from
        memcpy(espnow_gcs_mac, sender_mac, 6);
        espnow_gcs_mac_available = true;
    } else {
        if (memcmp(sender_mac, espnow_gcs_mac, 6) != 0) return; // ignore other senders
    }
    espnow_rxbuf_push(data, len);
}

#ifdef ESP8266
void espnow_recv_cb(uint8_t* mac, uint8_t* data, uint8_t len) { espnow_recv_callback(mac, data, len); }
#else
void espnow_recv_cb(const esp_now_recv_info_t* info, const uint8_t* data, int len) { espnow_recv_callback(info->src_addr, data, len); }
#endif

void espnow_add_peer_mac(uint8_t* mac, int wifi_channel) {
#ifdef ESP8266
    esp_now_add_peer(mac, ESP_NOW_ROLE_COMBO, wifi_channel, NULL, 0);
#else
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = wifi_channel;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
#endif
}

void espnow_setup(int wifi_channel) {
    espnow_rxbuf_head = espnow_rxbuf_tail = 0;
    espnow_gcs_mac_available = false;
    espnow_gcs_peer_added = false;
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
#ifdef ESP8266
    wifi_set_phy_mode(PHY_MODE_11B); // force 11b only for best reliability
    wifi_set_channel(wifi_channel);
#else
    // set country to EU to enable channels 1-13 (default may restrict to 1-11)
    wifi_country_t country = { .cc = "EU", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL };
    esp_wifi_set_country(&country);
    esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B); // force 11b only for best reliability
#endif
    setup_wifipower();
    if (esp_now_init() != 0) {
        DBG_PRINTLN("ESP-NOW init failed");
        return;
    }
#ifdef ESP8266
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
#endif
    esp_now_register_recv_cb(espnow_recv_cb);
    espnow_add_peer_mac(espnow_broadcast_mac, wifi_channel);
    DBG_PRINTLN("ESP-NOW started");
}

void espnow_send(int wifi_channel, uint8_t* buf, int len) {
    if (espnow_gcs_mac_available) { // if gcs seen, send unicast; otherwise broadcast
        if (!espnow_gcs_peer_added) { // ensure latched peer is registered
            espnow_add_peer_mac(espnow_gcs_mac, wifi_channel);
            espnow_gcs_peer_added = true;
        }
        esp_now_send(espnow_gcs_mac, buf, len);
    } else {
        esp_now_send(espnow_broadcast_mac, buf, len);
    }
}
#endif // USE_WIRELESS_PROTOCOL_ESPNOW

typedef enum {
    WIRELESS_PROTOCOL_TCP = 0,
    WIRELESS_PROTOCOL_UDP = 1,
    WIRELESS_PROTOCOL_UDPSTA = 2,
    WIRELESS_PROTOCOL_BT = 3,
    WIRELESS_PROTOCOL_UDPCl = 4,
    WIRELESS_PROTOCOL_BLE = 5,
    WIRELESS_PROTOCOL_ESPNOW = 6,
} WIRELESS_PROTOCOL_ENUM;

typedef enum {
    WIFIPOWER_LOW = 0,
    WIFIPOWER_MED,
    WIFIPOWER_MAX,
} WIFIPOWER_ENUM;

#define PROTOCOL_DEFAULT  WIRELESS_PROTOCOL
#define BAUDRATE_DEFAULT  BAUD_RATE
#define WIFICHANNEL_DEFAULT  WIFI_CHANNEL
#define WIFIPOWER_DEFAULT  WIFIPOWER_MED

#define G_PROTOCOL_STR  "protocol"
int g_protocol = PROTOCOL_DEFAULT;
#define G_BAUDRATE_STR  "baudrate"
int g_baudrate = BAUDRATE_DEFAULT;
#define G_WIFICHANNEL_STR  "wifichannel"
int g_wifichannel = WIFICHANNEL_DEFAULT;
#define G_WIFIPOWER_STR  "wifipower"
int g_wifipower = WIFIPOWER_DEFAULT;
#define G_BINDPHRASE_STR  "bindphrase"
String g_bindphrase = "mlrs.0";
#define G_PASSWORD_STR  "password"
String g_password = "";
#define G_NETWORK_SSID_STR  "network_ssid"
String g_network_ssid = "";

uint16_t device_id = 0; // is going to be set by setup_device_name_and_password(), and can be queried in at mode
String device_name = "";
String device_name_STAUDP;
String device_password = "";

#ifdef USE_AT_MODE
#include <Preferences.h>
Preferences preferences;
#include "mlrs-wireless-bridge-at-mode.h"
AtMode at_mode;
#endif

bool led_state;
unsigned long led_tlast_ms;
bool is_connected;
unsigned long is_connected_tlast_ms;


void serialFlushRx(void)
{
    while (SERIAL.available() > 0) { SERIAL.read(); }
}


//-------------------------------------------------------
// Clients list (for UDP)
//-------------------------------------------------------
#ifdef USE_WIRELESS_PROTOCOL_UDP

#define UDP_CLIENTS_COUNT_MAX  3

class tClientList {
  public:
    struct tUdpClient {
        IPAddress ip;
        int port;
    };

    void Init(void) {
        clients_cnt = 0;
        for (int i = 0; i < UDP_CLIENTS_COUNT_MAX; i++) {
            clients[i].port = -1; // indicates that it is empty
        }
        gcs_seen = false;
    }

    void Add(IPAddress ip, int port, bool is_gcs) {
        for (int i = 0; i < clients_cnt; i++) {
            if (clients[i].ip == ip && clients[i].port == port) return; // found, is already in list
        }
        for (int i = 0; i < UDP_CLIENTS_COUNT_MAX; i++) {
            if (clients[i].port < 0) { // empty spot found
                clients[i].ip = ip;
                clients[i].port = port;
                clients_cnt++;
                if (is_gcs) gcs_seen = true;
                return; // added
            }
        }
    }

    bool HasGcs(void) {
        return gcs_seen;
    }

    int clients_cnt;
    tUdpClient clients[UDP_CLIENTS_COUNT_MAX];
    bool gcs_seen;
};

#endif // USE_WIRELESS_PROTOCOL_UDP


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup_wifipower()
{
#ifndef ESP8266
    switch (g_wifipower) {
        case WIFIPOWER_LOW: WiFi.setTxPower(WIFI_POWER_MINUS_1dBm); break;
#ifdef WIFI_POWER
        case WIFIPOWER_MED: WiFi.setTxPower(WIFI_POWER); break;
#else
        case WIFIPOWER_MED: WiFi.setTxPower(WIFI_POWER_5dBm); break;
#endif
        case WIFIPOWER_MAX: WiFi.setTxPower(WIFI_POWER_19_5dBm); break;
    }
#else
    switch (g_wifipower) {
        case WIFIPOWER_LOW: WiFi.setOutputPower(0); break;
#ifdef WIFI_POWER
        case WIFIPOWER_MED: WiFi.setOutputPower(WIFI_POWER); break;
#else
        case WIFIPOWER_MED: WiFi.setOutputPower(5); break;
#endif
        case WIFIPOWER_MAX: WiFi.setOutputPower(20.5); break;
    }
#endif
}


void setup_ap_mode(IPAddress __ip)
{
    WiFi.mode(WIFI_AP); // seems not to be needed, done by WiFi.softAP()?
#ifndef ESP8266
    // set country to EU to enable channels 1-13 (default may restrict to 1-11)
    wifi_country_t country = { .cc = "EU", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL };
    esp_wifi_set_country(&country);
#endif
    WiFi.softAPConfig(__ip, ip_gateway, netmask);
    WiFi.softAP(device_name.c_str(), (device_password.length()) ? device_password.c_str() : NULL, g_wifichannel); // channel = 1 is default
    DBG_PRINT("ap ip address: ");
    DBG_PRINTLN(WiFi.softAPIP()); // comes out as 192.168.4.1
    DBG_PRINT("channel: ");
    DBG_PRINTLN(WiFi.channel());
}


// true: has connected, false: not yet connected, retry
bool setup_sta_mode_nonblocking(bool first, bool config_ip, IPAddress ip)
{
static unsigned long tlast_ms;

    if (first) {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        if (config_ip) {
            WiFi.config(ip, ip_gateway, netmask);
        }
        WiFi.begin(device_name.c_str(), device_password.c_str());
        tlast_ms = millis();
    }
    if (WiFi.status() == WL_CONNECTED) {
        DBG_PRINTLN("connected");
        DBG_PRINT("network ip address: ");
        DBG_PRINTLN(WiFi.localIP());
        return true;
    }
    if (millis() > tlast_ms + 1000) {
        tlast_ms = millis();
        DBG_PRINTLN("connecting to WiFi network...");
    }
    return false;
}


//-------------------------------------------------------
// Wifi Classes
//-------------------------------------------------------

//-------------------------------------------------------
//-- Wifi Base class
// note: g_ sould be already set up

class tWifiHandler {
  public:
    IPAddress _ip;
    int _port;
    unsigned long serial_data_received_tfirst_ms;
    int _setup_state; // 0: first call, 1: trying to connect, 2: done, some need a state machine

    void Init() {
        serial_data_received_tfirst_ms = 0;
        _setup_state = 0;

        uint8_t MAC_buf[6+2];
        // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/misc_system_api.html#mac-address
        // MACs are different for STA and AP, BT
#ifdef ESP8266
       wifi_get_macaddr(STATION_IF, MAC_buf);
#else
        esp_efuse_mac_get_default(MAC_buf);
#endif
        for (uint8_t i = 0; i < 5; i++) device_id += MAC_buf[i] + ((uint16_t)MAC_buf[i + 1] << 8) / 39;
        device_id += MAC_buf[5];
        device_name = "mLRS-" + String(device_id);
#ifdef DEVICE_NAME_HEAD
        device_name = String(DEVICE_NAME_HEAD) + "-mLRS-" + String(device_id);
#endif
        // set STAUDP devicename here, as it may be needed in AT
        if (g_network_ssid != "") { // definition in memory overwrites default
            device_name_STAUDP = g_network_ssid;
        } else { // we don't have any so set a default
            device_name_STAUDP = device_name + " STA UDP";
        }
    }

    void set_device_password(String forced_password, String std_password) {
        if (forced_password != "") {
            device_password = forced_password;
        } else if (g_password != "") {
            device_password = g_password;
        } else {
            device_password = std_password;
        }
    }

    void set_connected() {
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    void serial_read_wifi_write(uint8_t* buf, int sizeofbuf) {
        unsigned long tnow_ms = millis();
        int avail = SERIAL.available();
        if (avail <= 0) {
            serial_data_received_tfirst_ms = tnow_ms;
        } else
        if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
            serial_data_received_tfirst_ms = tnow_ms;
            int len = SERIAL.read(buf, sizeofbuf);
            wifi_write(buf, len);
        }
    }

    virtual void wifi_setup() {}
    virtual void wifi_write(uint8_t* buf, int len) {}

    void set_wifi_setup_trying() { _setup_state = 1; }
    void set_wifi_setup_done() { _setup_state = 2; }

    bool Setup() { // true: setup has completed and is not called anymore
        if (_setup_state >= 2) return true;
        wifi_setup();
        if (_setup_state == 1) return false;
        _setup_state = 2; 
        return true;
    }

    bool IsSetUp() {
        return (_setup_state >= 2);
    }

    virtual void Loop(uint8_t* buf, int sizeofbuf) {}
};

tWifiHandler* wifi_handler;


//-------------------------------------------------------
//-- TCP class
#ifdef USE_WIRELESS_PROTOCOL_TCP

class tTCPHandler : public tWifiHandler {
  public:
    void Init(IPAddress __ip) {
        tWifiHandler::Init();
        device_name = (ssid != "") ? ssid : device_name + " AP TCP";
        set_device_password(password, "");
        _ip = __ip;
    }

    void wifi_setup() override {
        setup_ap_mode(_ip); // AP mode
        setup_wifipower();
        server.begin();
        server.setNoDelay(true);
        set_wifi_setup_done();
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        if (server.hasClient()) {
            if (!client.connected()) {
                client.stop(); // doesn't appear to make a difference
                client = server.available();
                DBG_PRINTLN("connection");
            } else { // is already connected, so reject, doesn't seem to ever happen
                server.available().stop();
                DBG_PRINTLN("connection rejected");
            }
        }

        if (!client.connected()) { // nothing to do
            client.stop();
            serialFlushRx();
            is_connected = false;
            return;
        }

        while (client.available()) {
            int len = client.read(buf, sizeofbuf);
            SERIAL.write(buf, len);
            set_connected();
        }

        serial_read_wifi_write(buf, sizeofbuf);
    }

    void wifi_write(uint8_t* buf, int len) override {
        client.write(buf, len);
    }
};
tTCPHandler tcp_handler;
#endif


//-------------------------------------------------------
//-- UDP class
#ifdef USE_WIRELESS_PROTOCOL_UDP

class tUDPHandler : public tWifiHandler, tClientList {
  public:
    IPAddress _ip_ap;

    void Init(IPAddress __ip, int __port) {
        tWifiHandler::Init();
        tClientList::Init();
        device_name = (ssid != "") ? ssid : device_name + " AP UDP";
        set_device_password(password, "");
        _ip = _ip_ap = __ip; 
        //_ip = WiFi.broadcastIP(); // seems to not work for AP mode
        _ip[3] = 255; // start with broadcast, the subnet mask is 255.255.255.0 so just last octet needs to change
        _port = __port;

    }

    void wifi_setup() override {
        setup_ap_mode(_ip_ap); // AP mode
        setup_wifipower();
        udp.begin(_port);
        set_wifi_setup_done();
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
            int len = udp.read(buf, sizeofbuf);
            if (len > 0) { // let's assume that this is the GCS, so forward
                SERIAL.write(buf, len);
            }
            Add(udp.remoteIP(), udp.remotePort(), (len > 0)); // true if it's from a GCS
            set_connected(); // should we indicate connected only if we have seen a GCS?
        }
        serial_read_wifi_write(buf, sizeofbuf);
    }

    void wifi_write(uint8_t* buf, int len) override {
        if (!HasGcs()) {
            udp.beginPacket(_ip, _port);
            udp.write(buf, len);
            udp.endPacket();
        } else
        for (int i = 0; i < clients_cnt; i++) {
            udp.beginPacket(clients[i].ip, clients[i].port);
            udp.write(buf, len);
            udp.endPacket();
        }
    }
};
tUDPHandler udp_handler;
#endif


//-------------------------------------------------------
//-- UDPSTA class
// network_ssid, network_password
// g_bindphrase
#ifdef USE_WIRELESS_PROTOCOL_UDPSTA

class tUDPSTAHandler : public tWifiHandler {
  public:
    int _initial_port;

    void Init(int __port) {
        tWifiHandler::Init();
        device_name = (network_ssid != "") ? network_ssid : device_name_STAUDP;
        set_device_password(network_password, String("mLRS-") + g_bindphrase);
        _ip = WiFi.broadcastIP(); // start with broadcast
        _port = _initial_port = __port;
    }

    void wifi_setup() override {
        bool res = setup_sta_mode_nonblocking((_setup_state == 0), false, IPAddress()); // STA mode, without config ip, so dummy ip
        set_wifi_setup_trying(); // switch to trying
        if (res) { // done
            setup_wifipower();
            udp.begin(_port);
            set_wifi_setup_done(); // we are actually connected, so signal done
        }
    }

    void Loop(uint8_t* buf, int sizeofbuf)  override {
        if (!is_connected && WiFi.status() != WL_CONNECTED) {
            udp.stop();
            _port = _initial_port;
            _setup_state = 0; // attempt to reconnect if WiFi got disconnected
            return;
        }

        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
            int len = udp.read(buf, sizeofbuf);
            SERIAL.write(buf, len);
            if (!is_connected) { // first received UDP packet
                _ip = udp.remoteIP(); // stop broadcast, switch to unicast to avoid Aurdino performance issue
                _port = udp.remotePort();
            }
            set_connected();
        }

        serial_read_wifi_write(buf, sizeofbuf);
    }

    void wifi_write(uint8_t* buf, int len) override {
        udp.beginPacket(_ip, _port);
        udp.write(buf, len);
        udp.endPacket();
    }
};
tUDPSTAHandler udpsta_handler;
#endif


//-------------------------------------------------------
//-- UDPCl class
#ifdef USE_WIRELESS_PROTOCOL_UDPCL

class tUDPClHandler : public tWifiHandler {
  public:
    void Init(IPAddress __ip, int __port) {
        tWifiHandler::Init();
        device_name = network_ssid; // we only allow that specified in code
        device_password = network_password; // we only allow that specified in code
        _ip = __ip;
        _port = __port;
    }

    void wifi_setup() override {
        bool res = setup_sta_mode_nonblocking((_setup_state == 0), true, _ip); // STA mode, with config ip
        set_wifi_setup_trying(); // switch to trying
        if (res) { // done
            setup_wifipower();
            udp.begin(_port);
            set_wifi_setup_done(); // we are actually connected, so signal done
        }
    }

    void Loop(uint8_t* buf, int sizeofbuf)  override {
        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
            int len = udp.read(buf, sizeofbuf);
            SERIAL.write(buf, len);
            set_connected();
        }

        if (!is_connected) {
            // we wait for a first message from the remote
            // remote's ip and port not known, so jump out
            serialFlushRx();
            return;
        }

        serial_read_wifi_write(buf, sizeofbuf);
    }

    void wifi_write(uint8_t* buf, int len) override {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write(buf, len);
        udp.endPacket();
    }
};
tUDPClHandler udpcl_handler;
#endif


//-------------------------------------------------------
//-- BLUETOOTH class
#ifdef USE_WIRELESS_PROTOCOL_BLUETOOTH
// Comment: CONFIG_BT_SSP_ENABLED appears to be defined per default, so setPin() is not available

class tBTClassicHandler : public tWifiHandler {
  public:
    void Init() {
        tWifiHandler::Init();
        device_name = (bluetooth_device_name != "") ? bluetooth_device_name : device_name + " BT";
    }
    
    void wifi_setup() override {
        SerialBT.begin(device_name);
        set_wifi_setup_done();
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        int len = SerialBT.available();
        if (len > 0) {
            if (len > sizeofbuf) len = sizeofbuf;
            for (int i = 0; i < len; i++) buf[i] = SerialBT.read();
            SERIAL.write(buf, len);
            set_connected();
        }
        serial_read_wifi_write(buf, sizeofbuf);
    }

    void wifi_write(uint8_t* buf, int len) override {
        SerialBT.write(buf, len);
    }
};
tBTClassicHandler bt_handler;
#endif


//-------------------------------------------------------
//-- BLE class
#ifdef USE_WIRELESS_PROTOCOL_BLE

class tBLEHandler : public tWifiHandler {
  public:
    void Init() {
        tWifiHandler::Init();
        device_name = (ble_device_name != "") ? ble_device_name : device_name + " BLE";
    }

    void wifi_setup() override {
        ble_setup(device_name);
        set_wifi_setup_done();
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        unsigned long tnow_ms = millis();
        if (ble_device_connected) {
            int avail = SERIAL.available();
            if (avail <= 0) {
                serial_data_received_tfirst_ms = tnow_ms;
            } else
            if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) {
                serial_data_received_tfirst_ms = tnow_ms;
                uint16_t bytesToRead = (ble_negotiated_mtu - 3);
                if (bytesToRead < sizeofbuf) sizeofbuf = bytesToRead; // limit number of bytes to read to MTU - 3
                int len = SERIAL.read(buf, sizeofbuf);
                wifi_write(buf, len);
            }
        } else {
            serialFlushRx();
            is_connected = false;
            if (tnow_ms - ble_adv_tlast_ms > 5000) { // not connected, restart advertising every 5 sec if needed
                ble_adv_tlast_ms = tnow_ms;
                ble_server->startAdvertising();
            }
        }
    }

    void wifi_write(uint8_t* buf, int len) override {
        ble_tx_characteristic->setValue(buf, len);
        ble_tx_characteristic->notify();
    }
};
tBLEHandler ble_handler;

#endif // USE_WIRELESS_PROTOCOL_BLE


//-------------------------------------------------------
//-- ESPNOW class
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW

class tESPNOWHandler : public tWifiHandler {
  public:
    void Init() {
        tWifiHandler::Init();
        device_name = device_name + " ESPNOW";
    }

    void wifi_setup() override {
        espnow_setup(g_wifichannel);
        set_wifi_setup_done();
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        if (sizeofbuf > 250) sizeofbuf = 250; // cap at 250 bytes (esp-now max payload)
        int len = espnow_rxbuf_pop(buf, sizeofbuf);
        if (len > 0) {
            SERIAL.write(buf, len);
            set_connected();
        }
        serial_read_wifi_write(buf, sizeofbuf);
    }

    void wifi_write(uint8_t* buf, int len) override {
        espnow_send(g_wifichannel, buf, len);
    }
};
tESPNOWHandler espnow_handler;

#endif // USE_WIRELESS_PROTOCOL_ESPNOW


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup()
{
    led_init();
    dbg_init();
    //delay(500); // we delay for 750 ms anyway

    // Preferences
#ifdef USE_AT_MODE
    preferences.begin("setup", false);

    g_protocol = preferences.getInt(G_PROTOCOL_STR, 255); // 255 indicates not available
    if (g_protocol != WIRELESS_PROTOCOL_TCP && g_protocol != WIRELESS_PROTOCOL_UDP && g_protocol != WIRELESS_PROTOCOL_UDPSTA &&
        g_protocol != WIRELESS_PROTOCOL_UDPCl && g_protocol != WIRELESS_PROTOCOL_BT && g_protocol != WIRELESS_PROTOCOL_BLE &&
        g_protocol != WIRELESS_PROTOCOL_ESPNOW) { // not a valid value
        g_protocol = PROTOCOL_DEFAULT;
        preferences.putInt(G_PROTOCOL_STR, g_protocol);
    }

    g_baudrate = preferences.getInt(G_BAUDRATE_STR, 0); // 0 indicates not available
    if (g_baudrate != 9600 && g_baudrate != 19200 && g_baudrate != 38400 &&
        g_baudrate != 57600 && g_baudrate != 115200 && g_baudrate != 230400) { // not a valid value
        g_baudrate = BAUDRATE_DEFAULT;
        preferences.putInt(G_BAUDRATE_STR, g_baudrate);
    }

    g_wifichannel = preferences.getInt(G_WIFICHANNEL_STR, 0); // 0 indicates not available
    if (g_wifichannel != 1 && g_wifichannel != 6 && g_wifichannel != 11 && g_wifichannel != 13) { // not a valid value
        g_wifichannel = WIFICHANNEL_DEFAULT;
        preferences.putInt(G_WIFICHANNEL_STR, g_wifichannel);
    }

    g_wifipower = preferences.getInt(G_WIFIPOWER_STR, 255); // 255 indicates not available
    if (g_wifipower < WIFIPOWER_LOW || g_wifipower > WIFIPOWER_MAX) { // not a valid value
        g_wifipower = WIFIPOWER_DEFAULT;
        preferences.putInt(G_WIFIPOWER_STR, g_wifipower);
    }

    g_bindphrase = preferences.getString(G_BINDPHRASE_STR, "mlrs.0"); // "mlrs.0" is the mLRS default bind phrase
    // TODO: we should check for sanity

    g_password = preferences.getString(G_PASSWORD_STR, ""); // "" is the default password
    g_network_ssid = preferences.getString(G_NETWORK_SSID_STR, ""); // "" is the default network ssid
#endif

    // Wifi handler
    switch (g_protocol) {
#ifdef USE_WIRELESS_PROTOCOL_TCP
        case WIRELESS_PROTOCOL_TCP: tcp_handler.Init(ip); wifi_handler = &tcp_handler; break;
#endif
#ifdef USE_WIRELESS_PROTOCOL_UDP
        case WIRELESS_PROTOCOL_UDP: udp_handler.Init(ip, port_udp); wifi_handler = &udp_handler; break;
#endif
#ifdef USE_WIRELESS_PROTOCOL_UDPSTA
        case WIRELESS_PROTOCOL_UDPSTA: udpsta_handler.Init(port_udp); wifi_handler = &udpsta_handler; break;
#endif
#ifdef USE_WIRELESS_PROTOCOL_UDPCL
        case WIRELESS_PROTOCOL_UDPCl: udpcl_handler.Init(ip_udpcl, port_udpcl); wifi_handler = &udpcl_handler; break;
#endif
#ifdef USE_WIRELESS_PROTOCOL_BLUETOOTH
        case WIRELESS_PROTOCOL_BT: bt_handler.Init(); wifi_handler = &bt_handler; break;
#endif
#ifdef USE_WIRELESS_PROTOCOL_BLE
        case WIRELESS_PROTOCOL_BLE: ble_handler.Init(); wifi_handler = &ble_handler; break;
#endif
#ifdef USE_WIRELESS_PROTOCOL_ESPNOW
        case WIRELESS_PROTOCOL_ESPNOW: espnow_handler.Init(); wifi_handler = &espnow_handler; break;
#endif
    }

    // Serial
    size_t rxbufsize = SERIAL.setRxBufferSize(2*1024); // must come before uart started, retuns 0 if it fails
#ifndef ESP8266 // not implemented on ESP8266
    size_t txbufsize = SERIAL.setTxBufferSize(512); // must come before uart started, retuns 0 if it fails
#endif
#ifdef SERIAL_RXD // if SERIAL_TXD is not defined the compiler will complain, so all good
  #ifdef USE_SERIAL_INVERTED
    SERIAL.begin(g_baudrate, SERIAL_8N1, SERIAL_RXD, SERIAL_TXD, true);
  #else
    SERIAL.begin(g_baudrate, SERIAL_8N1, SERIAL_RXD, SERIAL_TXD);
  #endif
#else
    SERIAL.begin(g_baudrate);
#endif
//????used to work    pinMode(U1_RXD, INPUT_PULLUP); // important, at least in older versions Arduino serial lib did not do it

    DBG_PRINTLN(rxbufsize);
    DBG_PRINTLN(txbufsize);
    DBG_PRINTLN(g_protocol);
    DBG_PRINTLN(device_name);
    //DBG_PRINTLN(device_password);
    if (!wifi_handler) { DBG_PRINTLN("No protocol selected"); while(1){} }

    // Gpio0 handling
#ifdef USE_AT_MODE
    at_mode.Init(GPIO0_IO);
#endif

    led_tlast_ms = 0;
    led_state = false;

    is_connected = false;
    is_connected_tlast_ms = 0;

    serialFlushRx();
}


void loop()
{
#ifdef USE_AT_MODE
    if (at_mode.Do()) return;
#endif
    unsigned long tnow_ms = millis();

    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) { // nothing from GCS for 2 secs
        is_connected = false;
    }

    if (tnow_ms - led_tlast_ms > (is_connected ? 500 : (wifi_handler->IsSetUp()) ? 200 : 75)) {
        led_tlast_ms = tnow_ms;
        led_state = !led_state;
        if (led_state) led_on(is_connected); else led_off();
    }

    //-- here comes the core code, handle WiFi or Bluetooth connection and do the bridge

    uint8_t buf[256]; // working buffer

    if (!wifi_handler->Setup()) {
        return;
    }

    wifi_handler->Loop(buf, sizeof(buf));

    delay(2); // give it always a bit of time
}

