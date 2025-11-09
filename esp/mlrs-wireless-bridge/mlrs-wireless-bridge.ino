//*******************************************************
// mLRS Wireless Bridge for ESP32
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Basic but effective & reliable transparent WiFi or Bluetooth <-> serial bridge.
// Minimizes wireless traffic while respecting latency by better packeting algorithm.
//*******************************************************
// 10. Mai. 2025
//*********************************************************/
// inspired by examples from Arduino
// NOTES:
// - For ESP32: Partition Scheme needs to be changed to "No OTA (Large App)" !!
// - Use upload speed 115200 if serial passthrough shall be used for flashing (else 921600 is fine)
// - ArduinoIDE 2.3.2, esp32 by Espressif Systems 3.0.4
// This can be useful: https://github.com/espressif/arduino-esp32/blob/master/libraries
// Dependencies:
// You need to have in File->Prefernces->Additional Board managers URLs
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

List of supported modules, and board which needs to be selected

- ELRS Tx module ESP82xx backpack   board: Generic ESP8266 Module or Generic ESP8285 Module
  is used in many ELRS Tx modules
  Comment: ESP82xx needs to have a GPIO0 controllable
- Espressif ESP32-DevKitC V4        board: ESP32 Dev Module
- NodeMCU ESP32-Wroom-32            board: ESP32 Dev Module
- Espressif ESP32-PICO-KIT          board: ESP32 PICO-D4
- Adafruit QT Py S2                 board: Adafruit QT Py ESP32-S2
- Lilygo TTGO-MICRO32               board: ESP32 PICO-D4
- M5Stack M5Stamp C3 Mate           board: ESP32C3 Dev Module
  ATTENTION: when the 5V pin is used, one MUST not also use the USB port, since they are connected internally!!
- M5Stack M5Stamp Pico              board: ESP32 PICO-D4
- M5Stack M5Stamp C3U Mate          board: ESP32C3 Dev Module
  ATTENTION: when the 5V pin is used, one MUST not also use the USB port, since they are connected internally!!
- M5Stack ATOM Lite                 board: M5Stack-ATOM

Troubleshooting:
- If you get error "text section exceeds available space": Set Partition Scheme to "No OTA (Large APP)"
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
#define MODULE_MATEK_TXM_TD30
//#define MODULE_ESP82XX_ELRS_TX                // board: Generic ESP8266 Module or Generic ESP8285 Module
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
//#define MODULE_M5STACK_ATOM_LITE              // board: M5Stack-ATOM
//#define MODULE_GENERIC
//#define MODULE_DIY_E28DUAL_MODULE02_G491RE    // board: ESP32 PICO-D4, upload speed: 115200

// Serial level
// uncomment, if you need inverted serial for a supported module
// (for the generic module also SERIAL_RXD and SERIAL_TXD need to be defined)
//#define USE_SERIAL_INVERTED

// Wireless protocol
// 0 = WiFi TCP, 1 = WiFi UDP, 2 = Wifi UDPSTA, 3 = Bluetooth (not available for all boards), 4 = Wifi UDPCl
// Note: If GPIO0_IO is defined, then this only sets the default protocol
#define WIRELESS_PROTOCOL  1

// GPIO0 usage
// uncomment if your Tx module supports the RESET and GPIO0 lines on the ESP32/ESP82xx (aka AT mode)
// the number determines the IO pin, usally it is 0
//#define GPIO0_IO  0


//**********************//
//*** WiFi settings ***//

// For TCP, UDP (only for these two)
// ssid = "" results in a default name, like "mLRS-13427 AP UDP"
// password = "" makes it an open AP
String ssid = ""; // "mLRS AP"; // Wifi name
String password = ""; // "thisisgreat"; // WiFi password

IPAddress ip(192, 168, 4, 55); // connect to this IP // MissionPlanner default is 127.0.0.1, so enter 192.168.4.55 in MP

int port_tcp = 5760; // connect to this port per TCP // MissionPlanner default is 5760
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550

// For UDP, UDPSTA (only for these two)
// comment out only if broadcast won't work for you (ip_udp declared below will then be used)
#define WIFI_USE_BROADCAST_FOR_UDP

// For UDPSTA, UDPCl (only for these two)
// for UDPSTA setting network_ssid = "" results in
// - a default name, like "mLRS-13427 STA UDP"
// - a default password which includes the mLRS bindphrase, like "mLRS-mlrs.0"
// for UDPCl both strings MUST be set to what your Wifi network requires
String network_ssid = ""; // name of your WiFi network
String network_password = "****"; // password to access your WiFi network

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


//************************//
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

#define VERSION_STR  "v1.3.07" // to not get version salad, use what the current mLRS version is at the time


//-------------------------------------------------------
// Module details
//-------------------------------------------------------

#include "mlrs-wireless-bridge-boards.h"


//-------------------------------------------------------
// Includes
//-------------------------------------------------------

#ifdef GPIO0_IO
#define USE_AT_MODE
#endif

#ifndef ESP8266
#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    #error Version of your ESP Arduino Core below 3.0.0 !
#elif ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 4)
    #warning Consider upgrading your ESP Arduino Core ! // warnings may not be displayed in console !
#endif

#include <WiFi.h>
#include "esp_mac.h"
// for some reason checking
// #if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
// does not work here. Also checking e.g. PLATFORM_ESP32_C3 seems not to work.
// This sucks. So we don't try to be nice but let the compiler work it out.
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 3) // for AT commands we require BT be available
#ifdef CONFIG_IDF_TARGET_ESP32 // classic BT only available on ESP32
  #define USE_WIRELESS_PROTOCOL_BLUETOOTH
  #include <BluetoothSerial.h>
#endif
#endif
#endif // #ifndef ESP8266


//-------------------------------------------------------
// Internals
//-------------------------------------------------------

// TCP, UDP, UDPCl
IPAddress ip_gateway(0, 0, 0, 0);
IPAddress netmask(255, 255, 255, 0);
// UDP, UDPSTA (will be overwritten if WIFI_USE_BROADCAST_FOR_UDP is set)
#ifndef ESP8266
IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3]+1); // usually the client/MissionPlanner gets assigned +1
#else // the ESP8266 requires different, appears to be a bug in the Arduino lib
IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3]+99); // the first DHCP client/MissionPlanner gets assigned +99
#endif
WiFiUDP udp;
// TCP
WiFiServer server(port_tcp);
WiFiClient client;
// Bluetooth
#ifdef USE_WIRELESS_PROTOCOL_BLUETOOTH
BluetoothSerial SerialBT;
#endif

typedef enum {
    WIRELESS_PROTOCOL_TCP = 0,
    WIRELESS_PROTOCOL_UDP = 1,
    WIRELESS_PROTOCOL_UDPSTA = 2,
    WIRELESS_PROTOCOL_BT = 3,
    WIRELESS_PROTOCOL_UDPCl = 4,
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

uint16_t device_id = 0; // is going to be set by setup_device_name_and_password(), and can be queried in at mode
String device_name = "";
String device_password = "";

#ifdef USE_AT_MODE
#include <Preferences.h>
Preferences preferences;
#include "mlrs-wireless-bridge-at-mode.h"
AtMode at_mode;
#endif

bool wifi_initialized;
bool led_state;
unsigned long led_tlast_ms;
bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;


void serialFlushRx(void)
{
    while (SERIAL.available() > 0) { SERIAL.read(); }
}


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup_device_name_and_password(void)
{
    uint8_t MAC_buf[6+2];
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/misc_system_api.html#mac-address
    // MACs are different for STA and AP, BT
#ifndef ESP8266
    esp_base_mac_addr_get(MAC_buf);
#else
    wifi_get_macaddr(STATION_IF, MAC_buf);
#endif
    for (uint8_t i = 0; i < 5; i++) device_id += MAC_buf[i] + ((uint16_t)MAC_buf[i + 1] << 8) / 39;
    device_id += MAC_buf[5];
    device_name = "mLRS-";
#ifdef DEVICE_NAME_HEAD
    device_name = String(DEVICE_NAME_HEAD) + "-mLRS-";
#endif
    if (g_protocol == WIRELESS_PROTOCOL_TCP) {
        device_name = (ssid == "") ? device_name + String(device_id) + " AP TCP" : ssid;
    } else if (g_protocol == WIRELESS_PROTOCOL_UDP) {
        device_name = (ssid == "") ? device_name + String(device_id) + " AP UDP" : ssid;
    } else if (g_protocol == WIRELESS_PROTOCOL_UDPSTA) {
        device_name = (network_ssid == "") ? device_name + String(device_id) + " STA UDP" : network_ssid;
        device_password = (network_ssid == "") ? String("mLRS-") + g_bindphrase : network_password;
    } else if (g_protocol == WIRELESS_PROTOCOL_UDPCl) {
        device_name = network_ssid;
        device_password = network_password;
    } else if (g_protocol == WIRELESS_PROTOCOL_BT) {
        device_name = (bluetooth_device_name == "") ? device_name + String(device_id) + " BT" : bluetooth_device_name;
    }
}


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


void setup_wifi()
{
if (g_protocol == WIRELESS_PROTOCOL_TCP || g_protocol == WIRELESS_PROTOCOL_UDP) {
//-- WiFi TCP, UDP

    // AP mode
    WiFi.mode(WIFI_AP); // seems not to be needed, done by WiFi.softAP()?
    WiFi.softAPConfig(ip, ip_gateway, netmask);
    WiFi.softAP(device_name.c_str(), (password.length()) ? password.c_str() : NULL, g_wifichannel); // channel = 1 is default
    DBG_PRINT("ap ip address: ");
    DBG_PRINTLN(WiFi.softAPIP()); // comes out as 192.168.4.1
    DBG_PRINT("channel: ");
    DBG_PRINTLN(WiFi.channel());

    setup_wifipower();
    if (g_protocol == WIRELESS_PROTOCOL_TCP) {
        server.begin();
        server.setNoDelay(true);
    } else
    if (g_protocol == WIRELESS_PROTOCOL_UDP) {
#ifdef WIFI_USE_BROADCAST_FOR_UDP
        //ip_udp = WiFi.broadcastIP(); // seems to not work for AP mode
        ip_udp[3] = 255; // start with broadcast, the subnet mask is 255.255.255.0 so just last octet needs to change
#endif
        udp.begin(port_udp);
    }

} else
if (g_protocol == WIRELESS_PROTOCOL_BT) {
//-- Bluetooth
// Comment: CONFIG_BT_SSP_ENABLED appears to be defined per default, so setPin() is not available
#ifdef USE_WIRELESS_PROTOCOL_BLUETOOTH

    SerialBT.begin(device_name);

#endif
} else
if (g_protocol == WIRELESS_PROTOCOL_UDPSTA) {
//-- Wifi UDPSTA

    // STA mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.begin(device_name.c_str(), device_password.c_str());

    while (WiFi.status() != WL_CONNECTED) {
        led_on(true); delay(75); led_off(); delay(75);
        led_on(true); delay(75); led_off(); delay(75);
        led_on(true); delay(75); led_off(); delay(75);
        DBG_PRINTLN("connecting to WiFi network...");
    }
    DBG_PRINTLN("connected");
    DBG_PRINT("network ip address: ");
    DBG_PRINTLN(WiFi.localIP());

    setup_wifipower();
#ifdef WIFI_USE_BROADCAST_FOR_UDP
    ip_udp = WiFi.broadcastIP(); // start with broadcast
#endif
    udp.begin(port_udp);

} else
if (g_protocol == WIRELESS_PROTOCOL_UDPCl) {
//-- Wifi UDPCl

    // STA mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.config(ip_udpcl, ip_gateway, netmask);
    WiFi.begin(device_name.c_str(), device_password.c_str());

    while (WiFi.status() != WL_CONNECTED) {
        //delay(500);
        led_on(true); delay(75); led_off(); delay(75);
        led_on(true); delay(75); led_off(); delay(75);
        led_on(true); delay(75); led_off(); delay(75);
        DBG_PRINTLN("connecting to WiFi network...");
    }
    DBG_PRINTLN("connected");
    DBG_PRINT("network ip address: ");
    DBG_PRINTLN(WiFi.localIP());

    setup_wifipower();
    udp.begin(port_udpcl);

}
}


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
        g_protocol != WIRELESS_PROTOCOL_UDPCl && g_protocol != WIRELESS_PROTOCOL_BT) { // not a valid value
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
#endif
    setup_device_name_and_password();

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

    // Gpio0 handling
#ifdef USE_AT_MODE
    at_mode.Init(GPIO0_IO);
#endif

    wifi_initialized = false; // setup_wifi();

    led_tlast_ms = 0;
    led_state = false;

    is_connected = false;
    is_connected_tlast_ms = 0;

    serial_data_received_tfirst_ms = 0;

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

    if (tnow_ms - led_tlast_ms > (is_connected ? 500 : 200)) {
        led_tlast_ms = tnow_ms;
        led_state = !led_state;
        if (led_state) led_on(is_connected); else led_off();
    }

    //-- here comes the core code, handle WiFi or Bluetooth connection and do the bridge

    uint8_t buf[256]; // working buffer

    if (!wifi_initialized) {
        wifi_initialized = true;
        setup_wifi();
        return;
    }

if (g_protocol == WIRELESS_PROTOCOL_TCP) {
//-- WiFi TCP

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
        //uint8_t c = (uint8_t)client.read();
        //SERIAL.write(c);
        int len = client.read(buf, sizeof(buf));
        SERIAL.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // update it
    int avail = SERIAL.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = SERIAL.read(buf, sizeof(buf));
        client.write(buf, len);
    }

} else
if (g_protocol == WIRELESS_PROTOCOL_UDP || g_protocol == WIRELESS_PROTOCOL_UDPSTA) {
//-- WiFi UDP, UDPSTA

    if (g_protocol == WIRELESS_PROTOCOL_UDPSTA){
        if (!is_connected && WiFi.status() != WL_CONNECTED) {
            udp.stop();
            setup_wifi(); // attempt to reconnect if WiFi got disconnected
        }
    }

    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buf, sizeof(buf));
        SERIAL.write(buf, len);
#ifdef WIFI_USE_BROADCAST_FOR_UDP
        if (!is_connected) { // first received UDP packet
            ip_udp = udp.remoteIP(); // stop broadcast, switch to unicast to avoid Aurdino performance issue
            port_udp = udp.remotePort();
        }
#endif
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = SERIAL.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = SERIAL.read(buf, sizeof(buf));
        udp.beginPacket(ip_udp, port_udp);
        udp.write(buf, len);
        udp.endPacket();
    }

} else
if (g_protocol == WIRELESS_PROTOCOL_BT) {
//-- Bluetooth
#ifdef USE_WIRELESS_PROTOCOL_BLUETOOTH

    int len = SerialBT.available();
    if (len > 0) {
        if (len > sizeof(buf)) len = sizeof(buf);
        for (int i = 0; i < len; i++) buf[i] = SerialBT.read();
        SERIAL.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = SERIAL.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = SERIAL.read(buf, sizeof(buf));
        SerialBT.write(buf, len);
    }

#endif // USE_WIRELESS_PROTOCOL_BLUETOOTH
} else
if (g_protocol == WIRELESS_PROTOCOL_UDPCl) {
//-- WiFi UDPCl (STA mode)

    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buf, sizeof(buf));
        SERIAL.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    if (!is_connected) {
        // we wait for a first message from the remote
        // remote's ip and port not known, so jump out
        serialFlushRx();
        return;
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = SERIAL.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = SERIAL.read(buf, sizeof(buf));
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write(buf, len);
        udp.endPacket();
    }

}

    delay(2); // give it always a bit of time
}

