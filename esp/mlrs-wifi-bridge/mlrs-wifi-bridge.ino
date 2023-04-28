//*******************************************************
// mLRS WiFi Bridge
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Basic but effective & reliable transparent WiFi<->serial bridge
//*******************************************************
// 28. Apr. 2023
//*********************************************************/
// inspired by examples from Arduino
// ArduinoIDE 2.0.3, esp32 by Espressif Systems 2.0.6
// use upload speed 115200 if serial passthrough shall be used for flashing
/*
for more details on the boards see mlrs-wifi-bridge-boards.h

- Adafruit QT Py S2
  board: Adafruit QT Py ESP32-S2
- M5Stack M5Stamp C3 Mate
  board: ESP32C3 Dev Module
  ATTENTION: when the 5V pin is used, one MUST not also use the USB port, since they are connected internally!!
- M5Stack M5Stamp Pico
  board: ESP32-PICO-D4
- ESP32-PICO-KIT
  board: ESP32-PICO-D4
- TTGO-MICRO32
  board: ESP32-PICO-D4
- M5Stack M5Stamp C3U Mate
  board: ESP32C3 Dev Module
  ATTENTION: when the 5V pin is used, one MUST not also use the USB port, since they are connected internally!!
*/

#include <WiFi.h>


//-------------------------------------------------------
// User configuration
//-------------------------------------------------------

// Board
// un-comment what you want
//#define MODULE_GENERIC
#define MODULE_ADAFRUIT_QT_PY_ESP32_S2
//#define MODULE_M5STAMP_C3_MATE
//#define MODULE_TTGO_MICRO32
//#define MODULE_ESP32_PICO_KIT
//#define MODULE_M5STAMP_C3U_MATE_FOR_FRSKY_R9M
//#define MODULE_M5STAMP_PICO_FOR_FRSKY_R9M


// Wifi Protocol 0 = TCP, 1 = UDP
#define WIFI_PROTOCOL  1

// Wifi credentials
String ssid = "mLRS AP"; // Wifi name
String password = ""; // "thisisgreat"; // WiFi password, "" makes it an open AP

IPAddress ip(192, 168, 4, 55); // connect to this IP // MissionPlanner default is 127.0.0.1, so enter

int port_tcp = 5760; // connect to this port per TCP // MissionPlanner default is 5760
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550

// baudrate
int baudrate = 115200;

// WiFi channel
// 1 is the default, 13 (2461-2483 MHz) has the least overlap with mLRS 2.4 GHz frequencies.
// Note: Channel 13 is generally not available in the US, where 11 is the maximum.
int wifi_channel = 13;

// WiFi power
// comment out for default setting
#define WIFI_POWER  WIFI_POWER_2dBm // WIFI_POWER_MINUS_1dBm is the lowest possible, WIFI_POWER_19_5dBm is the max


// serial port usage (only effective for a generic board)
// comment all for default behavior, which is using only Serial port
//#define USE_SERIAL_DBG1 // use Serial for communication and flashing, and Serial1 for debug output
//#define USE_SERIAL1_DBG // use Serial1 for communication, and Serial for debug output and flashing

// LED pin (only effective for a generic board)
// un-comment if you want a LED
//#define LED_IO  13


//-------------------------------------------------------
// board details
//-------------------------------------------------------

#include "mlrs-wifi-bridge-boards.h"


//-------------------------------------------------------
// internals
//-------------------------------------------------------

IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3]+1); // speculation: it seems that MissionPlanner wants it +1
IPAddress netmask(255, 255, 255, 0);
#if WIFI_PROTOCOL == 1 // UDP
WiFiServer server(80);
WiFiUDP udp;
#else // TCP
WiFiServer server(port_tcp);
WiFiClient client;
#endif

bool led_state;
unsigned long led_tlast_ms;
bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;


void serialFlushRx(void)
{
    while (SERIAL.available() > 0) { uint8_t c = SERIAL.read(); }
}


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup() 
{
    led_init();
    dbg_init();
    delay(500);

    size_t rxbufsize = SERIAL.setRxBufferSize(2*1024); // must come before uart started, retuns 0 if it fails
    size_t txbufsize = SERIAL.setTxBufferSize(512); // must come before uart started, retuns 0 if it fails
#ifdef SERIAL_RXD // if SERIAL_TXD is not defined the compiler will complain, so all good
  #ifdef SERIAL_INVERT
    SERIAL.begin(baudrate, SERIAL_8N1, SERIAL_RXD, SERIAL_TXD, SERIAL_INVERT);
  #else
    SERIAL.begin(baudrate, SERIAL_8N1, SERIAL_RXD, SERIAL_TXD);
  #endif
#else    
    SERIAL.begin(baudrate);
#endif    
//????used to work    pinMode(U1_RXD, INPUT_PULLUP); // important, at least in older versions Arduino serial lib did not do it

    DBG_PRINTLN(rxbufsize);
    DBG_PRINTLN(txbufsize);

    // AP mode
    //WiFi.mode(WIFI_AP); // seems not to be needed, done by WiFi.softAP()?
    WiFi.softAPConfig(ip, ip, netmask);
#if WIFI_PROTOCOL == 1
    String ssid_full = ssid + " UDP";
#else    
    String ssid_full = ssid + " TCP";
#endif    
    WiFi.softAP(ssid_full.c_str(), (password.length()) ? password.c_str() : NULL, wifi_channel); // channel = 1 is default
    DBG_PRINT("ap ip address: ");
    DBG_PRINTLN(WiFi.softAPIP()); // comes out as 192.168.4.1
    DBG_PRINT("channel: ");
    DBG_PRINTLN(WiFi.channel());

    server.begin();
    server.setNoDelay(true);

#ifdef WIFI_POWER
    WiFi.setTxPower(WIFI_POWER); // set WiFi power, AP or STA must have been started, returns false if it fails
#endif    
#if WIFI_PROTOCOL == 1
    udp.begin(port_udp);
#endif    

    led_tlast_ms = 0;
    led_state = false;

    is_connected = false;
    is_connected_tlast_ms = 0;

    serial_data_received_tfirst_ms = 0;

    serialFlushRx();
}


void loop() 
{
    unsigned long tnow_ms = millis();

    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) { // nothing from GCS for 2 secs
        is_connected = false;
    }

    if (tnow_ms - led_tlast_ms > (is_connected ? 500 : 200)) {
        led_tlast_ms = tnow_ms;
        led_state = !led_state;
        if (led_state) led_on(is_connected); else led_off();
    }

    //-- here comes the core code, handle wifi connection and do the bridge

    uint8_t buf[256]; // working buffer

#if WIFI_PROTOCOL == 1 // UDP

    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buf, sizeof(buf));
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
        udp.beginPacket(ip_udp, port_udp);
        udp.write(buf, len);
        udp.endPacket();
    }

#else // TCP

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

#endif    
}

