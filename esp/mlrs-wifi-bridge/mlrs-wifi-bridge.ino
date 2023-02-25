//*******************************************************
// mLRS WiFi Bridge
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Basic but effective & reliable transparent WiFi<->serial bridge
//*******************************************************
// 24. Feb. 2023
//*********************************************************/
// inspired by examples from Arduino
// ArduinoIDE 2.0.3, esp32 by Espressif Systems 2.0.6
// use upload speed 115200 if serial passthrough shall be used for flashing
/*
for more details on the boards see mlrs-wifi-bridge-boards.h

- ESP32-PICO-KIT
  board: ESP32-PICO-D4
- TTGO-MICRO32
  board: ESP32-PICO-D4
- Adafruit QT Py S2
  board: Adafruit QT Py ESP32-S2
- M5Stack M5Stamp C3 Mate
  board: ESP32C3 Dev Module
  ATTENTION: when the 5V pin is used, one MUST not also use the USB port, since they are connected internally!!
*/

/*
much info is in 
C:\Users\...\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.6\cores\esp32\HardwareSerial.h/.cpp
default buffer sizes are
_rxBufferSize(256),
_txBufferSize(0), 
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


// Wifi credentials
String ssid = "mLRS AP"; // Wifi name
String password = ""; // "thisisgreat"; // WiFi password, "" makes it an open AP

IPAddress ip(192, 168, 4, 1); // connect to this IP // MissionPlanner default is 127.0.0.1, so enter
int port = 5760; // connect to this port // MissionPlanner default is 5760

// baudrate
int baudrate = 57600;

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

IPAddress netmask(255, 255, 255, 0);
WiFiServer server(port);
WiFiClient client;

int led_tlast_ms;
bool led_state;

bool connected_last;


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
    SERIAL.begin(baudrate, SERIAL_8N1, SERIAL_RXD, SERIAL_TXD);
#else    
    SERIAL.begin(baudrate);
#endif    
//????used to work    pinMode(U1_RXD, INPUT_PULLUP); // important, at least in older versions Arduino serial lib did not do it

    DBG_PRINTLN(rxbufsize);
    DBG_PRINTLN(txbufsize);

    // AP mode
    WiFi.softAPConfig(ip, ip, netmask);
    //WiFi.mode(WIFI_AP); // seems to not be needed, probably done by WiFi.softAP()
    WiFi.softAP(ssid.c_str(), (password.length()) ? password.c_str() : NULL);

    DBG_PRINT("ap ip address: ");
    DBG_PRINTLN(WiFi.softAPIP()); // comes out as 192.168.4.1

    server.begin();
    server.setNoDelay(true);

#ifdef WIFI_POWER
    WiFi.setTxPower(WIFI_POWER); // set WiFi power, AP or STA must have been started, returns false if it fails
#endif    

    led_tlast_ms = 0;
    led_state = false;

    connected_last = false;

    serialFlushRx();
}


void loop() 
{
    int tnow_ms = millis();
    if (tnow_ms - led_tlast_ms > ((client.connected()) ? 500 : 200)) {
        led_tlast_ms = tnow_ms;
        led_state = !led_state;
        if (led_state) led_on(); else led_off();
    }

    //-- here comes the core code, handle wifi connection and do the bridge

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

    bool connected = client.connected();
    if (connected_last != connected) { // connected status has changed
        connected_last = connected;
        DBG_PRINTLN((connected) ? "connected" : "disconnected"); 
    }

    if (!client.connected()) { // nothing to do
        client.stop();
        serialFlushRx();  
        return;
    }

    uint8_t buf[256]; // working buffer

    while (client.available()) {
        //uint8_t c = (uint8_t)client.read();
        //SERIAL.write(c);
        int len = client.read(buf, sizeof(buf));
        SERIAL.write(buf, len);
    }

    while (SERIAL.available()) {
        //uint8_t c = (uint8_t)SERIAL.read();
        //client.write(c);
        int len = SERIAL.read(buf, sizeof(buf));
        client.write(buf, len);
    }    
}

