//*******************************************************
// mLRS Wireless Bridge for ESP8266
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Basic but effective & reliable transparent WiFi<->serial bridge
//*******************************************************
// 28. Aug. 2023
//*********************************************************/
// Adapted from mlrs-wireless-bridge for ESP32
// ArduinoIDE 2.1.1, esp8266 by ESP8266 Community 3.1.2
// ------------------------------
// Tested with Ai-Thinker ESP-01S
// ------------------------------
// board: Generic ESP8266 Module
// http://www.ai-thinker.com/pro_view-88.html
// IO3/IO1: U0RXD/U0TXD is Serial
//
// Please be aware that this code here is unmaintained
// and would also benefit from further testing feedback.

#include <ESP8266WiFi.h>

#ifndef ESP8266 // ARDUINO_BOARD != Generic ESP8266 Module
  #error Select board Generic ESP8266 Module!
#endif

#define LED_IO  2

//-------------------------------------------------------
// User configuration
//-------------------------------------------------------

// Wireless protocol
// 0 = WiFi TCP, 1 = WiFi UDP, 2 = Wifi UDPCl
#define WIRELESS_PROTOCOL 1

// Wifi credentials
String ssid = "mLRS AP"; // Wifi name
String password = ""; // "thisisgreat"; // WiFi password, "" makes it an open AP

IPAddress ip(192, 168, 4, 55);

int port_tcp = 5760; // connect to this port per TCP // MissionPlanner default is 5760
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550

// for UDPCl
String network_ssid = "****"; // name of your WiFi network
String network_password = "****"; // password to access your WiFi network

IPAddress ip_udpcl(192, 168, 0, 164); // connect to this IP // MissionPlanner default is 192.168.0.164

int port_udpcl = 14550; // connect to this port per UDPCL // MissionPlanner default is 14550

// WiFi channel
// 1 is the default, 13 (2461-2483 MHz) has the least overlap with mLRS 2.4 GHz frequencies.
// Note: Channel 13 is generally not available in the US, where 11 is the maximum.
int wifi_channel = 6;

// WiFi power
// comment out for default setting
#define WIFI_POWER_dBm 0 // 0 (min) to +20.5 (max) dBm

// baudrate
int baudrate = 115200;

//-------------------------------------------------------
// internals
//-------------------------------------------------------

#if WIRELESS_PROTOCOL > 0 // UDP
#include <WiFiUdp.h>
#endif

#if (WIRELESS_PROTOCOL <= 1) // WiFi TCP, UDP
  IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3]+1); // speculation: it seems that MissionPlanner wants it +1
  IPAddress ip_gateway(0, 0, 0, 0);
  IPAddress netmask(255, 255, 255, 0);
  #if WIRELESS_PROTOCOL == 1 // UDP
    WiFiUDP udp;
  #else // TCP
    WiFiServer server(port_tcp);
    WiFiClient client;
  #endif
#elif (WIRELESS_PROTOCOL == 2) // WiFi UDPCl
  IPAddress ip_gateway(0, 0, 0, 0);
  IPAddress netmask(255, 255, 255, 0);
  WiFiUDP udp;
#else
  #error Invalid WIRELESS_PROTOCOL selected!
#endif

bool led_state;
unsigned long led_tlast_ms;
bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;

void led_init(void) 
{
  pinMode(LED_IO, OUTPUT);
  digitalWrite(LED_IO, LOW);
}

void led_on(bool is_connected) 
{
  digitalWrite(LED_IO, HIGH);
}

void led_off(void) 
{
  digitalWrite(LED_IO, LOW);
}


void serialFlushRx(void)
{
  while (Serial.available() > 0) { uint8_t c = Serial.read(); }
}


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup() 
{
    led_init();
    delay(500);

    size_t rxbufsize = Serial.setRxBufferSize(2*1024); // must come before uart started, retuns 0 if it fails
    Serial.begin(baudrate);

#if (WIRELESS_PROTOCOL <= 1)
  //-- WiFi TCP, UDP

  // AP mode
  WiFi.softAPConfig(ip, ip_gateway, netmask);
  #if (WIRELESS_PROTOCOL == 1)
    String ssid_full = ssid + " UDP";
  #else
    String ssid_full = ssid + " TCP";
  #endif
  WiFi.softAP(ssid_full.c_str(), (password.length()) ? password.c_str() : NULL, wifi_channel); // channel = 1 is default

  #ifdef WIFI_POWER_dBm
    WiFi.setOutputPower(WIFI_POWER_dBm); // set WiFi power, AP or STA must have been started, returns false if it fails
  #endif
  #if (WIRELESS_PROTOCOL == 1)
    udp.begin(port_udp);
  #else
    server.begin();
    server.setNoDelay(true);
  #endif    

#elif (WIRELESS_PROTOCOL == 2)
//-- Wifi UDPCl

    // STA mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.config(ip_udpcl, ip_gateway, netmask);
    WiFi.begin(network_ssid.c_str(), network_password.c_str());

    while (WiFi.status() != WL_CONNECTED) {
        //delay(500);
        led_on(true); delay(75); led_off(); delay(75); 
        led_on(true); delay(75); led_off(); delay(75); 
        led_on(true); delay(75); led_off(); delay(75); 
    }

  #ifdef WIFI_POWER_dBm
    WiFi.setOutputPower(WIFI_POWER_dBm); // set WiFi power, AP or STA must have been started, returns false if it fails
  #endif
    udp.begin(port_udpcl);
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

#if (WIRELESS_PROTOCOL == 0)
    //-- WiFi TCP

    if (server.hasClient()) {
      if (!client.connected()) {
        client.stop(); // doesn't appear to make a difference
        client = server.available();
        } else { // is already connected, so reject, doesn't seem to ever happen
            server.available().stop();
        }
    }

    if (!client.connected()) { // nothing to do
        client.stop();
        serialFlushRx();  
        is_connected = false;
        return;
    }

    while (client.available()) {
        int len = client.read(buf, sizeof(buf));
        Serial.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // update it
    int avail = Serial.available();
    if (avail <= 0) {
      serial_data_received_tfirst_ms = tnow_ms;      
    } else {
      if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
           serial_data_received_tfirst_ms = tnow_ms;
          int len = Serial.read(buf, sizeof(buf));
          client.write(buf, len);
      }
    }
#elif (WIRELESS_PROTOCOL == 1)
    //-- WiFi UDP
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buf, sizeof(buf));
        Serial.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = Serial.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;      
    } else 
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = Serial.read(buf, sizeof(buf));
        udp.beginPacket(ip_udp, port_udp);
        udp.write(buf, len);
        udp.endPacket();
    }

#elif (WIRELESS_PROTOCOL == 2)
    //-- WiFi UDPCl (STA mode)

    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buf, sizeof(buf));
        Serial.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    if (!is_connected) {
        // remote's ip and port not known, so jump out
        serialFlushRx();
        return;
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = Serial.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = Serial.read(buf, sizeof(buf));
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write(buf, len);
        udp.endPacket();
    }
#endif    
}
