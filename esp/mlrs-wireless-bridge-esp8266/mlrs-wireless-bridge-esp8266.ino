//*******************************************************
// mLRS Wireless Bridge for ESP8266
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Basic but effective & reliable transparent WiFi <-> serial bridge.
// Minimizes wireless traffic while respecting latency by better packeting algorithm.
//*******************************************************
// 1. June 2024
//*********************************************************/
// Adapted from mlrs-wireless-bridge for ESP32
// ArduinoIDE 2.1.1, esp8266 by ESP8266 Community 3.1.2
/*
Definitions:
- "module" refers to the physical hardware
- "board" refers to the board you need to select in the Arduino IDE menu Tools->Board
*/
#ifndef ESP8266
  #error Select board Generic ESP8266 Module!
#else  
  #include <ESP8266WiFi.h>
#endif


//-------------------------------------------------------
// User configuration
//-------------------------------------------------------

// Wireless protocol
// 0 = WiFi TCP, 1 = WiFi UDP, 2 = Wifi UDPCl
#define WIRELESS_PROTOCOL  1


//**********************//
//*** WiFi settings ***//

// for TCP, UDP
String ssid = "mLRS AP"; // Wifi name
String password = ""; // "thisisgreat"; // WiFi password, "" makes it an open AP

IPAddress ip(192, 168, 4, 55); // connect to this IP // MissionPlanner default is 127.0.0.1, so enter

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
// Note: In order to find the possible options, right click on WIFI_POWER_19_5dBm and choose "Go To Definiton"
//#define WIFI_POWER  0 // 0 (min) to +20.5 (max) dBm


//**************************//
//*** Bluetooth settings ***//

// Bluetooth not available on ESP8266


//************************//
//*** General settings ***//

// Baudrate
int baudrate = 115200;

// LED pin (only effective for the generic module)
// uncomment if you want a LED, and set the pin number as desired
//#define LED_IO  2


//-------------------------------------------------------
// Includes
//-------------------------------------------------------

#if (WIRELESS_PROTOCOL <= 2) // WiFi
//  #include <ESP8266WiFi.h>
  #if WIRELESS_PROTOCOL > 0 // UDP
    #include <WiFiUdp.h>
  #endif
#else
  #error Invalid WIRELESS_PROTOCOL chosen !
#endif


//-------------------------------------------------------
// Module details
//-------------------------------------------------------

#ifdef LED_IO
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
#else
void led_init(void) {}
void led_on(bool is_connected) {}
void led_off(void) {}
#endif

#define serial  Serial

#define DBG_PRINT(x)
#define DBG_PRINTLN(x)


//-------------------------------------------------------
// Internals
//-------------------------------------------------------

#if (WIRELESS_PROTOCOL <= 1) // WiFi TCP, UDP

IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3]+99); // The first DHCP client gets assigned +99
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

#elif (WIRELESS_PROTOCOL == 3) // Bluetooth
#endif

bool led_state;
unsigned long led_tlast_ms;
bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;


void serialFlushRx(void)
{
    while (serial.available() > 0) { serial.read(); }
}


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup()
{
    led_init();
    delay(500);

    size_t rxbufsize = serial.setRxBufferSize(2*1024); // must come before uart started, retuns 0 if it fails
    serial.begin(baudrate);

    WiFi.disconnect(true);

#if (WIRELESS_PROTOCOL <= 1)
//-- WiFi TCP, UDP

    // AP mode
    WiFi.mode(WIFI_AP); // seems not to be needed, done by WiFi.softAP()?
    WiFi.softAPConfig(ip, ip_gateway, netmask);
  #if (WIRELESS_PROTOCOL == 1)
    String ssid_full = ssid + " UDP";
  #else
    String ssid_full = ssid + " TCP";
  #endif
    WiFi.softAP(ssid_full.c_str(), (password.length()) ? password.c_str() : NULL, wifi_channel); // channel = 1 is default
    DBG_PRINT("ap ip address: ");
    DBG_PRINTLN(WiFi.softAPIP()); // comes out as 192.168.4.1
    DBG_PRINT("channel: ");
    DBG_PRINTLN(WiFi.channel());

  #ifdef WIFI_POWER
    WiFi.setOutputPower(WIFI_POWER); // set WiFi power, AP or STA must have been started, returns false if it fails
  #endif
  #if (WIRELESS_PROTOCOL == 1)
    // Broadcast causes message loss for some reason
    // ip_udp[3] = 255;
    udp.begin(port_udp);
  #else
    server.begin();
    server.setNoDelay(true);
  #endif    

#elif (WIRELESS_PROTOCOL == 2)
//-- Wifi UDPCl

    // STA mode
    WiFi.mode(WIFI_STA);
    //WiFi.disconnect();
    WiFi.config(ip_udpcl, ip_gateway, netmask);
    WiFi.begin(network_ssid.c_str(), network_password.c_str());

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

  #ifdef WIFI_POWER
    WiFi.setOutputPower(WIFI_POWER); // set WiFi power, AP or STA must have been started, returns false if it fails
  #endif
    udp.begin(port_udpcl);

#elif (WIRELESS_PROTOCOL == 3)
//-- Bluetooth

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

    //-- here comes the core code, handle WiFi or Bluetooth connection and do the bridge

    uint8_t buf[256]; // working buffer

#if (WIRELESS_PROTOCOL == 0)
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
        int len = client.read(buf, sizeof(buf));
        serial.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // update it
    int avail = serial.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = serial.read(buf, sizeof(buf));
        client.write(buf, len);
    }

#elif (WIRELESS_PROTOCOL == 1)
//-- WiFi UDP

    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buf, sizeof(buf));
        serial.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = serial.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = serial.read(buf, sizeof(buf));
        udp.beginPacket(ip_udp, port_udp);
        udp.write(buf, len);
        udp.endPacket();
    }

#elif (WIRELESS_PROTOCOL == 2)
//-- WiFi UDPCl (STA mode)

    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buf, sizeof(buf));
        serial.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    if (!is_connected) {
        // remote's ip and port not known, so jump out
        serialFlushRx();
        return;
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = serial.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;

        int len = serial.read(buf, sizeof(buf));
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write(buf, len);
        udp.endPacket();
    }

#elif (WIRELESS_PROTOCOL == 3)
//-- Bluetooth

#endif // (WIRELESS_PROTOCOL == 3)
}

