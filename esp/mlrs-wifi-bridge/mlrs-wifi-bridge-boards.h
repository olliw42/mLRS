//*******************************************************
// mLRS WiFi Bridge
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Basic but effective & reliable transparent WiFi<->serial bridge
//*******************************************************

/*
------------------------------
Espressif ESP32-PICO-KIT
------------------------------
board: ESP32-PICO-D4
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-pico-kit.html
IO3/IO1: U0RXD/U0TXD, connected via usb-ttl adapter to USB port, is Serial, spits out lots of preamble at power up
IO9/IO10: U1RXD/U1TXD, is Serial1
IO16/IO17: U2RXD/U2TXD, uses IO16/IO17 for internal flash, hence not available as serial

------------------------------
NodeMCU ESP32-Wroom-32 or ESP32-DevKitC V4
------------------------------
board: ESP32 Dev Module
https://esphome.io/devices/nodemcu_esp32.html
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html
IO3/IO1: U0RXD/U0TXD, connected via usb-ttl adapter to USB port, is Serial, spits out lots of preamble at power up
IO16/IO17: U2RXD/U2TXD, is Serial2

------------------------------
Lilygo TTGO-MICRO32
------------------------------
board: ESP32-PICO-D4
http://www.lilygo.cn/prod_view.aspx?TypeId=50033&Id=1091
IO3/IO1: U0RXD/U0TXD, is Serial, spits out lots of preamble at power up
IO9/IO10: U1RXD/U1TXD, is Serial1
no IO16/IO17 pads
use only U0

------------------------------
Adafruit QT Py S2
------------------------------
board: Adafruit QT Py ESP32-S2
https://learn.adafruit.com/adafruit-qt-py-esp32-s2/arduino-ide-setup
use only Serial1, Serial is USB port and can be used for debug
RESET and BOOT available on solder pads, BOOT is GPIO0, not very connvenient

------------------------------
M5Stack M5Stamp C3 Mate
------------------------------
board: ESP32C3 Dev Module
https://shop.m5stack.com/collections/m5-controllers/products/m5stamp-c3-mate-with-pin-headers
https://docs.m5stack.com/en/core/stamp_c3
IO20/IO21:: U0RXD/U0TXD, connected via usb-ttl adapter to USB port, available on pads, is Serial, spits out lots of preamble at power up
IO18/IO19: U1RXD/U1TXD, is Serial1
UARTs can be mapped to any pins, according to data sheet
ATTENTION: when the 5V pin is used, one MUST not also use the USB port, since they are connected internally!!

------------------------------
M5Stamp Pico for Frsky R9M
------------------------------
board: ESP32-PICO-D4
https://shop.m5stack.com/collections/m5-controllers/products/m5stamp-pico-mate-with-pin-headers
https://docs.m5stack.com/en/core/stamp_pico
IO3/IO1: is Serial, spits out lots of preamble at power up
IO32/IO33: will be mapped to Serial1, inverted
Mates cleanly with R9M inverted serial port pins

------------------------------
M5Stack M5Stamp C3U Mate for Frsky R9M
------------------------------
board: ESP32C3 Dev Module
https://shop.m5stack.com/collections/m5-controllers/products/m5stamp-c3u-mate-with-pin-headers
https://docs.m5stack.com/en/core/stamp_c3u
IO18/IO19: D-/D+ Native USB interface
IO20/IO21: U0RXD/U0TXD, is Serial
IO1/IO0: G1/G0, will be mapped to Serial1, inverted
UARTs can be mapped to any pins, according to data sheet
Mates cleanly with R9M inverted serial port pins
ATTENTION: when the 5V pin is used, one MUST not also use the USB port, since they are connected internally!!
Disconnect from application before programming via USB. Hold down central button (G9) when connecting to USB to program

------------------------------
M5Stack ATOM Lite
------------------------------
board: M5Stack-ATOM
https://shop.m5stack.com/products/atom-lite-esp32-development-kit
http://docs.m5stack.com/en/core/atom_lite
IO32/IO26: U0RXD/U0TXD, is remapped as Serial

*/

/*
ESP32:

shortening GPIO15 to GND suppresses the bootloader preamble on Serial port
GPIO15 = RTC_GPIO13
*/


//-------------------------------------------------------
// board details
//-------------------------------------------------------
//-- NodeMCU ESP32-Wroom-32 or ESP32-DevKitC V4
#if defined MODULE_NODEMCU_ESP32_WROOM32
    #ifndef ARDUINO_ESP32_DEV // ARDUINO_BOARD != ARDUINO_ESP32_DEV
	      #error Select board ESP32 Dev Module!
    #endif

    #undef USE_SERIAL_DBG1
    #undef USE_SERIAL1_DBG
    #define USE_SERIAL2_DBG

    // ESP32-DevKitC V4 does not have embedded LED
    #ifndef LED_IO
        #define LED_IO  2
    #endif    
    #define USE_LED


//-- NodeMCU ESP32-Wroom-32 or ESP32-DevKitC V4 (inverted serial)
#elif defined MODULE_NODEMCU_ESP32_WROOM32_SERIAL_INVERTED
    #ifndef ARDUINO_ESP32_DEV // ARDUINO_BOARD != ARDUINO_ESP32_DEV
	      #error Select board ESP32 Dev Module!
    #endif

    #undef USE_SERIAL_DBG1
    #undef USE_SERIAL1_DBG
    #define USE_SERIAL2_DBG

    #define SERIAL_RXD 16 // = RX2
    #define SERIAL_TXD 17 // = TX2
    #define SERIAL_INVERT true

    // ESP32-DevKitC V4 does not have embedded LED
    #ifndef LED_IO
        #define LED_IO  2
    #endif    
    #define USE_LED


//-- ESP32-PICO-KIT
#elif defined MODULE_ESP32_PICO_KIT // ARDUINO_ESP32_PICO, ARDUINO_BOARD = ESP32_PICO
    #ifndef ARDUINO_ESP32_PICO // ARDUINO_BOARD != ESP32_PICO
	      #error Select board ARDUINO_ESP32_PICO!
    #endif

    #undef USE_SERIAL_DBG1
    #define USE_SERIAL1_DBG
    #undef USE_SERIAL2_DBG

    #ifndef LED_IO
        #define LED_IO  13
    #endif    
    #define USE_LED


//-- TTGO-MICRO32
#elif defined MODULE_TTGO_MICRO32 // ARDUINO_ESP32_PICO, ARDUINO_BOARD = ESP32_PICO
    #ifndef ARDUINO_ESP32_PICO // ARDUINO_BOARD != ESP32_PICO
	      #error Select board ARDUINO_ESP32_PICO!
    #endif

    #undef USE_SERIAL_DBG1
    #undef USE_SERIAL1_DBG
    #undef USE_SERIAL2_DBG

    #ifndef LED_IO
        #define LED_IO  13
    #endif    
    #define USE_LED


//-- Adafruit QT Py S2
#elif defined MODULE_ADAFRUIT_QT_PY_ESP32_S2 // ARDUINO_ADAFRUIT_QTPY_ESP32S2, ARDUINO_BOARD == ADAFRUIT_QTPY_ESP32S2
    #ifndef ARDUINO_ADAFRUIT_QTPY_ESP32S2 // ARDUINO_BOARD != ADAFRUIT_QTPY_ESP32S2
	      #error Select board ADAFRUIT_QTPY_ESP32S2!
    #endif		

    #undef USE_SERIAL_DBG1
    #define USE_SERIAL1_DBG
    #undef USE_SERIAL2_DBG
    
    #undef LED_IO
    #define USE_LED
    #define NUMPIXELS  1
    // board defines PIN_NEOPIXEL


//-- M5Stack M5Stamp C3 Mate
#elif defined MODULE_M5STAMP_C3_MATE // ARDUINO_ESP32C3_DEV, ARDUINO_BOARD == ESP32C3_DEV
    #ifndef ARDUINO_ESP32C3_DEV // ARDUINO_BOARD != ESP32C3_DEV
	      #error Select board ESP32C3_DEV!
    #endif		

    #undef USE_SERIAL_DBG1
    #define USE_SERIAL1_DBG
    #undef USE_SERIAL2_DBG

    #undef LED_IO
    #define USE_LED
    #define NUMPIXELS  1
    #define PIN_NEOPIXEL  2


//-- M5 STAMP PICO
#elif defined MODULE_M5STAMP_PICO_FOR_FRSKY_R9M // M5STAMP_PICO , ARDUINO_BOARD = ESP32_PICO
    #ifndef ARDUINO_ESP32_PICO // ARDUINO_BOARD != ESP32_PICO
	      #error Select board ARDUINO_ESP32_PICO!
    #endif

    #undef USE_SERIAL_DBG1
    #define USE_SERIAL1_DBG
    #undef USE_SERIAL2_DBG

    #undef LED_IO
    #define USE_LED
    #define NUMPIXELS  1
    #define PIN_NEOPIXEL  27

    #define SERIAL_RXD  32 // = RX1
    #define SERIAL_TXD  33 // = TX1
    #define SERIAL_INVERT true


//-- M5Stack M5Stamp C3U Mate
#elif defined MODULE_M5STAMP_C3U_MATE_FOR_FRSKY_R9M // ARDUINO_ESP32C3_DEV, ARDUINO_BOARD == ESP32C3_DEV
    #ifndef ARDUINO_ESP32C3_DEV // ARDUINO_BOARD != ESP32C3_DEV
	      #error Select board ESP32C3_DEV!
    #endif

    #undef USE_SERIAL_DBG1
    #define USE_SERIAL1_DBG
    #undef USE_SERIAL2_DBG

    #undef LED_IO
    #define USE_LED
    #define NUMPIXELS  1
    #define PIN_NEOPIXEL  2

    #define SERIAL_RXD  1 // = RX1
    #define SERIAL_TXD  0 // = TX1
    #define SERIAL_INVERT true


//-- M5Stack ATOM Lite
#elif defined MODULE_M5STACK_ATOM_LITE
    #ifndef ARDUINO_M5Stack_ATOM // ARDUINO_BOARD != ARDUINO_M5Stack_ATOM
	      #error Select board ARDUINO_M5Stack_ATOM!
    #endif

    #undef USE_SERIAL_DBG1
    #undef USE_SERIAL1_DBG
    #undef USE_SERIAL2_DBG

    #define SERIAL_RXD 32 // = RX1
    #define SERIAL_TXD 26 // = TX1

    #undef LED_IO
    #define USE_LED
    #define NUMPIXELS  1
    #define PIN_NEOPIXEL  27


//-- Generic
#else
    #ifdef LED_IO  
        #define USE_LED
    #endif
#endif


//-------------------------------------------------------
// internals
//-------------------------------------------------------

#ifdef NUMPIXELS
    #include <Adafruit_NeoPixel.h> // Requires library install
    Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

    void led_init(void)
    {
        #if defined(NEOPIXEL_POWER)
        pinMode(NEOPIXEL_POWER, OUTPUT);
        digitalWrite(NEOPIXEL_POWER, HIGH);
        #endif

        pixels.begin();
        pixels.setBrightness(20); // not so bright
    }

    void led_on(bool is_connected)
    {
        pixels.fill((is_connected) ? 0x00FF00 : 0xFF0000); // green | red
        pixels.show();
    }

    void led_off(void)
    {
        pixels.fill(0x000000); // off
        pixels.show();
    }
#endif


#if defined USE_SERIAL_DBG1
    #define SERIAL Serial
    #define DBG Serial1
    #define DBG_PRINT(x) Serial1.print(x)
    #define DBG_PRINTLN(x) Serial1.println(x)

#elif defined USE_SERIAL1_DBG
    #define SERIAL Serial1
//    #define SERIAL_RXD  9 // = RX1
//    #define SERIAL_TXD  10 // = TX1
//    #define SERIAL_INVERT false
    #define DBG Serial
    #define DBG_PRINT(x) Serial.print(x)
    #define DBG_PRINTLN(x) Serial.println(x)

#elif defined USE_SERIAL2_DBG
    #define SERIAL Serial2
    #define DBG Serial
    #define DBG_PRINT(x) Serial.print(x)
    #define DBG_PRINTLN(x) Serial.println(x)

#else    
    #define SERIAL Serial
    #define DBG_PRINT(x)
    #define DBG_PRINTLN(x)
#endif


#ifdef DBG
    void dbg_init(void) 
    {
        DBG.begin(115200);
        DBG_PRINTLN();
        DBG_PRINTLN("Hello");
    }
#else
    void dbg_init(void) {}
#endif    


#if defined LED_IO && defined USE_LED
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
#endif

#ifndef USE_LED
    void led_init(void) {}
    void led_on(bool is_connected) {}
    void led_off(void) {}
#endif    

