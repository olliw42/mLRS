//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// ESP Glue
//*******************************************************


//-------------------------------------------------------
// what we'll use anyway
//-------------------------------------------------------

#include <inttypes.h>
#include <string.h>


//-------------------------------------------------------
// ESP
//-------------------------------------------------------

#include <Arduino.h>

// needed to switch off the wifi module. 
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32) 
#include <WiFi.h>
#endif


// used by Arduino to reset the microcontroller 
void resetFunc(void)
{
    ESP.restart();
}


void __disable_irq(void)
{
    noInterrupts();
}


void __enable_irq(void)
{
    interrupts();
}


// https://forum.arduino.cc/t/very-short-delays/43445
#define __NOP() __asm__("nop")


void hal_init(void)
{
    WiFi.mode(WIFI_OFF);
}
