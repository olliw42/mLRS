//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Peripherals
//********************************************************
#ifndef ESPLIB_PERIPHERALS_H
#define ESPLIB_PERIPHERALS_H


void gpio_low(uint8_t GPIO_Pin)
{
    digitalWrite(GPIO_Pin, LOW);
}


void gpio_high(uint8_t GPIO_Pin)
{
    digitalWrite(GPIO_Pin, HIGH);
}


void gpio_toggle(uint8_t GPIO_Pin)
{
    digitalWrite(GPIO_Pin, !digitalRead(GPIO_Pin));
}


#endif // ESPLIB_PERIPHERALS_H
