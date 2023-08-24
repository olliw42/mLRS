//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Stack check
//*******************************************************
#ifndef STACK_CHECK_H
#define STACK_CHECK_H
#pragma once


#include <inttypes.h>


//-------------------------------------------------------
//
//-------------------------------------------------------
// see
// - https://mcuoneclipse.com/2016/11/01/getting-the-memory-range-of-sections-with-gnu-linker-files/
// - https://stackoverflow.com/questions/55622174/is-accessing-the-value-of-a-linker-script-variable-undefined-behavior-in-c
extern uint32_t _estack;
extern uint32_t _Min_Stack_Size;


uint32_t stack_check_used(void)
{
    uint32_t ptr = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
    uint32_t end = (uint32_t)&_estack;
    while (ptr < end) {
        if (*(uint8_t*)ptr != 0xAA) break;
        ptr++;
    }

    return (end - ptr);
}


void stack_check_init(void)
{
    // paint the stack memory area
    uint32_t ptr = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
    //uint32_t end = (uint32_t)&_estack;
    uint32_t end = __get_MSP() - 1; // let's use MSP to avoid concerns with used stack space at time of function call
    while (ptr < end) {
        *(uint8_t*)ptr = 0xAA;
        ptr++;
    }
}


#endif // STACK_CHECK_H
