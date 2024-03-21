//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Micros
//********************************************************
#ifndef MICROS_H
#define MICROS_H
#pragma once

// free running timer with 1us time base


#ifndef MICROS_TIMx
#error MICROS_TIMx not defined !
#endif


//-------------------------------------------------------
// Micros functions
//-------------------------------------------------------

void micros_init(void)
{
    tim_init_1us_freerunning(MICROS_TIMx);
}


uint16_t micros16(void)
{
    return MICROS_TIMx->CNT;
}


#endif // MICROS_H
