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


#ifndef TIM3 // the L433xx do not have a TIM3
#define MICROS_TIMx               TIM15
#else
#define MICROS_TIMx               TIM3
#endif


//-------------------------------------------------------
// Micros functions
//-------------------------------------------------------

void micros_init(void)
{
  tim_init_1us_freerunning(MICROS_TIMx);
}


uint16_t micros(void)
{
  return MICROS_TIMx->CNT;
}


#endif // MICROS_H
