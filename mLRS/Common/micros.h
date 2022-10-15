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


// this should eventually go consistently into hal, allow migration
#ifndef MICROS_TIMx

#if defined TIM3
#define MICROS_TIMx               TIM3
#elif defined TIM15 // the L433xx do not have a TIM3, but have a TIM15
#define MICROS_TIMx               TIM15
#elif defined TIM16 // the WL5xx do not have a TIM3 nor TIM15, but have a TIM16
#define MICROS_TIMx               TIM16
#endif

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
