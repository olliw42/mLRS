//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// glue
//*******************************************************


//-------------------------------------------------------
// what we'll use anyway
//-------------------------------------------------------

#include <inttypes.h>
#include <string.h>


//-------------------------------------------------------
// STm32Cube LL & HAL
//-------------------------------------------------------
// STM32F1 etc are not yet defined, so we need to be explicit

#ifdef STM32F103xB

#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_hal.h"

#endif
#ifdef STM32F373xC

#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_spi.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_hal.h"

#endif



//-------------------------------------------------------
// some useful defines
//-------------------------------------------------------
/*
#ifdef __cplusplus
#  define IRQHANDLER(__Declaration__)  extern "C" {__Declaration__}
#else
#  define IRQHANDLER(__Declaration__)  __Declaration__
#endif
*/


