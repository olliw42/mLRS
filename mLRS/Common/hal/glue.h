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
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"

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
#include "stm32f3xx_hal_flash.h"
#include "stm32f3xx_hal_flash_ex.h"

#endif
#if (defined STM32G491xx) || (defined STM32G441xx)  || (defined STM32G431xx)

#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include "stm32g4xx_ll_lpuart.h"

#endif
#ifdef STM32L433xx

#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_flash.h"
#include "stm32l4xx_hal_flash_ex.h"

#endif
#ifdef STM32WLE5xx

#include "stm32wlxx_ll_cortex.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_ll_gpio.h"
#include "stm32wlxx_ll_tim.h"
#include "stm32wlxx_ll_usart.h"
#include "stm32wlxx_ll_spi.h"
#include "stm32wlxx_ll_system.h"
#include "stm32wlxx_ll_exti.h"
//#include "stm32wlxx_ll_lpuart.h"
#include "stm32wlxx_hal.h"
#include "stm32wlxx_hal_flash.h"
#include "stm32wlxx_hal_flash_ex.h"

#endif


//-------------------------------------------------------
// some useful defines
//-------------------------------------------------------



