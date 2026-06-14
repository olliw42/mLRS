/**
  ******************************************************************************
  * @file    usbd_conf_template.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_conf_template.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif


// can we pl somehow do this elsewhere, better?
// USB_RXBUFSIZE, USB_TXBUFSIZE we can do in stdstm32-usb-vcp.h, but not USBD_IRQ_PRIORITY
// so we do all here to have them together, but not nice
#if defined STM32F072xB
#define USB_RXBUFSIZE       256
#define USB_TXBUFSIZE       256
#elif defined STM32G431xx || defined STM32G441xx || defined STM32G491xx || defined STM32G474xx
#define USB_RXBUFSIZE       2048 // for serial
#define USB_TXBUFSIZE       2048 // helps with cli
#else
#define USB_RXBUFSIZE       512
#define USB_TXBUFSIZE       512
#endif
#define USBD_IRQ_PRIORITY   0


#include <stdlib.h>
#include <string.h>

#if defined STM32F103xE
  #include "stm32f1xx.h"
  #include "stm32f1xx_hal.h"
  #define USBD_IRQn         USB_LP_IRQn
  #define USBD_IRQHandler   USB_LP_IRQHandler
  #define USBD_INST         USB
#elif defined STM32G431xx || defined STM32G441xx || defined STM32G491xx || defined STM32G474xx
  #include "stm32g4xx.h"
  #include "stm32g4xx_hal.h"
  #define USBD_IRQn         USB_LP_IRQn
  #define USBD_IRQHandler   USB_LP_IRQHandler
  #define USBD_INST         USB
#elif defined STM32F072xB
  #include "stm32f0xx.h"
  #include "stm32f0xx_hal.h"
  #define USBD_IRQn         USB_IRQn
  #define USBD_IRQHandler   USB_IRQHandler
  #define USBD_INST         USB
#else
  #error STM32 device not supported by USBD library !
#endif


#define USBD_MAX_NUM_INTERFACES                     1U
#define USBD_MAX_NUM_CONFIGURATION                  1U
#define USBD_MAX_STR_DESC_SIZ                       0x100U
#define USBD_SELF_POWERED                           1U
#define USBD_DEBUG_LEVEL                            0U
/* #define USBD_USER_REGISTER_CALLBACK                 1U */
#define USBD_LPM_ENABLED                            0U

/* CDC Class Config */
#define USBD_CDC_INTERVAL                           2000U


/* Memory management macros make sure to use static memory allocation */
/** Alias for memory allocation. */
#define USBD_malloc         (void *)USBD_static_malloc

/** Alias for memory release. */
#define USBD_free           USBD_static_free

/** Alias for memory set. */
#define USBD_memset         memset

/** Alias for memory copy. */
//#define USBD_memcpy         memcpy

/** Alias for delay. */
//#define USBD_Delay          HAL_Delay


/* DEBUG macros */
#if (USBD_DEBUG_LEVEL > 0U)
#define  USBD_UsrLog(...)   do { \
                                 printf(__VA_ARGS__); \
                                 printf("\n"); \
                               } while (0)
#else
#define USBD_UsrLog(...) do {} while (0)
#endif /* (USBD_DEBUG_LEVEL > 0U) */

#if (USBD_DEBUG_LEVEL > 1U)

#define  USBD_ErrLog(...) do { \
                               printf("ERROR: ") ; \
                               printf(__VA_ARGS__); \
                               printf("\n"); \
                             } while (0)
#else
#define USBD_ErrLog(...) do {} while (0)
#endif /* (USBD_DEBUG_LEVEL > 1U) */

#if (USBD_DEBUG_LEVEL > 2U)
#define  USBD_DbgLog(...)   do { \
                                 printf("DEBUG : ") ; \
                                 printf(__VA_ARGS__); \
                                 printf("\n"); \
                               } while (0)
#else
#define USBD_DbgLog(...) do {} while (0)
#endif /* (USBD_DEBUG_LEVEL > 2U) */


void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);



#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_H */
