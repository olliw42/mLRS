//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down MCU standard library
//*******************************************************
#ifndef STDSTM32_LL_MCU_H
#define STDSTM32_LL_MCU_H
#ifdef __cplusplus
extern "C" {
#endif


//-------------------------------------------------------
// BootLoaderInit
//-------------------------------------------------------

// see AN2606 for system flash start location
#ifdef STM32F1
#define ST_BOOTLOADER_ADDRESS               0x1FFFF000 // = SystemMemory: F103T8 F103CB F103RC
#elif defined STM32F3
#define ST_BOOTLOADER_ADDRESS               0x1FFFD800
#elif defined STM32G4 || defined STM32L4 || defined STM32WL
#define ST_BOOTLOADER_ADDRESS               0x1FFF0000
#elif defined STM32F070xB || defined STM32F072xB // system memory location varies across the STM32F0 family
#define ST_BOOTLOADER_ADDRESS               0x1FFFC800
#else
  #warning ST_BOOTLOADER_ADDRESS not defined for chip !
#endif


void (*SysMemBootJump)(void);


#ifdef STDSTM32_USE_USB
#include "stdstm32-usb-vcp.h"
#endif


void BootLoaderInit(void)
{
    SysMemBootJump = (void (*)(void)) (*((uint32_t*)(ST_BOOTLOADER_ADDRESS+4))); // point PC to system memory reset vector

#ifdef STDSTM32_USE_USB
    usb_deinit();
#endif

    HAL_DeInit(); // is important

    // shut down any running tasks, done already by HAL_DeInit()!
    LL_GPIO_DeInit(GPIOA);
    LL_GPIO_DeInit(GPIOB);
    LL_GPIO_DeInit(GPIOC);
    LL_USART_DeInit(USART1);
#ifdef USART2
    LL_USART_DeInit(USART2);
#endif

    LL_RCC_DeInit(); // HAL_RCC_DeInit(); ?? ATTENTION: HAL_RCC_DeInit() uses SysTick!

    // reset systick timer
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // select HSI as system clock source
    //LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI); // is done already in LL_RCC_Deinit() !?

    // disable interrupts
    //__set_PRIMASK(1);
    // this is what works for F0 to enter DFU!
    __disable_irq();
    for (uint8_t i = 0; i < (sizeof(NVIC->ICER) / sizeof(*NVIC->ICER)); i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    __enable_irq();

    // remap system memory
    // stated in several sources, but doesn't seem to be relevant
    // SYSCFG->CFGR1 = 0x01;
    // SYSCFG->MEMRMP = 0x01;
    // __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    //LL_SYSCFG_SetRemapMemory(LL_SYSCFG_REMAP_SYSTEMFLASH);

    // set main stack pointer to its default
    __set_MSP( *((volatile uint32_t*)ST_BOOTLOADER_ADDRESS) );
    // jump
    SysMemBootJump();
    while(1);
}


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif  // STDSTM32_LL_MCU_H
