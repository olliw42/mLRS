#ifndef STDSTM32_LL_MCU_H
#define STDSTM32_LL_MCU_H
#ifdef __cplusplus
extern "C" {
#endif

//*******************************************************
// MCU standard interface
//*******************************************************


//-------------------------------------------------------
// BootLoaderInit
//-------------------------------------------------------

// see AN2606 for system flash start location
#ifdef STM32F1
#define ST_BOOTLOADER_ADDRESS               0x1FFFF000 // = SystemMemory: F103T8 F103CB F103RC
#elif defined STM32G4 || defined STM32L4 || defined STM32WL
#define ST_BOOTLOADER_ADDRESS               0x1FFF0000
#else
  #warning ST_BOOTLOADER_ADDRESS no defined for chip !
#endif


void (*SysMemBootJump)(void);

void BootLoaderInit(void)
{
    SysMemBootJump = (void (*)(void)) (*((uint32_t*)(ST_BOOTLOADER_ADDRESS+4))); // point PC to system memory reset vector

    HAL_DeInit(); // is important

    // shut down any running tasks
    LL_GPIO_DeInit(GPIOA);
    LL_GPIO_DeInit(GPIOB);
    LL_GPIO_DeInit(GPIOC);
    LL_USART_DeInit(USART1);

    LL_RCC_DeInit();

    // reset systick timer
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // select HSI as system clock source
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI); // done already in LL_RCC_Deinit() !?

    // disable interrupts
    __set_PRIMASK(1);

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
