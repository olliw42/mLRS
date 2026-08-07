//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down EXTI standard library
//*******************************************************
#ifndef STDSTM32_LL_EXTI_H
#define STDSTM32_LL_EXTI_H
#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
  EXTI_TRIG_RISING = 0,
  EXTI_TRIG_FALLING,
  EXTI_TRIG_RISING_FALLING,
} EXTI_TRIG_ENUM;



#if defined STM32F1
  #define _LL_EXTI_APPEND(x)  LL_GPIO_AF_EXTI ## x
#elif defined STM32G4 || defined STM32F3 || defined STM32WL || defined STM32L4
  #define _LL_EXTI_APPEND(x)  LL_SYSCFG_EXTI ## x
#else
  #error MCU not supported by EXTI library!
#endif
#define LL_EXTI_APPEND(x)   _LL_EXTI_APPEND(x)


#if defined(GPIOA)
#define EXTI_IO_PA0   LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE0), LL_EXTI_LINE_0
#define EXTI_IO_PA1   LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE1), LL_EXTI_LINE_1
#define EXTI_IO_PA2   LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE2), LL_EXTI_LINE_2
#define EXTI_IO_PA3   LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE3), LL_EXTI_LINE_3
#define EXTI_IO_PA4   LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE4), LL_EXTI_LINE_4
#define EXTI_IO_PA5   LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE5), LL_EXTI_LINE_5
#define EXTI_IO_PA6   LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE6), LL_EXTI_LINE_6
#define EXTI_IO_PA7   LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE7), LL_EXTI_LINE_7

#define EXTI_IO_PA8   LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE8), LL_EXTI_LINE_8
#define EXTI_IO_PA9   LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE9), LL_EXTI_LINE_9
#define EXTI_IO_PA10  LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE10), LL_EXTI_LINE_10
#define EXTI_IO_PA11  LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE11), LL_EXTI_LINE_11
#define EXTI_IO_PA12  LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE12), LL_EXTI_LINE_12
#define EXTI_IO_PA13  LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE13), LL_EXTI_LINE_13
#define EXTI_IO_PA14  LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE14), LL_EXTI_LINE_14
#define EXTI_IO_PA15  LL_EXTI_APPEND(_PORTA), LL_EXTI_APPEND(_LINE15), LL_EXTI_LINE_15
#endif

#if defined(GPIOB)
#define EXTI_IO_PB0   LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE0), LL_EXTI_LINE_0
#define EXTI_IO_PB1   LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE1), LL_EXTI_LINE_1
#define EXTI_IO_PB2   LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE2), LL_EXTI_LINE_2
#define EXTI_IO_PB3   LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE3), LL_EXTI_LINE_3
#define EXTI_IO_PB4   LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE4), LL_EXTI_LINE_4
#define EXTI_IO_PB5   LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE5), LL_EXTI_LINE_5
#define EXTI_IO_PB6   LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE6), LL_EXTI_LINE_6
#define EXTI_IO_PB7   LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE7), LL_EXTI_LINE_7

#define EXTI_IO_PB8   LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE8), LL_EXTI_LINE_8
#define EXTI_IO_PB9   LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE9), LL_EXTI_LINE_9
#define EXTI_IO_PB10  LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE10), LL_EXTI_LINE_10
#define EXTI_IO_PB11  LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE11), LL_EXTI_LINE_11
#define EXTI_IO_PB12  LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE12), LL_EXTI_LINE_12
#define EXTI_IO_PB13  LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE13), LL_EXTI_LINE_13
#define EXTI_IO_PB14  LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE14), LL_EXTI_LINE_14
#define EXTI_IO_PB15  LL_EXTI_APPEND(_PORTB), LL_EXTI_APPEND(_LINE15), LL_EXTI_LINE_15
#endif

#if defined(GPIOC)
#define EXTI_IO_PC0   LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE0), LL_EXTI_LINE_0
#define EXTI_IO_PC1   LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE1), LL_EXTI_LINE_1
#define EXTI_IO_PC2   LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE2), LL_EXTI_LINE_2
#define EXTI_IO_PC3   LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE3), LL_EXTI_LINE_3
#define EXTI_IO_PC4   LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE4), LL_EXTI_LINE_4
#define EXTI_IO_PC5   LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE5), LL_EXTI_LINE_5
#define EXTI_IO_PC6   LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE6), LL_EXTI_LINE_6
#define EXTI_IO_PC7   LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE7), LL_EXTI_LINE_7

#define EXTI_IO_PC8   LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE8), LL_EXTI_LINE_8
#define EXTI_IO_PC9   LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE9), LL_EXTI_LINE_9
#define EXTI_IO_PC10  LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE10), LL_EXTI_LINE_10
#define EXTI_IO_PC11  LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE11), LL_EXTI_LINE_11
#define EXTI_IO_PC12  LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE12), LL_EXTI_LINE_12
#define EXTI_IO_PC13  LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE13), LL_EXTI_LINE_13
#define EXTI_IO_PC14  LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE14), LL_EXTI_LINE_14
#define EXTI_IO_PC15  LL_EXTI_APPEND(_PORTC), LL_EXTI_APPEND(_LINE15), LL_EXTI_LINE_15
#endif


static inline void exti_enableisr(uint32_t Port, uint32_t Line, uint32_t ExtiLine)
{
  LL_EXTI_ClearFlag_0_31(ExtiLine);
  LL_EXTI_EnableIT_0_31(ExtiLine);
}


static inline void exti_clearisrflag(uint32_t Port, uint32_t Line, uint32_t ExtiLine)
{
  LL_EXTI_ClearFlag_0_31(ExtiLine);
}


void exti_init_isroff(uint32_t Port, uint32_t Line, uint32_t ExtiLine, uint32_t trigger)
{
  // gpio_init(GPIOx, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);

#ifdef STM32F1
  LL_GPIO_AF_SetEXTISource(Port, Line);
#elif defined STM32G4 || defined STM32F3 || defined STM32WL || defined STM32L4
  LL_SYSCFG_SetEXTISource(Port, Line);
#endif

  // let's not use LL_EXTI_Init(), but let's do it by hand, is easier to allow enabling isr later
  LL_EXTI_DisableEvent_0_31(ExtiLine);
  LL_EXTI_DisableIT_0_31(ExtiLine);

  //LL_EXTI_DisableFallingTrig_0_31(ExtiLine);
  //LL_EXTI_EnableRisingTrig_0_31(ExtiLine);
  switch (trigger) {
  case EXTI_TRIG_RISING:
    LL_EXTI_DisableFallingTrig_0_31(ExtiLine);
    LL_EXTI_EnableRisingTrig_0_31(ExtiLine);
    break;
  case EXTI_TRIG_FALLING:
    LL_EXTI_DisableRisingTrig_0_31(ExtiLine);
    LL_EXTI_EnableFallingTrig_0_31(ExtiLine);
    break;
  case EXTI_TRIG_RISING_FALLING:
    LL_EXTI_EnableFallingTrig_0_31(ExtiLine);
    LL_EXTI_EnableRisingTrig_0_31(ExtiLine);
    break;
  }

  //NVIC_SetPriority(SX_DIO_EXTI_IRQn, SX_DIO_EXTI_IRQ_PRIORITY);
  //NVIC_EnableIRQ(SX_DIO_EXTI_IRQn);
}


void exti_init(uint32_t Port, uint32_t Line, uint32_t ExtiLine, uint32_t trigger)
{
  exti_init_isroff(Port, Line, ExtiLine, trigger);
  exti_enableisr(Port, Line, ExtiLine);
}


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_LL_EXTI_H
