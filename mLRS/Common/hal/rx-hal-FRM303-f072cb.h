//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// RX FRM303 STM32F072CB
//-------------------------------------------------------
// this MCU has no DWT
// 4 UARTS, but U3 & U4 are on same IRQ so cannot be used both
// T1, T2 (32b), T3, T14, T15, T16, T17, internal T6, T7

#define DEVICE_HAS_OUT
#define DEVICE_HAS_BUZZER
//#define DEVICE_HAS_DEBUG_SWUART
#define DEVICE_HAS_SYSTEMBOOT

#ifdef DEBUG_ENABLED
#undef DEBUG_ENABLED
#endif


// -- IRQ priorities
// This is very IMPORTANT! The F07x has only 2bits priority level.
// Note: NVIC_SetPriority() does priority << (8 - __NVIC_PRIO_BITS), so if 4bit priorities are given to it
// while __NVIC_PRIO_BITS = 2, then they get quite disordered:
// 10 = 0b1010 -> 0b10 = 2
// 11 = 0b1011 -> 0b11 = 3
// 13 = 0b1101 -> 0b01 = 1
// 14 = 0b1110 -> 0b10 = 2
// 15 = 0b1111 -> 0b11 = 3
#undef CLOCK_IRQ_PRIORITY
#undef UARTB_IRQ_PRIORITY
#undef UART_IRQ_PRIORITY
#undef UARTC_IRQ_PRIORITY
#undef SX_DIO_EXTI_IRQ_PRIORITY
#undef SX2_DIO_EXTI_IRQ_PRIORITY
#undef SWUART_TIM_IRQ_PRIORITY
#undef BUZZER_TIM_IRQ_PRIORITY
#define CLOCK_IRQ_PRIORITY          0 // 10
#define UARTB_IRQ_PRIORITY          1 // 11 // serial
#define UART_IRQ_PRIORITY           2 // 12 // out pin
#define UARTC_IRQ_PRIORITY          1 // 11 // debug
#define SX_DIO_EXTI_IRQ_PRIORITY    2 // 13
#define SX2_DIO_EXTI_IRQ_PRIORITY   2 // 13
#define SWUART_TIM_IRQ_PRIORITY     0 // 11 // debug on swuart
#define BUZZER_TIM_IRQ_PRIORITY     3 // 14


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_TIM7_W_INIT //DELAY_USE_DWT,
static inline void delay_ns(uint32_t ns) { __NOP();__NOP(); __NOP(); __NOP(); } // 48 MHz => 4x nop = 100 ns

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             60 // 128 kB flash, 2 kB page

#define MICROS_TIMx               TIM16

#define CLOCK_TIMx                TIM1
#define CLOCK_IRQn                TIM1_CC_IRQn
#define CLOCK_IRQHandler          TIM1_CC_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTC = debug port

#define UARTB_USE_UART2 // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_UART1 // out pin
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            256
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512

/*
#define SWUART_USE_TIM17 // debug
#define SWUART_TX_IO              IO_PB14 // that's the I2C2 SDA pin
#define SWUART_BAUD               115200
#define SWUART_USE_TX
#define SWUART_TXBUFSIZE          512
//#define SWUART_TIM_IRQ_PRIORITY   11
*/


//-- SX12xx & SPI

#define SPI_USE_SPI1_PB3PB4PB5    // PB3, PB4, PB5
#define SPI_CS_IO                 IO_PA15
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_18MHZ  // equals to 12 MHz

#define SX_RESET                  IO_PA0
#define SX_DIO1                   IO_PB10
#define SX_BUSY                   IO_PB6
#define SX_RX_EN                  IO_PB1
#define SX_TX_EN                  IO_PA4
#define SX_PA_EN                  IO_PA5

#define SX_DIO1_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTB
#define SX_DIO1_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE10
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_10
#define SX_DIO_EXTI_IRQn              EXTI4_15_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI4_15_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY   11

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX_PA_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

void sx_amp_transmit(void)
{
    gpio_low(SX_RX_EN);
    gpio_high(SX_PA_EN);
    gpio_high(SX_TX_EN);
}

void sx_amp_receive(void)
{
    gpio_low(SX_TX_EN);
    gpio_low(SX_PA_EN);
    gpio_high(SX_RX_EN);
}

void sx_dio_init_exti_isroff(void)
{
    LL_SYSCFG_SetEXTISource(SX_DIO1_SYSCFG_EXTI_PORTx, SX_DIO1_SYSCFG_EXTI_LINEx);

    // let's not use LL_EXTI_Init(), but let's do it by hand, is easier to allow enabling isr later
    LL_EXTI_DisableEvent_0_31(SX_DIO_EXTI_LINE_x);
    LL_EXTI_DisableIT_0_31(SX_DIO_EXTI_LINE_x);
    LL_EXTI_DisableFallingTrig_0_31(SX_DIO_EXTI_LINE_x);
    LL_EXTI_EnableRisingTrig_0_31(SX_DIO_EXTI_LINE_x);

    NVIC_SetPriority(SX_DIO_EXTI_IRQn, SX_DIO_EXTI_IRQ_PRIORITY);
    NVIC_EnableIRQ(SX_DIO_EXTI_IRQn);
}

void sx_dio_enable_exti_isr(void)
{
    LL_EXTI_ClearFlag_0_31(SX_DIO_EXTI_LINE_x);
    LL_EXTI_EnableIT_0_31(SX_DIO_EXTI_LINE_x);
}

void sx_dio_exti_isr_clearflag(void)
{
    LL_EXTI_ClearFlag_0_31(SX_DIO_EXTI_LINE_x);
}


//-- SX12xx II & SPIB
// has none


//-- Out port

#define OUT_UARTx                 USART1

void out_init_gpio(void)
{
}

void out_set_normal(void)
{
    LL_USART_Disable(OUT_UARTx);
    LL_USART_SetTXPinLevel(OUT_UARTx, LL_USART_TXPIN_LEVEL_STANDARD);
    LL_USART_Enable(OUT_UARTx);
}

void out_set_inverted(void)
{
    LL_USART_Disable(OUT_UARTx);
    LL_USART_SetTXPinLevel(OUT_UARTx, LL_USART_TXPIN_LEVEL_INVERTED);
    LL_USART_Enable(OUT_UARTx);
}


//-- Button
// has none, use 5 way on PA7, down

#define BUTTON                    IO_PA7

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON);
}


//-- LEDs

#define LED_GREEN                 IO_PB8
#define LED_RED                   IO_PB7
#define LED_RIGHT_GREEN           IO_PB9 // blue

void leds_init(void)
{
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
    gpio_init(LED_RIGHT_GREEN, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
}

void led_green_off(void) { gpio_high(LED_GREEN); }
void led_green_on(void) { gpio_low(LED_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

void led_red_off(void) { gpio_high(LED_RED); }
void led_red_on(void) { gpio_low(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- Buzzer
// Buzzer is active high

#define BUZZER                    IO_PB11
#define BUZZER_IO_AF              IO_AF_2
#define BUZZER_TIMx               TIM2
#define BUZZER_IRQn               TIM2_IRQn
#define BUZZER_IRQHandler         TIM2_IRQHandler
#define BUZZER_TIM_CHANNEL        LL_TIM_CHANNEL_CH4
//#define BUZZER_TIM_IRQ_PRIORITY   14


//-- SystemBootLoader
// go into boot if FIVEWAY is DOWN during power up
// FIVEWAY-DOWN becomes bind button later on

#ifdef DEVICE_HAS_SYSTEMBOOT
void systembootloader_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (gpio_read_activelow(BUTTON)) cnt++;
    }
    if (cnt > 12) {
        BootLoaderInit();
    }
}
#endif


//-- POWER

#define POWER_GAIN_DBM            24 // 35 // gain of a PA stage if present // datasheet of SKY66312-11 says 35dB !
#define POWER_SX1280_MAX_DBM      SX1280_POWER_6_DBM //SX1280_POWER_m3_DBM // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12,
    LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, /*LL_GPIO_PIN_7,*/
    /*LL_GPIO_PIN_8,*/ /*LL_GPIO_PIN_9,*/ LL_GPIO_PIN_10, /*LL_GPIO_PIN_11,*/ LL_GPIO_PIN_12, LL_GPIO_PIN_13,
    LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};








