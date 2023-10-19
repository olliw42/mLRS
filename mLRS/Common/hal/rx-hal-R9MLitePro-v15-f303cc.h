//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// R9M Lite Pro v1.5 TX Module STM32F303CC as RECEIVER
//-------------------------------------------------------

#define DEVICE_HAS_OUT

#ifdef DEBUG_ENABLED
#undef DEBUG_ENABLED
#endif


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             124 // 256 kB flash, 2 kB page

#define MICROS_TIMx               TIM3

#define CLOCK_TIMx                TIM2
#define CLOCK_IRQn                TIM2_IRQn
#define CLOCK_IRQHandler          TIM2_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTC = debug port

#define UARTB_USE_UART2 // serial PA2/PA3
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE
#define UARTB_INVERTED

#define UART_USE_UART3 // out pin PB10
#define UART_USE_TX_IO            IO_PB10
#define UART_USE_IO_AF            IO_AF_7
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            256
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512
#define OUT_UARTx                 USART3 // UART_UARTx is not known yet, so define by hand

/*
#define UARTC_USE_UART2 // debug // Tx goes via an inverter to JR Pin2, solder to R15 for TTL UART signal, C23 provides GND
#define UARTC_BAUD                115200
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           512
#define UARTC_USE_TX_ISR
//#define UARTC_USE_RX
//#define UARTC_RXBUFSIZE           512
*/

//-- SX1: SX12xx & SPI

#define SPI_USE_SPI2              // PB13, PB14, PB15
#define SPI_CS_IO                 IO_PB12
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PA9
#define SX_DIO0                   IO_PA8
#define SX_SWITCH_RX_EN           IO_PA6

#define SX_DIO0_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTA
#define SX_DIO0_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE8
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_8
#define SX_DIO_EXTI_IRQn              EXTI9_5_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI9_5_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11


void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO0, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_SWITCH_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

void sx_amp_transmit(void)
{
    gpio_low(SX_SWITCH_RX_EN);
}

void sx_amp_receive(void)
{
    gpio_high(SX_SWITCH_RX_EN);
}

void sx_dio_init_exti_isroff(void)
{
    LL_SYSCFG_SetEXTISource(SX_DIO0_SYSCFG_EXTI_PORTx, SX_DIO0_SYSCFG_EXTI_LINEx);

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


//-- Out port
// the R9MLitePro has a fixed inverter

#define OUT_EN                    IO_PB2

void out_init_gpio(void)
{
    gpio_init(OUT_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
}

void out_set_normal(void)
{
    LL_USART_Disable(OUT_UARTx);
    LL_USART_SetTXPinLevel(OUT_UARTx, LL_USART_TXPIN_LEVEL_INVERTED);
    LL_USART_Enable(OUT_UARTx);
}

void out_set_inverted(void)
{
    LL_USART_Disable(OUT_UARTx);
    LL_USART_SetTXPinLevel(OUT_UARTx, LL_USART_TXPIN_LEVEL_STANDARD);
    LL_USART_Enable(OUT_UARTx);
}


//-- Button
// none

void button_init(void)
{
}

bool button_pressed(void)
{
    return false;
}


//-- LEDs

#define LED_GREEN                 IO_PA15
#define LED_RED                   IO_PB3

void leds_init(void)
{
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_low(LED_GREEN); // LED_GREEN_OFF
    gpio_low(LED_RED); // LED_RED_OFF

    // that's the blue LED, it would glim otherwise
    gpio_init(IO_PB4, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
}

void led_green_off(void) { gpio_low(LED_GREEN); }
void led_green_on(void) { gpio_high(LED_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- POWER

#define DEVICE_HAS_INTERNAL_DAC_TWOCHANNELS
#define SX_PA_DAC             DAC1
#define SX_PA_DAC_IO1         IO_PA5
#define SX_PA_DAC_IO2         IO_PA4
#define SX_PA_DAC_CHANNEL1    LL_DAC_CHANNEL_1
#define SX_PA_DAC_CHANNEL2    LL_DAC_CHANNEL_2

void rfpower_calc(int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, tInternalDacBase* dac)
{
    // -25 dBm 0,0 ... 1700
    // -15 dBm 1800
    // 0 dBm   1890,1890
    // 10 dBm  1930,1930
    // 20 dBm  2020,2020
    // 30 dBm  2700,2700
    uint32_t voltage1_mV, voltage2_mV;
    if (power_dbm > 28) {
        voltage1_mV = 2250;
        *actual_power_dbm = 30;
    } else if (power_dbm > 22) {
        voltage1_mV = 2130;
        *actual_power_dbm = 27;
    } else if (power_dbm > 18) {
        voltage1_mV = 2020;
        *actual_power_dbm = 20;
    } else if (power_dbm > 5) {
        voltage1_mV = 1930;
        *actual_power_dbm = 10;
    } else {
        voltage1_mV = 1800;
        *actual_power_dbm = -15;
    }

    voltage2_mV = voltage1_mV;

    uint16_t value1 = (voltage1_mV * 4096) / 3300; // don't bother with rounding
    uint16_t value2 = (voltage2_mV * 4096) / 3300; // don't bother with rounding

    dac->put_channel1(value1);
    dac->put_channel2(value2);

    *sx_power = 0; // minimum power level, is 2 dBm
}

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, /*LL_GPIO_PIN_10,*/
    LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11,
    LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
};


