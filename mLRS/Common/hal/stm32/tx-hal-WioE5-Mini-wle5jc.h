//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************
// 11.Feb.2023: DBG pin changed! LED pin changed! pin for artificial GND (PA0)!
// 5.Aug.2023: jrpin5 changed from JRPIN5_RX_TX_INVERT_INTERNAL to JRPIN5_FULL_INTERNAL
// 5.Sep.2023: jrpin5 and in simultaneously supported

//#define MLRS_DEV_FEATURE_JRPIN5_SDIODE

//-------------------------------------------------------
// TX Seeedstudio Wio-E5 Mini Dev board STM32WLE5JC, https://wiki.seeedstudio.com/LoRa_E5_mini
//-------------------------------------------------------

#define DEVICE_HAS_JRPIN5
//#define DEVICE_HAS_IN
#define DEVICE_HAS_IN_ON_JRPIN5_RX
#define DEVICE_HAS_DEBUG_SWUART
#define DEVICE_HAS_SINGLE_LED
//#define DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL


#ifdef MLRS_DEV_FEATURE_JRPIN5_SDIODE
  #define DEVICE_HAS_JRPIN5
  #undef DEVICE_HAS_IN
  #undef DEVICE_HAS_IN_ON_JRPIN5_RX
#endif


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             120 // 256 kB flash, 2 kB page

#define MICROS_TIMx               TIM16


//-- UARTS
// UARTB = serial port
// UARTC = COM (CLI)
// UARTD = serial2 BT/ESP port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = --
// SWUART= debug port

#define UARTB_USE_UART2_PA2PA3 // serial // PA2,PA3
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTC_USE_UART1_PB6PB7 // com USB/CLI // PB6,PB7
#define UARTC_BAUD                TX_COM_BAUDRATE
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           TX_COM_TXBUFSIZE
#define UARTC_USE_TX_ISR
#define UARTC_USE_RX
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE

#define UART_USE_LPUART1_PC1PC0 // JR pin5, MBridge // PC1,PC0
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#ifndef MLRS_DEV_FEATURE_JRPIN5_SDIODE
#define JRPIN5_FULL_INTERNAL_ON_RX // does not require an external diode
#else
#define JRPIN5_RX_TX_INVERT_INTERNAL // requires external diode from Tx to Rx
#endif

#define UARTE_USE_LPUART1_PC1PC0 // in port // PC0
#define UARTE_BAUD                100000 // SBus normal baud rate, is being set later anyhow
//#define UARTE_USE_TX
//#define UARTE_TXBUFSIZE           512
//#define UARTE_USE_TX_ISR
#define UARTE_USE_RX
#define UARTE_RXBUFSIZE           512

#define SWUART_USE_TIM17 // debug
#define SWUART_TX_IO              IO_PB3
#define SWUART_BAUD               115200
#define SWUART_USE_TX
#define SWUART_TXBUFSIZE          512
//#define SWUART_TIM_IRQ_PRIORITY   9


//-- SX12xx & SPI

#define SPI_USE_SUBGHZSPI
#define SPI_USE_CLOCKSPEED_12MHZ

#define SX_BUSY                   0 // busy is provided by subghz, we need to define a dummy to fool sx126x_driver lib
#define SX_HAS_NO_RESET           // SubGHz has no reset, reset is done by spi_init()

#define SX_RX_EN                  IO_PA4
#define SX_TX_EN                  IO_PA5

//#define SX_DIO1_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTA
//#define SX_DIO1_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE1
//#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_1 // not defining SX_DIO_EXTI_LINE_x has impact on ISR routine
#define SX_DIO_EXTI_IRQn              SUBGHZ_Radio_IRQn
#define SX_DIO_EXTI_IRQHandler        SUBGHZ_Radio_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

void sx_init_gpio(void)
{
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx_busy_read(void)
{
    return subghz_is_busy();
}

void sx_amp_transmit(void)
{
    gpio_low(SX_RX_EN);
    gpio_high(SX_TX_EN);
}

void sx_amp_receive(void)
{
    gpio_low(SX_TX_EN);
    gpio_high(SX_RX_EN);
}

void sx_dio_init_exti_isroff(void)
{
    // there is no EXTI_LINE_44 interrupt flag
    //LL_EXTI_DisableEvent_32_63(SX_DIO_EXTI_LINE_x);
    //LL_EXTI_DisableIT_32_63(SX_DIO_EXTI_LINE_x);

    NVIC_SetPriority(SX_DIO_EXTI_IRQn, SX_DIO_EXTI_IRQ_PRIORITY);
    //NVIC_EnableIRQ(SX_DIO_EXTI_IRQn);
}

void sx_dio_enable_exti_isr(void)
{ /*
    LL_EXTI_ClearFlag_0_31(SX_DIO_EXTI_LINE_x);
    LL_EXTI_EnableIT_0_31(SX_DIO_EXTI_LINE_x); */

    // there is no EXTI_LINE_44 interrupt flag
    //LL_EXTI_ClearFlag_32_63(SX_DIO_EXTI_LINE_x);
    //LL_EXTI_EnableIT_32_63(SX_DIO_EXTI_LINE_x);
    NVIC_EnableIRQ(SX_DIO_EXTI_IRQn);
}

void sx_dio_exti_isr_clearflag(void)
{
    // there is no EXTI_LINE_44 interrupt flag
}


//-- In port
// this is nasty, UARTE defines not yet known, but cumbersome to add, so we include the lib
#ifdef DEVICE_HAS_IN
#include "../../../modules/stm32ll-lib/src/stdstm32-uarte.h"

void in_init_gpio(void)
{
}

void in_set_normal(void)
{
    LL_USART_Disable(UARTE_UARTx);
    LL_USART_SetRXPinLevel(UARTE_UARTx, LL_USART_RXPIN_LEVEL_STANDARD);
    LL_USART_Enable(UARTE_UARTx);
    gpio_init_af(UARTE_RX_IO, IO_MODE_INPUT_PU, UARTE_IO_AF, IO_SPEED_VERYFAST);
}

void in_set_inverted(void)
{
    LL_USART_Disable(UARTE_UARTx);
    LL_USART_SetRXPinLevel(UARTE_UARTx, LL_USART_RXPIN_LEVEL_INVERTED);
    LL_USART_Enable(UARTE_UARTx);
    gpio_init_af(UARTE_RX_IO, IO_MODE_INPUT_PD, UARTE_IO_AF, IO_SPEED_VERYFAST);
}
#endif


//-- Button

#define BUTTON                    IO_PB13

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON);
}


//-- LEDs
// we keep the green LED stuff in case a user wants it

#define LED_GREEN                 IO_PA15
#define LED_RED                   IO_PB5

void leds_init(void)
{
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);

    // pin IO_PB15 must be floating, is used as artificial pad for green LED!
    gpio_init(IO_PB15, IO_MODE_Z, IO_SPEED_DEFAULT);

    // artificial GND for R+Diode mod, ONLY temporary
    // this is dirty! we do it here in leds_init() to ensure it is called
    gpio_init(IO_PA0, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
}

void led_green_off(void) { gpio_low(LED_GREEN); }
void led_green_on(void) { gpio_high(LED_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

void led_red_off(void) { gpio_high(LED_RED); }
void led_red_on(void) { gpio_low(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- 5 Way Switch
// template for resistor chain Vcc - 4.7k - down - 1k - left - 2.2k - right - 4.7k - up - 4.7k - center
/*
#define FIVEWAY_ADCx              ADC
#define FIVEWAY_ADC_IO            IO_PB4 // ADC_IN3
#define FIVEWAY_ADC_CHANNELx      LL_ADC_CHANNEL_3

extern "C" { void delay_us(uint32_t us); }

void fiveway_init(void)
{
    adc_init_begin(FIVEWAY_ADCx);
    adc_init_one_channel(FIVEWAY_ADCx);
    adc_config_channel(FIVEWAY_ADCx, LL_ADC_REG_RANK_1, FIVEWAY_ADC_CHANNELx, FIVEWAY_ADC_IO);
    adc_enable(FIVEWAY_ADCx);
    delay_us(100);
    adc_start_conversion(FIVEWAY_ADCx);
}

uint16_t fiveway_adc_read(void)
{
    return LL_ADC_REG_ReadConversionData12(FIVEWAY_ADCx);
}

uint8_t fiveway_read(void)
{
    uint16_t adc = LL_ADC_REG_ReadConversionData12(FIVEWAY_ADCx);
    if (adc < (0+200)) return (1 << KEY_DOWN); // 0
    if (adc > (655-200) && adc < (655+200)) return (1 << KEY_LEFT); // 655
    if (adc > (1595-200) && adc < (1595+200)) return (1 << KEY_RIGHT); // 1595
    if (adc > (2505-200) && adc < (2505+200)) return (1 << KEY_UP); // 2505
    if (adc > (3054-200) && adc < (3054+200)) return (1 << KEY_CENTER); // 3054
    return 0;
} */


//-- Buzzer
// Buzzer is active high // TODO: needs pin and AF check! do not use

#define BUZZER                    IO_PB9
#define BUZZER_IO_AF              IO_AF_12
#define BUZZER_TIMx               TIM1
#define BUZZER_IRQn               TIM1_UP_IRQn
#define BUZZER_IRQHandler         TIM1_UP_IRQHandler
#define BUZZER_TIM_CHANNEL        LL_TIM_CHANNEL_CH3N
//#define BUZZER_TIM_IRQ_PRIORITY   14


//-- ESP32 Wifi Bridge
#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL

#define ESP_RESET                 IO_PA9
#define ESP_GPIO0                 IO_PB10

void esp_init(void)
{
    gpio_init(ESP_GPIO0, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT); // low = esp will start in bootloader mode
//    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT); // low = esp is in reset
    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT); // low = esp is in reset
}

void esp_reset_high(void) { gpio_high(ESP_RESET); }
void esp_reset_low(void) { gpio_low(ESP_RESET); }

void esp_gpio0_high(void) { gpio_high(ESP_GPIO0); }
void esp_gpio0_low(void) { gpio_low(ESP_GPIO0); }
#endif


//-- POWER

#define POWER_GAIN_DBM            0 // gain of a PA stage if present
#define POWER_SX126X_MAX_DBM      SX126X_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           2 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_22_DBM, .mW = 158 },
};


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
};

uint32_t portb[] = {
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_13,
};

uint32_t portc[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1,
};









