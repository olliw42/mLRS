//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// TX DIY DUAL-E22 MODULE02 v042 STM32G491RE
//-------------------------------------------------------

//#define DEVICE_HAS_DIVERSITY
#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_I2C_DISPLAY_ROT180
#define DEVICE_HAS_BUZZER
#define DEVICE_HAS_SERIAL2
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             250 // 512 kB flash, 2 kB page

#define MICROS_TIMx               TIM3


//-- UARTS
// UARTB = serial port
// UARTC = COM (CLI)
// UARTD = serial2 BT/ESP port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_UART4_PC10PC11 // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTC_USE_UART1_PA9PA10 // com USB/CLI
#define UARTC_BAUD                TX_COM_BAUDRATE
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           TX_COM_TXBUFSIZE_LARGE // TX_COM_TXBUFSIZE
#define UARTC_USE_TX_ISR
#define UARTC_USE_RX
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE

#define UART_USE_UART2_PB3PB4 // JR pin5, MBridge
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_RX_TX_INVERT_INTERNAL

#define UARTD_USE_UART3_PB10PB11 // serial2 BT/ESP
#define UARTD_BAUD                115200
#define UARTD_USE_TX
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTD_USE_TX_ISR
#define UARTD_USE_RX
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define ESP_RESET                 IO_PC8
#define ESP_GPIO0                 IO_PB1

#define UARTF_USE_LPUART1_PC1PC0 // debug
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           512
#define UARTF_USE_TX_ISR
//#define UARTF_USE_RX
//#define UARTF_RXBUFSIZE           512


//-- SX12xx & SPI

#define SPI_USE_SPI1              // PA5, PA6, PA7
#define SPI_CS_IO                 IO_PA3
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PA4
#define SX_DIO1                   IO_PA1
#define SX_BUSY                   IO_PA2
#define SX_RX_EN                  IO_PC4
#define SX_TX_EN                  IO_PC5

#define SX_DIO1_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTA
#define SX_DIO1_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE1
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_1
#define SX_DIO_EXTI_IRQn              EXTI1_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI1_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
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

#define SPIB_USE_SPI2             // PB13, PB14, PB15
#define SPIB_CS_IO                IO_PC7
#define SPIB_USE_CLK_LOW_1EDGE    // datasheet says CPHA = 0  CPOL = 0
#define SPIB_USE_CLOCKSPEED_9MHZ

#define SX2_RESET                 IO_PC6
#define SX2_DIO1                  IO_PB0
#define SX2_BUSY                  IO_PB12
#define SX2_RX_EN                 IO_PA11
#define SX2_TX_EN                 IO_PA12

#define SX2_DIO1_SYSCFG_EXTI_PORTx    LL_SYSCFG_EXTI_PORTB
#define SX2_DIO1_SYSCFG_EXTI_LINEx    LL_SYSCFG_EXTI_LINE0
#define SX2_DIO_EXTI_LINE_x           LL_EXTI_LINE_0
#define SX2_DIO_EXTI_IRQn             EXTI0_IRQn
#define SX2_DIO_EXTI_IRQHandler       EXTI0_IRQHandler
//#define SX2_DIO_EXTI_IRQ_PRIORITY   11

void sx2_init_gpio(void)
{
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX2_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX2_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init(SX2_TX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX2_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx2_busy_read(void)
{
    return (gpio_read_activehigh(SX2_BUSY)) ? true : false;
}

void sx2_amp_transmit(void)
{
    gpio_low(SX2_RX_EN);
    gpio_high(SX2_TX_EN);
}

void sx2_amp_receive(void)
{
    gpio_low(SX2_TX_EN);
    gpio_high(SX2_RX_EN);
}

void sx2_dio_init_exti_isroff(void)
{
    LL_SYSCFG_SetEXTISource(SX2_DIO1_SYSCFG_EXTI_PORTx, SX2_DIO1_SYSCFG_EXTI_LINEx);

    // let's not use LL_EXTI_Init(), but let's do it by hand, is easier to allow enabling isr later
    LL_EXTI_DisableEvent_0_31(SX2_DIO_EXTI_LINE_x);
    LL_EXTI_DisableIT_0_31(SX2_DIO_EXTI_LINE_x);
    LL_EXTI_DisableFallingTrig_0_31(SX2_DIO_EXTI_LINE_x);
    LL_EXTI_EnableRisingTrig_0_31(SX2_DIO_EXTI_LINE_x);

    NVIC_SetPriority(SX2_DIO_EXTI_IRQn, SX2_DIO_EXTI_IRQ_PRIORITY);
    NVIC_EnableIRQ(SX2_DIO_EXTI_IRQn);
}

void sx2_dio_enable_exti_isr(void)
{
    LL_EXTI_ClearFlag_0_31(SX2_DIO_EXTI_LINE_x);
    LL_EXTI_EnableIT_0_31(SX2_DIO_EXTI_LINE_x);
}

void sx2_dio_exti_isr_clearflag(void)
{
    LL_EXTI_ClearFlag_0_31(SX2_DIO_EXTI_LINE_x);
}


//-- Button

#define BUTTON                    IO_PB7

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON);
}


//-- LEDs

#define LED_GREEN                 IO_PA15
#define LED_RED                   IO_PC12
#define LED_RIGHT_GREEN           IO_PA0

void leds_init(void)
{
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_init(LED_RIGHT_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
}

void led_green_off(void) { gpio_low(LED_GREEN); }
void led_green_on(void) { gpio_high(LED_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- 5 Way Switch
// PC2: resistor chain Vcc - 4.7k - down - 1k - left - 2.2k - right - 4.7k - up
// PC13: center

#define FIVEWAY_SWITCH_CENTER     IO_PC13
#define FIVEWAY_ADCx              ADC2 // could also be ADC1
#define FIVEWAY_ADC_IO            IO_PC2 // ADC12_IN8
#define FIVEWAY_ADC_CHANNELx      LL_ADC_CHANNEL_8

#ifdef DEVICE_HAS_I2C_DISPLAY_ROT180
extern "C" { void delay_us(uint32_t us); }

void fiveway_init(void)
{
    gpio_init(FIVEWAY_SWITCH_CENTER, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
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
    uint8_t center_pressed = gpio_read_activelow(FIVEWAY_SWITCH_CENTER);
    uint16_t adc = LL_ADC_REG_ReadConversionData12(FIVEWAY_ADCx);
    if (adc < (0+200)) return (1 << KEY_DOWN); // 0
    if (adc > (655-200) && adc < (655+200)) return (1 << KEY_LEFT); // 655
    if (adc > (1595-200) && adc < (1595+200)) return (1 << KEY_RIGHT); // 1595
    if (adc > (2505-200) && adc < (2505+200)) return (1 << KEY_UP); // 2505
    return (center_pressed << KEY_CENTER);
}
#endif


//-- Display I2C

#define I2C_USE_I2C3              // PA8, PC9
#define I2C_CLOCKSPEED_400KHZ     // not all displays seem to work well with I2C_CLOCKSPEED_1000KHZ
#define I2C_USE_DMAMODE


//-- Buzzer
// Buzzer is active high

#define BUZZER                    IO_PB9
#define BUZZER_IO_AF              IO_AF_12
#define BUZZER_TIMx               TIM1
#define BUZZER_IRQn               TIM1_UP_TIM16_IRQn
#define BUZZER_IRQHandler         TIM1_UP_TIM16_IRQHandler
#define BUZZER_TIM_CHANNEL        LL_TIM_CHANNEL_CH3N
//#define BUZZER_TIM_IRQ_PRIORITY   14


//-- ESP32 Wifi Bridge

#define ESP_RESET                 IO_PC8
#define ESP_GPIO0                 IO_PB1
#define ESP_DTR                   IO_PC14 // DTR from USB-TTL adapter -> GPIO
#define ESP_RTS                   IO_PC3  // RTS from USB-TTL adapter -> RESET

#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
void esp_init(void)
{
    gpio_init(ESP_GPIO0, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT); // low -> esp will start in bootloader mode
    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT); // low -> esp is in reset
    gpio_init(ESP_DTR, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT); // is normally high
    gpio_init(ESP_RTS, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT); // is normally high
}

void esp_reset_high(void) { gpio_high(ESP_RESET); }
void esp_reset_low(void) { gpio_low(ESP_RESET); }

void esp_gpio0_high(void) { gpio_high(ESP_GPIO0); }
void esp_gpio0_low(void) { gpio_low(ESP_GPIO0); }

uint8_t esp_dtr_rts(void)
{
    return gpio_read_activehigh(ESP_DTR) + (gpio_read_activehigh(ESP_RTS) << 1);
}
#endif


//-- POWER

#define POWER_PA_NONE_SX126X
#include "../hal-power-pa.h"


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12,
    LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_7,
    LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11,
    LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14,
};









