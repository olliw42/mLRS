//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// TX MATEK MTX-DB30 STM32G474CE
//-------------------------------------------------------
// UART1: available on "left" JST-GH plug
//        also connected to ESP32-PICO when jumpers set
// UART4: available on "right" JST-GH plug

#define DEVICE_HAS_JRPIN5
//xx#define DEVICE_HAS_IN_ON_JRPIN5_TX
#define DEVICE_HAS_I2C_DISPLAY_ROT180
//#define DEVICE_HAS_SERIAL2
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL
#define DEVICE_HAS_COM_ON_USB // com USB/CLI
//#define DEVICE_HAS_NO_DEBUG


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             120 // 512 kB flash, 2 kB page, G474 has dual bank flash, hence only 256 kB available

#define MICROS_TIMx               TIM3


//-- UARTS
// UARTB = serial port BT/ESP port
// USB-C = COM (CLI)
// UARTD = serial2
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_UART1_PA9PA10 // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           8192 //xx TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           8192 //xx TX_SERIAL_RXBUFSIZE

#define UART_USE_UART2_PB3PB4 // JR pin5, MBridge
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_FULL_INTERNAL_ON_TX

#define UARTD_USE_UART4_PC10PC11 // serial2
#define UARTD_BAUD                115200
#define UARTD_USE_TX
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTD_USE_TX_ISR
#define UARTD_USE_RX
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTF_USE_UART4_PC10PC11 // debug
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           512
#define UARTF_USE_TX_ISR


//-- SX1262 & SPI

#define SPI_USE_SPI2             // PB13, PB14, PB15
#define SPI_CS_IO                IO_PB12
#define SPI_USE_CLK_LOW_1EDGE    // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                 IO_PB2
#define SX_DIO1                  IO_PB11
#define SX_BUSY                  IO_PB10
#define SX_RX_EN                 IO_PB0
#define SX_TX_EN                 IO_PB1

#define SX_DIO1_SYSCFG_EXTI_PORTx    LL_SYSCFG_EXTI_PORTB
#define SX_DIO1_SYSCFG_EXTI_LINEx    LL_SYSCFG_EXTI_LINE11
#define SX_DIO_EXTI_LINE_x           LL_EXTI_LINE_11
#define SX_DIO_EXTI_IRQn             EXTI15_10_IRQn
#define SX_DIO_EXTI_IRQHandler       EXTI15_10_IRQHandler
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


//-- SX1281 & SPIB

#define SPIB_USE_SPI1              // PA5, PA6, PA7
#define SPIB_CS_IO                 IO_PC4
#define SPIB_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPIB_USE_CLOCKSPEED_9MHZ

#define SX2_RESET                  IO_PA3
#define SX2_DIO1                   IO_PA1
#define SX2_BUSY                   IO_PA2
#define SX2_RX_EN                  IO_PC14
#define SX2_TX_EN                  IO_PC15

#define SX2_DIO1_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTA
#define SX2_DIO1_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE1
#define SX2_DIO_EXTI_LINE_x            LL_EXTI_LINE_1
#define SX2_DIO_EXTI_IRQn              EXTI1_IRQn
#define SX2_DIO_EXTI_IRQHandler        EXTI1_IRQHandler
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

#define BUTTON                    IO_PB8

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activehigh(BUTTON);
}


//-- LEDs

#define LED_GREEN                 IO_PA8
#define LED_RED                   IO_PC6


void leds_init(void)
{
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
}

void led_green_off(void) { gpio_low(LED_GREEN); }
void led_green_on(void) { gpio_high(LED_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- Cooling Fan

#define DEVICE_HAS_FAN_ONOFF

#define FAN_IO                    IO_PB7

void fan_init(void)
{
    gpio_init(FAN_IO, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT); // high = on
    gpio_low(FAN_IO);
}

void fan_set_power(int8_t power_dbm)
{
    if (power_dbm >= POWER_24_DBM) {
        gpio_high(FAN_IO);
    } else {
        gpio_low(FAN_IO);
    }
}


//-- 5 Way Switch
//resistor chain Vcc - 4.7k - left -4.7k - up - 4.7k - right - 4.7k - down - 4.7k - GND

#define FIVEWAY_ADCx              ADC1
#define FIVEWAY_ADC_IO            IO_PA0 // ADC12_IN1
#define FIVEWAY_ADC_CHANNELx      LL_ADC_CHANNEL_1

#define KEY_LEFT_THRESH           3300
#define KEY_UP_THRESH             2550
#define KEY_RIGHT_THRESH          1770
#define KEY_DOWN_THRESH           940
#define KEY_CENTER_THRESH         0

#ifdef DEVICE_HAS_I2C_DISPLAY_ROT180
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

uint8_t fiveway_read(void)
{
    int16_t adc = LL_ADC_REG_ReadConversionData12(FIVEWAY_ADCx);
    if (adc > (KEY_CENTER_THRESH-150) && adc < (KEY_CENTER_THRESH+150)) return (1 << KEY_CENTER);
    if (adc > (KEY_LEFT_THRESH-150) && adc < (KEY_LEFT_THRESH+150)) return (1 << KEY_LEFT);
    if (adc > (KEY_DOWN_THRESH-150) && adc < (KEY_DOWN_THRESH+150)) return (1 << KEY_DOWN);
    if (adc > (KEY_UP_THRESH-150) && adc < (KEY_UP_THRESH+150)) return (1 << KEY_UP);
    if (adc > (KEY_RIGHT_THRESH-150) && adc < (KEY_RIGHT_THRESH+150)) return (1 << KEY_RIGHT);
    return 0;
}
#endif


//-- Display I2C

#define I2C_USE_I2C1              // PA13, PA14
#define I2C_CLOCKSPEED_400KHZ
#define I2C_USE_DMAMODE


//-- ESP32 Wifi Bridge

#define ESP_RESET                 IO_PA15
#define ESP_GPIO0                 IO_PB9
#define ESP_DTR_RTS_USB

#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL
void esp_init(void)
{
    gpio_init(ESP_GPIO0, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT); // low -> esp will start in bootloader mode
    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT); // low -> esp is in reset
}

void esp_reset_high(void) { gpio_high(ESP_RESET); }
void esp_reset_low(void) { gpio_low(ESP_RESET); }

void esp_gpio0_high(void) { gpio_high(ESP_GPIO0); }
void esp_gpio0_low(void) { gpio_low(ESP_GPIO0); }
#endif


//-- POWER

#define POWER_PA_MATEK_MTX_DB30
#define POWER2_PA_MATEK_MTX_DB30
#include "../hal-power-pa.h"


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12,
};

uint32_t portb[] = {
    LL_GPIO_PIN_3, LL_GPIO_PIN_4,
};

uint32_t portc[] = {
};

