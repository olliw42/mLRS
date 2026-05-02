//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************

/*
  Flashing ESP32 wireless bridge:
  ???
*/

//-------------------------------------------------------
// TX FRSKY LR2021 STM32G474RE
//-------------------------------------------------------

//#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_I2C_DISPLAY_ROT180
//#define DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
//#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE
#define DEVICE_HAS_COM_ON_USB
#define DEVICE_HAS_NO_DEBUG
//#define DEVICE_HAS_NO_SERIAL


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             120 // 512 kB flash, 2 kB page, G474 has dual bank flash, hence only 256 kB available

#define MICROS_TIMx               TIM3
#define MICROS_TIM_NAMEPREFIX     TIM3_


//-- UARTS
// UARTB = serial port
// USB-C = COM (CLI)
// UARTD = serial2 BT/ESP port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_UART2_PA2PA3 // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_UART4_PC10PC11 // JR pin5, MBridge
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_FULL_INTERNAL_ON_TX

#define UARTD_USE_UART3_PB10PB11 // serial2
#define UARTD_BAUD                115200
#define UARTD_USE_TX
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTD_USE_TX_ISR
#define UARTD_USE_RX
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTF_USE_UART1_PA9PA10 // debug
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           512
#define UARTF_USE_TX_ISR


//-- SX1262 & SPI

#define SPI_USE_SPI1              // PA5, PA6, PA7
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PC7
#define SX_DIO1                   IO_PC4
#define SX_BUSY                   IO_PC12
#define SX_C0                     IO_PC15
#define SX_C1                     IO_PC14

#define SX_DIO1_SYSCFG_EXTI_PORTx    LL_SYSCFG_EXTI_PORTC
#define SX_DIO1_SYSCFG_EXTI_LINEx    LL_SYSCFG_EXTI_LINE4
#define SX_DIO_EXTI_LINE_x           LL_EXTI_LINE_4
#define SX_DIO_EXTI_IRQn             EXTI4_IRQn
#define SX_DIO_EXTI_IRQHandler       EXTI4_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

#define SX_USE_IRQ_DIO_NO         LR20XX_DIO_9

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
//??    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
//??    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

void sx_amp_transmit(void)
{
}

void sx_amp_receive(void)
{
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


//-- LR2021 & SPIB



//-- Button

#define BUTTON                    IO_PB3

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activehigh(BUTTON);
}


//-- LEDs

#define LED_GREEN                 IO_PA15
#define LED_RED                   IO_PC9


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
#if 0
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
#endif

//-- 5 Way Switch
//resistor chain Vcc - 4.7k - left - 1k - up - 2.2k - right - 4.7k - down - 15k - GND

#define FIVEWAY_ADCx              ADC1
#define FIVEWAY_ADC_IO            IO_PC2 // ADC12_IN8
#define FIVEWAY_ADC_CHANNELx      LL_ADC_CHANNEL_8

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

#define I2C_USE_I2C1              // PB6, PB7
#define I2C_CLOCKSPEED_400KHZ
#define I2C_USE_DMAMODE


//-- ESP32 Wifi Bridge

#define ESP_RESET                 IO_PB1
#define ESP_GPIO0                 IO_PB2
#define ESP_DTR_RTS_USB

#if defined DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL || defined DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
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

#define POWER_GAIN_DBM            0 // gain of a PA stage if present
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_0_DBM, .mW = 1 },
//    { .dbm = POWER_10_DBM, .mW = 10 },
//    { .dbm = POWER_14_DBM, .mW = 25 },
//    { .dbm = POWER_20_DBM, .mW = 100 },
//    { .dbm = POWER_22_DBM, .mW = 158 },
};


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

