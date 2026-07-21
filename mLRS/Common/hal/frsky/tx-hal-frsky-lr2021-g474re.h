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
  - ser dest: change to serial2 (wireless bridge is on serial)
  - ser baudrate: does not matter
  - connect board to USB (no need for manual intervention, Tx module is put into FLASH_ESP mode via esptool's DTR&RTS method)
  - Board: ESP32 PICO-D4
  - Upload Speed: can be 921600
  - Reset Method: no dtr (aka ck) (the normal method)
*/

//-------------------------------------------------------
// TX FRSKY LR2021 STM32G474RE V1.2
//-------------------------------------------------------
// com:     USB
// serial:  UART2, PA2 PA3,   JST-GH connector, Vcc,Gnd,Tx,Rx
// wbridge: UART3, PB10 PB11, wired to ESP32
// dbg:     UART1, PA9 PA10,  wired to JR pin header, 4 = Rx, 6 = Tx, 7 = Gnd
// jrpin5:  UART4, PC10,      wired to JR pin header, 5 = SPort

// BUTTONs  PB3: left, PD2: right
// LEDs     PC8: left, right, bottom two are WS2812-type LEDs  (TIM8, CH3, DMA2)
// OLed:    PA15, PB7: I2C1, PC11: OLED Reset
// Fiveway: PC2: ADC12 IN8
// FAN:     PA8: PWM 0 = off, 1 = full, PA8: Enable, PB0: FOO
// NTC:     PC0: ADC12 IN6
// CAN:     FDCAN2, PB5,PB6, PC14: CAN_STB
// ESP32:   RESET: PB1, GPIO: PB2, U0: PB10,PB11
// VT (main voltage detect):  PC1

#define DEVICE_HAS_JRPIN5
#ifndef DEVICE_HAS_DUAL_LR20xx_LR20xx
  #define DEVICE_HAS_DIVERSITY
#endif
#define DEVICE_HAS_COM_ON_USB
//#define DEVICE_HAS_NO_DEBUG
//#define DEVICE_HAS_NO_SERIAL
#define DEVICE_HAS_I2C_DISPLAY_ROT180
#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_FAN_TEMPCONTROLLED_PWM
#define DEVICE_HAS_ESP_WIFI_BRIDGE
#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE


#undef SETUP_TX_SERIAL_PORT2
#define SETUP_TX_SERIAL_PORT2  2


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             120 // 512 kB flash, 2 kB page, G474 has dual bank flash, hence only 256 kB available

#define MICROS_TIMx               TIM3
#define MICROS_TIM_NAMEPREFIX     TIM3_


//-- UARTS
// UARTB = serial port
// UARTC (or USB) = com (CLI) port
// UARTD = serial2 port or wireless bridge port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF or SWUART = debug port


//XX #define UARTB_USE_UART2_PA2PA3 // serial
    #define UARTB_USE_UART1_PA9PA10 // misuse debug as serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTD_USE_UART3_PB10PB11 // serial2 or wireless bridge
#define UARTD_BAUD                115200
#define UARTD_USE_TX
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTD_USE_TX_ISR
#define UARTD_USE_RX
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_UART4_PC10PC11 // JR pin5, MBridge
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_FULL_INTERNAL_ON_TX

//XX #define UARTF_USE_UART1_PA9PA10 // debug
     #define UARTF_USE_UART2_PA2PA3 // misuse serial as debug
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           512
#define UARTF_USE_TX_ISR


//-- SX1262 & SPI - left RF chain, as seen from top
// 2.4 GHz chain
//    PAEN1     DIO11   HIGH → PA ON (transmit mode), LOW → PA OFF (receive or idle)
//    VDET1     PA0     Analog output from the internal RF power detector
//    RF1_C0    DIO5    C1,C0 =  0 0 Shutdown/bypass, 0 1 Receive (LNA active), 1 0 Transmit (PA path), 1 1 Test/bypass/alt mode
//    RF1_C1    DIO6
// 900 MHz chain
//    900_CSD1  DIO10
//    900_CPS1  DIO8
//    900_CTX1  DIO7
// switch
//    VC1       PB9     VC = HIGH selects RF1 (RFC ↔ RF1 ON, RF2 OFF) for KCT2827L

#define SPI_USE_SPI1              // PA5, PA6, PA7
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PC7
#define SX_DIO                    IO_PC4
#define SX_BUSY                   IO_PA1
#define SX_VC                     IO_PB9

#define SX_DIO_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTC
#define SX_DIO_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE4
#define SX_DIO_EXTI_LINE_x           LL_EXTI_LINE_4
#define SX_DIO_EXTI_IRQn             EXTI4_IRQn
#define SX_DIO_EXTI_IRQHandler       EXTI4_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

#define SX_USE_IRQ_DIO_NO         LR20XX_DIO_9
#define SX_USE_TCXO_VOLTAGE       LR20XX_TCXO_SUPPLY_VOLTAGE_3_3

/*            low band    high band
              Tx  Rx      Tx  Rx
VC    PB9     1   1       0   0
CPS   DIO8    x   1       0   0
CSD   DIO10   1   1       0   0
CTX   DIO7    1   0       0   0
PAEN  DIO11   0   0       1   0
C0    DIO5    0   0       0   1
C1    DIO6    0   0       1   0 */

#define SX_USE_RFSW_DIO_CONFIG  {{LR20XX_DIO_5, LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF}, \
                                 {LR20XX_DIO_6, LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF}, \
                                 {LR20XX_DIO_7, LR20XX_DIO_RF_SWITCH_CONFIG_TX_LF}, \
                                 {LR20XX_DIO_8, LR20XX_DIO_RF_SWITCH_CONFIG_RX_LF}, \
                                 {LR20XX_DIO_10, LR20XX_DIO_RF_SWITCH_CONFIG_TX_LF | LR20XX_DIO_RF_SWITCH_CONFIG_RX_LF}, \
                                 {LR20XX_DIO_11, LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF }}

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init(SX_VC, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

void sx_amp_transmit(void)
{
    // done via DIO
}

void sx_amp_receive(void)
{
    // done via DIO
}

void sx_band(bool high_band)
{
    if (high_band) gpio_low(SX_VC); else gpio_high(SX_VC);
}

void sx_dio_init_exti_isroff(void)
{
    LL_SYSCFG_SetEXTISource(SX_DIO_SYSCFG_EXTI_PORTx, SX_DIO_SYSCFG_EXTI_LINEx);

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


//-- LR2021 & SPIB - right RF chain, as seen from top
// 2.4 GHz chain
//    PAEN2     DIO11   HIGH → PA ON (transmit mode), LOW → PA OFF (receive or idle)
//    VDET2     PC3     Analog output from the internal RF power detector
//    RF1_C0    DIO5    C1,C0 =  0 0 Shutdown/bypass, 0 1 Receive (LNA active), 1 0 Transmit (PA path), 1 1 Test/bypass/alt mode
//    RF1_C1    DIO6
// 900 MHz chain
//    900_CSD1  DIO10
//    900_CPS1  DIO8
//    900_CTX1  DIO7
// switch
//    VC2       PB4     VC = HIGH selects RF1 (RFC ↔ RF1 ON, RF2 OFF) for KCT2827L

#define SPIB_USE_SPI2             // PB13, PB14, PB15
#define SPIB_CS_IO                IO_PB12
#define SPIB_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPIB_USE_CLOCKSPEED_9MHZ

#define SX2_RESET                 IO_PC6
#define SX2_DIO                   IO_PC5
#define SX2_BUSY                  IO_PC12
#define SX2_VC                    IO_PB4

#define SX2_DIO_SYSCFG_EXTI_PORTx    LL_SYSCFG_EXTI_PORTC
#define SX2_DIO_SYSCFG_EXTI_LINEx    LL_SYSCFG_EXTI_LINE5
#define SX2_DIO_EXTI_LINE_x          LL_EXTI_LINE_5
#define SX2_DIO_EXTI_IRQn            EXTI9_5_IRQn
#define SX2_DIO_EXTI_IRQHandler      EXTI9_5_IRQHandler
//#define SX2_DIO_EXTI_IRQ_PRIORITY    11

#define SX2_USE_IRQ_DIO_NO        LR20XX_DIO_9
#define SX2_USE_TCXO_VOLTAGE      LR20XX_TCXO_SUPPLY_VOLTAGE_3_3
#define SX2_USE_RFSW_DIO_CONFIG   SX_USE_RFSW_DIO_CONFIG // use same as for SX

void sx2_init_gpio(void)
{
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX2_DIO, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX2_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init(SX2_VC, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx2_busy_read(void)
{
    return (gpio_read_activehigh(SX2_BUSY)) ? true : false;
}

void sx2_amp_transmit(void)
{
    // done via DIO
}

void sx2_amp_receive(void)
{
    // done via DIO
}

void sx2_band(bool high_band)
{
    if (high_band) gpio_low(SX2_VC); else gpio_high(SX2_VC);
}

void sx2_dio_init_exti_isroff(void)
{
    LL_SYSCFG_SetEXTISource(SX2_DIO_SYSCFG_EXTI_PORTx, SX2_DIO_SYSCFG_EXTI_LINEx);

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

#define BUTTON                    IO_PB3 // left: PB3, right: PD2

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON);
}


//-- LEDs

#define WS2812_NUMBER_OF_LEDS     2
#define WS2812_IO                 IO_PC8
#define WS2812_IO_AF              IO_AF_4
#define WS2812_TIMx               TIM8
#define WS2812_TIMno              8
#define WS2812_CHno               3
#define WS2812_DMAx               DMA2
#define WS2812_DMA_CHANNEL_x      LL_DMA_CHANNEL_3
#include "../../thirdparty/stdstm32-ws2812.h"

tWs2812Color ledCurrentColor;

void leds_color_and_state(tWs2812Color color)
{
    if (color == ledCurrentColor) return;
    ledCurrentColor = color;
    ws2812_fill_all(color);
    ws2812_send();
}

void leds_init(void)
{
    ws2812_init();
    ledCurrentColor = 0;
}

void led_red_off(void) { leds_color_and_state(0); }
void led_red_on(void) { leds_color_and_state(WS2812_RED); }
void led_red_toggle(void) { (ledCurrentColor == WS2812_RED) ? led_red_off() : led_red_on(); }

void led_green_off(void) { leds_color_and_state(0); }
void led_green_on(void) { leds_color_and_state(WS2812_GREEN); }
void led_green_toggle(void) { (ledCurrentColor == WS2812_GREEN) ? led_green_off() : led_green_on(); }

void led_blue_off(void) { leds_color_and_state(0); }
void led_blue_on(void) { leds_color_and_state(WS2812_BLUE); }
void led_blue_toggle(void) { (ledCurrentColor == WS2812_BLUE) ? led_blue_off() : led_blue_on(); }

void led_purple_off(void) { leds_color_and_state(0); }
void led_purple_on(void) { leds_color_and_state(WS2812_PURPLE); }
void led_purple_toggle(void) { (ledCurrentColor == WS2812_PURPLE) ? led_purple_off() : led_purple_on(); }


//-- Cooling Fan

#define FAN_IO                    IO_PA8 // PA8 is the PWM, 0 = off, 1 = full, don't know what PB0 is doing
#define FAN_TIMx                  TIM1
#define FAN_TIM_CHANNEL_CHx       LL_TIM_CHANNEL_CH1
#include "../../thirdparty/stdstm32-tim-ext.h"

#define FAN_TSENSOR_ADCx          ADC2
#define FAN_TSENSOR_ADC_IO        IO_PC0 // ADC12_IN6
#define FAN_TSENSOR_ADC_CHANNELx  LL_ADC_CHANNEL_6

extern "C" { void delay_us(uint32_t us); }

void fan_init(void)
{
    adc_init_begin(FAN_TSENSOR_ADCx);
    adc_init_one_channel(FAN_TSENSOR_ADCx);
    adc_config_channel(FAN_TSENSOR_ADCx, LL_ADC_REG_RANK_1, FAN_TSENSOR_ADC_CHANNELx, FAN_TSENSOR_ADC_IO);
    adc_enable(FAN_TSENSOR_ADCx);
    delay_us(100);
    adc_start_conversion(FAN_TSENSOR_ADCx);

    gpio_init_af(FAN_IO, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_6, IO_SPEED_DEFAULT);
    tim_config_up(FAN_TIMx, 400, TIMER_BASE_10MHZ); // 25 kHz, 400 steps
    tim_config_oc(FAN_TIMx, FAN_TIM_CHANNEL_CHx);
    tim_oc_enable(FAN_TIMx);
    tim_enable(FAN_TIMx);
}

void fan_set_pwm(uint16_t pwm) // pwm in percent
{
    FAN_TIMx->CCR1 = 4*pwm; // LL_TIM_OC_SetCompareCH1(FAN_TIMx, pwm);
}

int16_t fan_tempsensor_read_dC(void) // 300 = 30.0 °C
{
    uint16_t adc = LL_ADC_REG_ReadConversionData12(FAN_TSENSOR_ADCx);

    // SDNT1608X104F3950: 25 °C = 100 kOhm, B constant (B25/50) = 3950 K
    // R = 100 kOhm
    // => 25 °C = 2048
    static const uint16_t ntc_adc_table[9] = {
    //  0.0,  12.5, 25.0, 37.5, 50.0, 62.5, 75.0, 87.5, 100.0 °C
        3156, 2745, 2048, 1470, 1055,  760,  560,  420, 320
    };
    if (adc >= ntc_adc_table[0]) return 0; // 0 °C
    if (adc <= ntc_adc_table[8]) return 1000; // 100 °C
    for (uint8_t i = 0; i < 8 - 1; i++) {
        uint16_t a1 = ntc_adc_table[i];
        uint16_t a2 = ntc_adc_table[i + 1];
        if (adc <= a1 && adc > a2) { // is in slot
            // t = t1 + (adc - a1)*(t2 - t1)/(a2 - a1)
            int32_t t1 = i * 125;
            int32_t num = (int32_t)(adc - a1) * 125;
            return t1 + num / (a2 - a1);
        }
    }
    return 2000; // error
}


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

#define I2C_USE_I2C1              // PA15, PB7
#define I2C_CLOCKSPEED_400KHZ
#define I2C_USE_DMAMODE

#define I2C_USE_SCL_IO            IO_PA15
#define I2C_USE_SDA_IO            IO_PB7

#define DISPLAY_INIT

extern "C" { void delay_ms(uint16_t ms); }

void display_init(void)
{
    gpio_init(IO_PC11, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
    delay_ms(10);
}


//-- ESP32 Wifi Bridge

#define ESP_RESET                 IO_PB1
#define ESP_GPIO0                 IO_PB2
#define ESP_DTR_RTS_USB

#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE
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

#define POWER_SUPPLY_DETECT_IO            IO_PC1
#define POWER_SUPPLY_LOW_POWER_LIMIT_DBM  20

#define POWER_GAIN_DBM            23 // gain of a PA stage if present
#define POWER_USE_DEFAULT_RFPOWER_CALC
/*
void lr20xx_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    // for now just mimics calc_default, is to be prepared for more sophisticated schemes

    int16_t power_sx = ((int16_t)power_dbm - POWER_GAIN_DBM) * 2; // LR20xx power is in units of 0.5 dBm
    if (frequency_band == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
        if (power_sx < LR20XX_POWER_HF_MIN) power_sx = LR20XX_POWER_HF_MIN;
        if (power_sx > LR20XX_POWER_HF_MAX) power_sx = LR20XX_POWER_HF_MAX;
    } else {
        if (power_sx < LR20XX_POWER_LF_MIN) power_sx = LR20XX_POWER_LF_MIN;
        if (power_sx > LR20XX_POWER_LF_MAX) power_sx = LR20XX_POWER_LF_MAX;
    }
    *sx_power = power_sx;
    *actual_power_dbm = power_sx / 2 + POWER_GAIN_DBM;
}
*/
#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
//    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
//    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_14,
};

