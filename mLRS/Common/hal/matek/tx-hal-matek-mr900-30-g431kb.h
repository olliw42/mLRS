//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

//#define MLRS_FEATURE_COM_ON_USB // this MLRS_FEATURE define can be used locally here
//#define MLRS_FEATURE_HC04_MODULE // this MLRS_FEATURE define can be used locally here
//#define MLRS_FEATURE_OLED // this MLRS_FEATURE define can be used locally here


//-------------------------------------------------------
// MATEKSYS mR900-30 STM32G431KB, as Tx module
//-------------------------------------------------------

#define DEVICE_HAS_JRPIN5
//#define DEVICE_HAS_IN
#define DEVICE_HAS_IN_ON_JRPIN5_TX
#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_FAN_ONOFF // FAN_TEMPCONTROLLED_ONOFF was tested to work not so well


#ifdef MLRS_FEATURE_HC04_MODULE
#define DEVICE_HAS_HC04_MODULE_ON_SERIAL
#endif

#ifdef MLRS_FEATURE_COM_ON_USB
#define DEVICE_HAS_COM_ON_USB
#define DEVICE_HAS_SERIAL2
#endif

#ifdef MLRS_FEATURE_OLED
#define DEVICE_HAS_COM_ON_USB
#undef DEVICE_HAS_SERIAL2
#undef DEVICE_HAS_FAN_TEMPCONTROLLED_ONOFF
#define DEVICE_HAS_FAN_ONOFF
#define DEVICE_HAS_I2C_DISPLAY_ROT180
#endif


#include "hal-matek-mr-g431kb-common.h"


//-- Timers, Timing, EEPROM, and such stuff


//-- UARTS
// UARTB = serial port
// UARTC = COM (CLI)
// UARTD = serial2 BT/ESP port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever

#define UARTB_USE_UART1_PA9PA10 // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#ifndef DEVICE_HAS_COM_ON_USB
#define UARTC_USE_LPUART1_PA2PA3 // com USB/CLI
#define UARTC_BAUD                TX_COM_BAUDRATE
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           TX_COM_TXBUFSIZE
#define UARTC_USE_TX_ISR
#define UARTC_USE_RX
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE
#endif

#define UART_USE_UART2_PB3PB4 // JR pin5, MBridge
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_FULL_INTERNAL_ON_TX // does not require an external diode

#define UARTE_USE_UART2_PB3PB4 // in pin
#define UARTE_BAUD                100000 // SBus normal baud rate, is being set later anyhow
//#define UARTE_USE_TX
//#define UARTE_TXBUFSIZE           512
//#define UARTE_USE_TX_ISR
#define UARTE_USE_RX
#define UARTE_RXBUFSIZE           512

#ifdef DEVICE_HAS_SERIAL2
#define UARTD_USE_LPUART1_PA2PA3 // serial2
#define UARTD_BAUD                115200
#define UARTD_USE_TX
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTD_USE_TX_ISR
#define UARTD_USE_RX
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE
#endif


//-- SX1: SX12xx & SPI


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
    LL_USART_SetTXRXSwap(UARTE_UARTx, LL_USART_TXRX_SWAPPED);
    LL_USART_SetRXPinLevel(UARTE_UARTx, LL_USART_RXPIN_LEVEL_STANDARD);
    LL_USART_Enable(UARTE_UARTx);
    gpio_init_af(UARTE_TX_IO, IO_MODE_INPUT_PU, UARTE_IO_AF, IO_SPEED_VERYFAST);
}

void in_set_inverted(void)
{
    LL_USART_Disable(UARTE_UARTx);
    LL_USART_SetTXRXSwap(UARTE_UARTx, LL_USART_TXRX_SWAPPED);
    LL_USART_SetRXPinLevel(UARTE_UARTx, LL_USART_RXPIN_LEVEL_INVERTED);
    LL_USART_Enable(UARTE_UARTx);
    gpio_init_af(UARTE_TX_IO, IO_MODE_INPUT_PD, UARTE_IO_AF, IO_SPEED_VERYFAST);
}
#endif


//-- Button


//-- LEDs


//-- Cooling Fan


//-- Display I2C

#define I2C_USE_I2C1              // PA13, PA14
#define I2C_CLOCKSPEED_400KHZ     // not all displays seem to work well with I2C_CLOCKSPEED_1000KHZ
#define I2C_USE_DMAMODE


//-- 5 Way Switch

#define FIVEWAY_ADCx              ADC1
#define FIVEWAY_ADC_IO            IO_PA2 // ADC1_IN3
#define FIVEWAY_ADC_CHANNELx      LL_ADC_CHANNEL_3

#define KEY_UP_THRESH             3230 // BetaFPV 1W Micro scheme
#define KEY_DOWN_THRESH           0
#define KEY_LEFT_THRESH           1890
#define KEY_RIGHT_THRESH          2623
#define KEY_CENTER_THRESH         1205

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
    if (adc > (KEY_CENTER_THRESH-250) && adc < (KEY_CENTER_THRESH+250)) return (1 << KEY_CENTER);
    if (adc > (KEY_LEFT_THRESH-250) && adc < (KEY_LEFT_THRESH+250)) return (1 << KEY_LEFT);
    if (adc > (KEY_DOWN_THRESH-250) && adc < (KEY_DOWN_THRESH+250)) return (1 << KEY_DOWN);
    if (adc > (KEY_UP_THRESH-250) && adc < (KEY_UP_THRESH+250)) return (1 << KEY_UP);
    if (adc > (KEY_RIGHT_THRESH-250) && adc < (KEY_RIGHT_THRESH+250)) return (1 << KEY_RIGHT);
    return 0;
}
#endif


//-- POWER
// SX126X power setting can vary from -9 .. 22 for -9 dBm ... 22 dBm
#include "../../setup_types.h"

void sx126x_rfpower_calc(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    if (power_dbm >= POWER_30_DBM) {
        *sx_power = (frequency_band == SETUP_FREQUENCY_BAND_868_MHZ) ? 10 : 10;
        *actual_power_dbm = 30;
    } else
    if (power_dbm >= POWER_27_DBM) {
        *sx_power = (frequency_band == SETUP_FREQUENCY_BAND_868_MHZ) ? -7 : -4;
        *actual_power_dbm = 27;
    } else
    if (power_dbm >= POWER_24_DBM) {
        *sx_power = (frequency_band == SETUP_FREQUENCY_BAND_868_MHZ) ? -9 : -7;
        *actual_power_dbm = 24;
    } else
    if (power_dbm >= POWER_20_DBM) {
        *sx_power = -9;
        *actual_power_dbm = (frequency_band == SETUP_FREQUENCY_BAND_868_MHZ) ? 24 : 22;
    } else {
        *sx_power = -9;
        *actual_power_dbm = (frequency_band == SETUP_FREQUENCY_BAND_868_MHZ) ? 24 : 22;
    }
}

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN }, // doesn't make sense IMHO
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};

