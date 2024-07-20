//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************
// 5.Aug.2023: jrpin5 changed from JRPIN5_RX_TX_INVERT_INTERNAL to JRPIN5_FULL_INTERNAL
// 5.Sep.2023: jrpin5 and in simultaneously supported

//#define MLRS_DEV_FEATURE_JRPIN5_SDIODE
//#define MLRS_FEATURE_E77_XTAL // must be defined high, not here, affects main !

//-------------------------------------------------------
// TX EByte E77 MBL Kit, STM32WLE5CC
//-------------------------------------------------------
// LED1 (green)           -> PB4
// LED2 (green)           -> PB3
// button1                -> PA0
// button2                -> PA1
// USB <-> TxD,RxD pins   -> PA2,PA3 = LPUART1,UART2

//#define DEVICE_HAS_DIVERSITY // TODO, could be an add-on board/shield for the MBL Kit
#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_IN_ON_JRPIN5_RX
#define DEVICE_HAS_SERIAL_OR_COM // serial or com is selected by pressing BUTTON during power on
#define DEVICE_HAS_DEBUG_SWUART
#define DEVICE_HAS_SYSTEMBOOT


#ifdef MLRS_DEV_FEATURE_JRPIN5_SDIODE
  #define DEVICE_HAS_JRPIN5
  #undef DEVICE_HAS_IN_ON_JRPIN5_RX
#endif


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             120 // 256 kB flash, 2 kB page

#define MICROS_TIMx               TIM16


//-- UARTS
// UARTB = serial port
// UARTC = COM (CLI)
// UARTD = -- //serial2 BT/ESP port
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

#define UARTC_USE_UART2_PA2PA3 // com USB/CLI // PA2,PA3
#define UARTC_BAUD                TX_COM_BAUDRATE
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           TX_COM_TXBUFSIZE
#define UARTC_USE_TX_ISR
#define UARTC_USE_RX
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE

#define UART_USE_UART1_PB6PB7 // JR pin5, MBridge // PB6,PB7
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

/*
#define UARTE_USE_UART1_PB6PB7 // in port // PB7
#define UARTE_BAUD                100000 // SBus normal baud rate, is being set later anyhow
//#define UARTE_USE_TX
//#define UARTE_TXBUFSIZE           512
//#define UARTE_USE_TX_ISR
#define UARTE_USE_RX
#define UARTE_RXBUFSIZE           512
*/

#define SWUART_USE_TIM17 // debug
#define SWUART_TX_IO              IO_PA5
#define SWUART_BAUD               115200
#define SWUART_USE_TX
#define SWUART_TXBUFSIZE          512
//#define SWUART_TIM_IRQ_PRIORITY   9


//-- SX12xx & SPI

#define SPI_USE_SUBGHZSPI
#define SPI_USE_CLOCKSPEED_12MHZ

#define SX_BUSY                   0 // busy is provided by subghz, we need to define a dummy to fool sx126x_driver lib
#define SX_HAS_NO_RESET           // SubGHz has no reset, reset is done by spi_init()

#define SX_RX_EN                  IO_PA7
#define SX_TX_EN                  IO_PA6

#define SX_DIO_EXTI_IRQn              SUBGHZ_Radio_IRQn
#define SX_DIO_EXTI_IRQHandler        SUBGHZ_Radio_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

#ifdef MLRS_FEATURE_E77_XTAL
#define SX_USE_CRYSTALOSCILLATOR
#endif

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
{
    // there is no EXTI_LINE_44 interrupt flag
    //LL_EXTI_ClearFlag_32_63(SX_DIO_EXTI_LINE_x);
    //LL_EXTI_EnableIT_32_63(SX_DIO_EXTI_LINE_x);

    NVIC_EnableIRQ(SX_DIO_EXTI_IRQn);
}

void sx_dio_exti_isr_clearflag(void)
{
    // there is no EXTI_LINE_44 interrupt flag
}


//-- SX12xx II & SPIB

#define SPIB_USE_SPI1             // PA5, PA11, PA12
#define SPIB_USE_SCK_IO           IO_PA5
#define SPIB_USE_MISO_IO          IO_PA11
#define SPIB_USE_MOSI_IO          IO_PA12
#define SPIB_CS_IO                IO_PB12
#define SPIB_USE_CLK_LOW_1EDGE    // datasheet says CPHA = 0  CPOL = 0
#define SPIB_USE_CLOCKSPEED_9MHZ

#define SX2_RESET                 IO_PB2
#define SX2_DIO1                  IO_PC13
#define SX2_BUSY                  IO_PA15
#define SX2_RX_EN                 IO_PA0
#define SX2_TX_EN                 IO_PB8

#define SX2_DIO1_SYSCFG_EXTI_PORTx    LL_SYSCFG_EXTI_PORTC
#define SX2_DIO1_SYSCFG_EXTI_LINEx    LL_SYSCFG_EXTI_LINE13
#define SX2_DIO_EXTI_LINE_x           LL_EXTI_LINE_13
#define SX2_DIO_EXTI_IRQn             EXTI15_10_IRQn
#define SX2_DIO_EXTI_IRQHandler       EXTI15_10_IRQHandler
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

#define BUTTON                    IO_PA0

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}


bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON);
}


//-- LEDs

#define LED_GREEN                 IO_PB3
#define LED_RED                   IO_PB4

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


//-- SystemBootLoader

#define BOOT_BUTTON               IO_PA1

extern "C" { void delay_ms(uint16_t ms); }

void systembootloader_init(void)
{
    gpio_init(BOOT_BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
    // on this board, the button has a capacitor, so we need to wait for the cap to charge up
    // 1 ms is found to not be enough, 2 ms is, but play it safe
    delay_ms(10);
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (gpio_read_activelow(BOOT_BUTTON)) cnt++;
    }
    if (cnt > 12) {
        BootLoaderInit();
    }
}


//-- Serial or Com Switch
// use com if BUTTON is pressed during power up, else use serial
// BUTTON becomes bind button later on

bool e77mblkit_ser_or_com_serial = true; // we use serial as default

void ser_or_com_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (gpio_read_activelow(BUTTON)) cnt++;
    }
    e77mblkit_ser_or_com_serial = !(cnt > 8);
}

bool ser_or_com_serial(void)
{
    return e77mblkit_ser_or_com_serial;
}


//-- 5 Way Switch
// has none


//-- Buzzer
// has none


//-- ESP32 Wifi Bridge
// has none


//-- Cooling Fan
// has none


//-- POWER

#define POWER_PA_NONE_SX126X
#include "../hal-power-pa.h"


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_5,
    LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12,
    LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8,
    LL_GPIO_PIN_12,
};

uint32_t portc[] = {
    //LL_GPIO_PIN_13,
};









