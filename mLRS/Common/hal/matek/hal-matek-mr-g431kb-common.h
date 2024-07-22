//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// MATEKSYS mR24-30, mR900-30 STM32G431KB
//-------------------------------------------------------
// common HAL

/*
pins which are available for free use, i.e., are not used for LoRa
they are all broken out on the modules

label	location		pin			functions	alternative functions
Tx1		pin header		PA9			U1_TX		I2C2_SCL / T2_CH3
Rx1		pin header		PA10		U1_RX		T1_CH3 / T2_CH4
Tx2		pin header		PB3			U2_TX		T2_CH2
Rx2		pin header		PB4			U2_RX       T3_CH1 / T16_CH1
LT1		pin header		PA2			LPU1_TX		U2_TX / T2_CH3 / T15_CH1 / ADC1_IN3
LR1		pin header		PA3 		LPU1_RX		U2_RX / T2_CH4 / T15_CH2 / ADC1_IN4

FAN		solder pad		PA8			GPIO    	I2C2_SDA, T1_CH1
D-		solder pad		PA11		USB DM		T1_CH4 / T4_CH1 / FDCAN1_RX
D+		solder pad		PA12		USB DP		T4_CH2 / T16_CH1 / FDCAN1_TX
SWD		solder pad		PA13		SWDIO		I2C1_SCL / T4_CH3
SWC		solder pad		PA14 		SWDCLK		I2C1_SDA / T8_CH2
*/

//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             60 // 128 kB flash, 2 kB page

#define MICROS_TIMx               TIM3


//-- UARTS
//--


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI1              // PA5, PA6, PA7
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PB6
#define SX_DIO1                   IO_PA15
#define SX_BUSY                   IO_PB5
#define SX_RX_EN                  IO_PB0
#define SX_TX_EN                  IO_PB7

#define SX_DIO1_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTA
#define SX_DIO1_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE15
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_15
#define SX_DIO_EXTI_IRQn              EXTI15_10_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI15_10_IRQHandler
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

#define LED_GREEN 		          IO_PA1
#define LED_RED		              IO_PA0

void leds_init(void)
{
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_low(LED_GREEN); // LED_GREEN_OFF
    gpio_low(LED_RED); // LED_RED_OFF
}

void led_green_off(void) { gpio_low(LED_GREEN); }
void led_green_on(void) { gpio_high(LED_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- Cooling Fan

#define FAN_IO                    IO_PA8

void fan_init(void)
{
    gpio_init(FAN_IO, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_low(FAN_IO);
}

void fan_set_power(int8_t power_dbm)
{
    if (power_dbm >= POWER_27_DBM) {
        gpio_high(FAN_IO);
    } else {
        gpio_low(FAN_IO);
    }
}


//-- POWER
//--


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
};

uint32_t portc[] = {
};


