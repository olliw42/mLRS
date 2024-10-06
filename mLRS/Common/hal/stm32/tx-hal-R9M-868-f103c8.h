//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// R9M TX Module STM32F103C8
//-------------------------------------------------------
// F9RM Pro Lite https://github.com/ExpressLRS/ExpressLRS/issues/326
// https://github.com/ExpressLRS/ExpressLRS/blob/master/src/include/target/Frsky_TX_R9M.h
// many THX to the ExpressLRS project !
// note: these sources are not always correct in features ELRS doesn't use and not complete for what we need

#define DEVICE_HAS_IN_INVERTED
#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_SERIAL_OR_COM // is selected in device specific ways, here: dip switch
#define DEVICE_HAS_DEBUG_SWUART
#define DEVICE_HAS_BUZZER
#define DEVICE_HAS_FAN_ONOFF


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             124 // 128 kB flash, 1 kB page

#define MICROS_TIMx               TIM3


//-- UARTS
// UARTB = serial port or COM (CLI)
// UARTC = --
// UART = JR bay pin5 (SPORT)
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_UART1_PA9PA10 // serial or COM (CLI) // goes via inverter to RX/TX of RS232 port
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_COM_TXBUFSIZE_SMALL // TX_SERIAL_TXBUFSIZE // choose the bigger one
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_UART3_PB10PB11 // JR pin5, MBridge
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_TX_OE              IO_PA5 // high = enabled
#define JRPIN5_TX_OE_DISABLED     gpio_low(JRPIN5_TX_OE)
#define JRPIN5_TX_OE_ENABLED      gpio_high(JRPIN5_TX_OE)

#define UARTE_USE_UART2_PA2PA3 // in port // PA3
#define UARTE_BAUD                100000 // SBus normal baud rate, is being set later anyhow
//#define UARTE_USE_TX
//#define UARTE_TXBUFSIZE           512
//#define UARTE_USE_TX_ISR
#define UARTE_USE_RX
#define UARTE_RXBUFSIZE           512

#define SWUART_USE_TIM4 // debug // goes via an inverter to JR Pin2, solder to R15 for TTL UART signal, C23 provides GND
#define SWUART_TX_IO              IO_PA2
#define SWUART_BAUD               115200
#define SWUART_USE_TX
#define SWUART_TXBUFSIZE          512
//#define SWUART_TIM_IRQ_PRIORITY   9


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI2              // PB13, PB14, PB15
#define SPI_CS_IO                 IO_PB12
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PC14
#define SX_DIO0                   IO_PA15
#define SX_DIO1                   // I believe it doesn't have a connection to DIO1
#define SX_SWITCH_RX_EN           IO_PB3
#define SX_PA_EN                  IO_PA6

#define SX_DIO0_GPIO_AF_EXTI_PORTx    LL_GPIO_AF_EXTI_PORTA
#define SX_DIO0_GPIO_AF_EXTI_LINEx    LL_GPIO_AF_EXTI_LINE15
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_15
#define SX_DIO_EXTI_IRQn              EXTI15_10_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI15_10_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

// R9M power management:
// the Sky65111 (S111) PA has two pins, Vapc1, Vapc2, to control the gains of its two amplifier stages
// Vapc2 is connected to the DAC081C081 (X86C) DAC
// the DAC in turn is connected to I2C1, PB6(SCL), PB7(SDA), and PB5(ADR0) (PB5 floating means adr 0x0C)
// Vapc1 seems to be controllable by two gpios, PB0, PA6, which appear to go to the two ends of R38
// I would think the two gpios allow to set Vapc1 to zero or to a fixed voltage value
// ELRS handles only PA6, not PB0, so it seems that that PA6 sets Vapc1 to zero
// the Sky13330 (SKY3330) switch's CNTRL pin 7 is connected to PB3, ENABLE pin 8 is high
// PB3 low = TX, high = RX

//#define SX_PA_DAC_I2C_DEVICE_ADR  0x0C // defined below in Power section

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO0, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_SWITCH_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX_PA_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

void sx_amp_transmit(void)
{
    gpio_low(SX_SWITCH_RX_EN);
    gpio_high(SX_PA_EN);
}

void sx_amp_receive(void)
{
    gpio_low(SX_PA_EN);
    gpio_high(SX_SWITCH_RX_EN);
}

void sx_dio_init_exti_isroff(void)
{
    LL_GPIO_AF_SetEXTISource(SX_DIO0_GPIO_AF_EXTI_PORTx, SX_DIO0_GPIO_AF_EXTI_LINEx);

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


//-- In port
// is on IO_PA2, UART2 RX, inverted

void in_init_gpio(void)
{
}

void in_set_normal(void)
{
}

void in_set_inverted(void)
{
}


//-- Button

#define BUTTON                    IO_PA8 // active low

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON);
}


//-- LEDs

#define LED_GREEN                 IO_PA12
#define LED_RED                   IO_PA11

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


//-- Serial or Com Switch
// the dips are on the same gpios as the LEDs ! so we need to play dirty tricks

#define DIP1                      IO_PA12 // same a green
#define DIP2                      IO_PA11 // same as red

bool r9m_ser_or_com_serial = false;  // we use com as default

void ser_or_com_init(void)
{
    gpio_init(DIP1, IO_MODE_INPUT_PU, IO_SPEED_SLOW);
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (gpio_read_activelow(DIP1)) cnt++;
    }
    r9m_ser_or_com_serial = (cnt > 8);
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    led_green_off(); // LED_GREEN_OFF
}

bool ser_or_com_serial(void)
{
    return r9m_ser_or_com_serial;
}


//-- Buzzer
// Buzzer is active high

#define BUZZER                    IO_PB1
#define BUZZER_TIMx               TIM1
#define BUZZER_IRQn               TIM1_UP_IRQn
#define BUZZER_IRQHandler         TIM1_UP_IRQHandler
#define BUZZER_TIM_CHANNEL        LL_TIM_CHANNEL_CH3N
//#define BUZZER_TIM_IRQ_PRIORITY   14


//-- EEPROM, not used

#define EEPROM_SDA                IO_PB7
#define EEPROM_SCL                IO_PB6
#define EEPROM_ADDR               0x51
#define EEPROM_400K


//-- Cooling Fan

#define FAN_IO                    IO_PB9

void fan_init(void)
{
    gpio_init(FAN_IO, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_low(FAN_IO);
}

void fan_set_power(int8_t power_dbm)
{
    if (power_dbm >= POWER_23_DBM) {
        gpio_high(FAN_IO);
    } else {
        gpio_low(FAN_IO);
    }
}


//-- POWER

#define DEVICE_HAS_I2C_DAC
#define I2C_USE_I2C1
#define I2C_CLOCKSPEED_400KHZ
#define SX_PA_DAC_I2C_DEVICE_ADR  0x0C

void rfpower_calc(int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, tI2cBase* dac)
{
    // these are the values of ELRS
    // 10mW   10dbm   720
    // 25mW   14dbm   875
    // 50mW   17dBm   1000
    // 100mW  20dBm   1140
    // 250mW  24dBm   1390
    // 500mW  27dBm   1730
    // 1000mW 30dBm   2100
    // my estimated 1mW 0dBm 200
    // measurements with an IRC power meter suggests changes (https://www.rcgroups.com/forums/showpost.php?p=49934177&postcount=888)
    uint32_t voltage_mV; // 2500 was too high
    if (power_dbm > 28) {
        voltage_mV = 2250; // was 2100
        *actual_power_dbm = 30;
    } else if (power_dbm > 25) {
        voltage_mV = 1730;
        *actual_power_dbm = 27;
    } else if (power_dbm > 22) {
        //voltage_mV = 1390;
        //*actual_power_dbm = 23;
        voltage_mV = 1475;
        *actual_power_dbm = 24;
    } else if (power_dbm > 18) {
        voltage_mV = 1195; // was 1140
        *actual_power_dbm = 20;
    } else if (power_dbm > 15) {
        voltage_mV = 1000;
        *actual_power_dbm = 17;
    } else if (power_dbm > 12) {
        voltage_mV = 875;
        *actual_power_dbm = 14;
    } else if (power_dbm > 5) {
        voltage_mV = 720;
        *actual_power_dbm = 10;
    } else {
        voltage_mV = 100; // was 200
        *actual_power_dbm = 3; // was 0
    }

    //if (!dac->initialized) return;
    // convert voltage to 0 .. 255
    uint16_t value = (voltage_mV >= 3300) ? 255 : (voltage_mV * 255) / 3300; // don't bother with rounding
    // construct data word
    uint8_t buf[2];
    buf[0] = (value & 0x00F0) >> 4;
    buf[1] = (value & 0x000F) << 4;
    dac->put_buf_blocking(SX_PA_DAC_I2C_DEVICE_ADR, buf, 2);

    *sx_power = 0;
}

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_3_DBM, .mW = 2 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_3,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_13,
    LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_13,
};


