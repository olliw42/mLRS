//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_FAN_ONOFF // board has a Fan, which can be set on or off
#define DEVICE_HAS_OUT

#ifdef RX_ELRS_RADIOMASTER_BANDIT_900_ESP32
    #define DEVICE_HAS_SINGLE_LED_RGB
#else
    #define DEVICE_HAS_SINGLE_LED
#endif

//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL // serial, 4-pin connector need to be unplugged for programming
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P1
#define UARTB_USE_RX_IO           IO_P3
#define UARTB_TXBUFSIZE           1024 // TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_SERIAL1 // out is on JRPin5
#define UART_BAUD                416666   // CRSF baud rate
#define UART_USE_TX_IO           IO_P13
#define UART_USE_RX_IO            -1       // no Rx pin needed
#define UART_TXBUFSIZE            256


//-- Out port

void out_init_gpio(void) {}

void out_set_normal(void)
{
    // https://github.com/espressif/esp-idf/blob/release/v4.4/components/esp_rom/include/esp32/rom/gpio.h#L228-L242
    gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, false, false);
}

void out_set_inverted(void) 
{
    gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, true, false);
}


//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P4
#define SPI_MISO                  IO_P19
#define SPI_MOSI                  IO_P23
#define SPI_SCK                   IO_P18
#define SPI_FREQUENCY             10000000L
#define SX_RESET                  IO_P5
#define SX_DIO0                   IO_P22
#define SX_TX_EN                  IO_P33

#define SX_USE_RFO

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO0, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
}

IRAM_ATTR void sx_amp_transmit(void)
{
    gpio_high(SX_TX_EN);
}

IRAM_ATTR void sx_amp_receive(void)
{
    gpio_low(SX_TX_EN);
}

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO0, SX_DIO_EXTI_IRQHandler, RISING);
}

void sx_dio_init_exti_isroff(void)
{
    detachInterrupt(SX_DIO0);
}

void sx_dio_exti_isr_clearflag(void) {}


//-- Button

#define FIVEWAY_ADC_IO            IO_P39  // 5-Way Button
#define KEY_CENTER_THRESH         1205

void button_init(void) {}

IRAM_ATTR bool button_pressed(void)
{
    int16_t adc = analogRead(FIVEWAY_ADC_IO);
    return adc > (KEY_CENTER_THRESH-250) && adc < (KEY_CENTER_THRESH+250);
}


//-- LEDs

#define LED_RED                   IO_P15

#ifdef RX_ELRS_RADIOMASTER_BANDIT_900_ESP32
    #define DEVICE_HAS_SINGLE_LED_RGB

    #include <NeoPixelBus.h>
    bool ledRedState;
    bool ledGreenState;
    bool ledBlueState;

    uint8_t pixelNum = 6;

    NeoPixelBus<NeoGrbFeature, NeoEsp32I2s0Ws2812xMethod> ledRGB(pixelNum, LED_RED);

    void leds_init(void)
    {
        ledRGB.Begin();
        ledRGB.Show();
    }

    IRAM_ATTR void led_red_off(void)
    {
        if (!ledRedState) return;
        ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
        ledRGB.SetPixelColor(1, RgbColor(0, 0, 0));
        ledRGB.Show();
        ledRedState = 0;
    }

    IRAM_ATTR void led_red_on(void)
    {
        if (ledRedState) return;
        ledRGB.SetPixelColor(0, RgbColor(255, 0, 0));
        ledRGB.SetPixelColor(1, RgbColor(255, 0, 0));
        ledRGB.Show();
        ledRedState = 1;
    }

    IRAM_ATTR void led_red_toggle(void)
    {
        if (ledRedState) { led_red_off(); } else { led_red_on(); }
    }

    IRAM_ATTR void led_green_off(void)
    {
        if (!ledGreenState) return;
        ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
        ledRGB.SetPixelColor(1, RgbColor(0, 0, 0));
        ledRGB.Show();
        ledGreenState = 0;
    }

    IRAM_ATTR void led_green_on(void)
    {
        if (ledGreenState) return;
        ledRGB.SetPixelColor(0, RgbColor(0, 255, 0));
        ledRGB.SetPixelColor(1, RgbColor(0, 255, 0));
        ledRGB.Show();
        ledGreenState = 1;
    }

    IRAM_ATTR void led_green_toggle(void)
    {
        if (ledGreenState) { led_green_off(); } else { led_green_on(); }
    }

    IRAM_ATTR void led_blue_off(void)
    {
        if (!ledBlueState) return;
        ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
        ledRGB.SetPixelColor(1, RgbColor(0, 0, 0));
        ledRGB.Show();
        ledBlueState = 0;
    }

    IRAM_ATTR void led_blue_on(void)
    {
        if (ledBlueState) return;
        ledRGB.SetPixelColor(0, RgbColor(0, 0, 255));
        ledRGB.SetPixelColor(1, RgbColor(0, 0, 255));
        ledRGB.Show();
        ledBlueState = 1;
    }

    IRAM_ATTR void led_blue_toggle(void)
    {
        if (ledBlueState) { led_blue_off(); } else { led_blue_on(); }
    }

#else

    void leds_init(void)
    {
        gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW);
    }

    IRAM_ATTR void led_red_off(void) { gpio_low(LED_RED); }
    IRAM_ATTR void led_red_on(void) { gpio_high(LED_RED); }
    IRAM_ATTR void led_red_toggle(void) { gpio_toggle(LED_RED); }

#endif  // RX_ELRS_RADIOMASTER_BANDIT_900_ESP32


//-- Cooling Fan

#define FAN_IO                    IO_P2

void fan_init(void)
{
    gpio_init(FAN_IO, IO_MODE_OUTPUT_PP_LOW);
    gpio_low(FAN_IO);
}

IRAM_ATTR void fan_set_power(int8_t power_dbm)
{
    if (power_dbm >= POWER_23_DBM) {
        gpio_high(FAN_IO);
    } else {
        gpio_low(FAN_IO);
    }
}


//-- POWER

#define POWER_GAIN_DBM            0 // 13 // gain of a PA stage if present
#define POWER_SX1276_MAX_DBM      SX1276_OUTPUT_POWER_MAX // maximum allowed sx power
//#define POWER_USE_DEFAULT_RFPOWER_CALC

void sx1276_rfpower_calc(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, const uint8_t GAIN_DBM, const uint8_t SX1276_MAX_DBM)
{
    // jr bay: 
    //   SX1276_MAX_POWER_15_DBM:   dac = 0,   sx_power = 15 => 30.5 dBm
    //                              dac = 0,   sx_power = 0  => 27.4 dBm
    //                              dac = 190, sx_power = 0  => -25 dBm !!!
    //                              dac = 180, sx_power = 0  => 18.8 dBm
    //                              dac = 150, sx_power = 0  => 24.7 dBm
    //                              dac = 100, sx_power = 0  => 27.3 dBm
    //   SX1276_MAX_POWER_10p8_DBM: dac = 0,   sx_power = 15 => 29.4 dBm
    //                              dac = 0,   sx_power = 0  => 21.0 dBm
    //                              dac = 180, sx_power = 0  => 10.7 dBm
    //   SX1276_MAX_POWER_11p4_DBM  dac = 0,   sx_power = 0  => 27.3 dBm
    //                              dac = 100, sx_power = 15 => 29.9 dBm
    //                              dac = 0,   sx_power = 15 => 30.0 dBm
    //                              dac = 180, sx_power = 0  => 10.1 dBm

    uint8_t dac = 100;

    if (power_dbm > 28) { // -> 30
        dac = 0;
        *sx_power = 15; // equals SX1276_OUTPUT_POWER_MAX
        *actual_power_dbm = 30;
    } else if (power_dbm > 25) { // -> 27
        dac = 140;
        *sx_power = 15;
        *actual_power_dbm = 27;
    } else if (power_dbm > 22) { // -> 24
        dac = 150;
        *sx_power = 6;
        *actual_power_dbm = 24;
    } else if (power_dbm > 18) { // -> 20
        dac = 150;
        *sx_power = 0;
        *actual_power_dbm = 20;
    } else {
        dac = 180;
        *sx_power = 0;
        *actual_power_dbm = 10;
    }

    dacWrite(IO_P26, dac);
    dacWrite(IO_P26, dac);
}

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};

