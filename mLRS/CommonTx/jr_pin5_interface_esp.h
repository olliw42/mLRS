//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// JR Pin5 Interface Header for ESP32
//********************************************************
#ifndef JRPIN5_INTERFACE_ESP_H
#define JRPIN5_INTERFACE_ESP_H


#include "../Common/esp-lib/esp-uart.h"
#include "../Common/protocols/crsf_protocol.h"
#include <hal/uart_ll.h>


//-------------------------------------------------------
// 100 us ISR timer to check for end of transmit in half-duplex mode
// need to know when the UART has finished transmitting, so can switch back to receive
// Arduino doesn't expose a UART transmit complete interrupt / callback like STM32
// so poll the UART state machine every 100 us using an interrupt on Core 0
// mLRS uses Core 1, so this shouldn't have any impact

volatile bool uart_is_transmitting;

IRQHANDLER(
void CLOCK100US_IRQHandler(void)
{
    if (!uart_is_transmitting) return;

    if (uart_ll_is_tx_idle(UART_LL_GET_HW(1))) {
        uart_is_transmitting = false;
        uart_ll_rxfifo_rst(UART_LL_GET_HW(1)); // discards ghost byte caused by switching
        gpio_set_direction((gpio_num_t)UART_USE_TX_IO, GPIO_MODE_INPUT);
        gpio_set_pull_mode((gpio_num_t)UART_USE_TX_IO, GPIO_PULLDOWN_ONLY);
        gpio_matrix_in((gpio_num_t)UART_USE_TX_IO, U1RXD_IN_IDX, true);
    }
})


//-------------------------------------------------------
// Pin5BridgeBase class

class tPin5BridgeBase
{
  public:
    void Init(void);

    // telemetry handling
    bool telemetry_start_next_tick;
    uint16_t telemetry_state;

    void TelemetryStart(void);

    // interface to the uart hardware peripheral used for the bridge
    void pin5_init(void);
    void pin5_putbuf(uint8_t* const buf, uint16_t len) { uart_putbuf(buf, len); }
    void pin5_getbuf(char* const buf, uint16_t len) { uart_getbuf(buf, len); }
    uint16_t pin5_bytes_available(void) { return uart_rx_bytesavailable(); }

    // only for half-duplex
    IRAM_ATTR void pin5_tx_enable(void);
    IRAM_ATTR void pin5_rx_enable(void);

    // for callback processing
    virtual void parse_nextchar(uint8_t c) = 0;
    virtual bool transmit_start(void) = 0; // returns true if transmission should be started

    // callback functions
    IRAM_ATTR void pin5_rx_callback(uint8_t c);
    void pin5_tc_callback(void) {} // is needed in derived classes

    // parser
    typedef enum {
        STATE_IDLE = 0,

        // mBridge receive states
        STATE_RECEIVE_MBRIDGE_STX2,
        STATE_RECEIVE_MBRIDGE_LEN,
        STATE_RECEIVE_MBRIDGE_SERIALPACKET,
        STATE_RECEIVE_MBRIDGE_CHANNELPACKET,
        STATE_RECEIVE_MBRIDGE_COMMANDPACKET,

        // CRSF receive states
        STATE_RECEIVE_CRSF_LEN,
        STATE_RECEIVE_CRSF_PAYLOAD,
        STATE_RECEIVE_CRSF_CRC,

        // transmit states, used by all
        STATE_TRANSMIT_START,
        STATE_TRANSMITING,
    } STATE_ENUM;

    // not used in this class, but required by the children, so just add them here
    // no need for volatile since used only in isr context
    uint8_t state;
    uint8_t len;
    uint8_t cnt;
    uint16_t tlast_us;

    // check and rescue, is here for compatibility, not needed
    void CheckAndRescue(void) {}

  private:
    tFifo<char,128> pin5_fifo; // enough for 2 full CRSF messages
    bool pin5_clock_initialized;
};


void tPin5BridgeBase::Init(void)
{
    state = STATE_IDLE;
    len = 0;
    cnt = 0;
    tlast_us = 0;

    telemetry_start_next_tick = false;
    telemetry_state = 0;
    
    pin5_fifo.Init();

    pin5_init();

    uart_is_transmitting = false;
    pin5_clock_initialized = false;
}


void tPin5BridgeBase::TelemetryStart(void)
{
    telemetry_start_next_tick = true;
}


//-------------------------------------------------------
// Interface to the uart hardware peripheral used for the bridge
// receive callback is triggered once the radio has stopped transmitting
// the buffer will contain a complete CRSF message or potentially
// the end of a message when the callback is first initialized
// use a fifo to play it safe

void tPin5BridgeBase::pin5_init(void)
{
    uart_init();

    // onReceive uses the pin5_rx_callback function
    // true means trigger only on a symbol timeout
    UART_SERIAL_NO.onReceive((void (*)(void)) uart_rx_callback_ptr, true);
    
#ifndef JR_PIN5_FULL_DUPLEX

#ifndef UART_USE_SERIAL1
  #error JRPin5 must use Serial1!
#endif
    
    pin5_rx_enable();  // configure the pin for receive  
    
    // setup timer interrupt, only needs to be done on first boot
    if (pin5_clock_initialized) return;
    
    xTaskCreatePinnedToCore([](void *parameter) {
        hw_timer_t* timer1_cfg = nullptr;
        timer1_cfg = timerBegin(1, 800, 1); // Timer 1, APB clock is 80 Mhz | divide by 800 is 100 KHz / 10 us, count up
        timerAttachInterrupt(timer1_cfg, &CLOCK100US_IRQHandler, true);
        timerAlarmWrite(timer1_cfg, 10, true); // 10 * 10 = 100 us
        timerAlarmEnable(timer1_cfg);
        vTaskDelete(NULL);
    }, "TimerSetup", 2048, NULL, 1, NULL, 0); // last argument here is Core 0, ignored on ESP32C3

    pin5_clock_initialized = true;
#endif
}


IRAM_ATTR void tPin5BridgeBase::pin5_tx_enable(void)
{
#ifndef JR_PIN5_FULL_DUPLEX
constexpr uint8_t MATRIX_DETACH_IN_LOW = 0x30; // routes 0 to matrix slot
    
    gpio_matrix_in(MATRIX_DETACH_IN_LOW, U1RXD_IN_IDX, false); // disconnect RX from all pads
    gpio_set_pull_mode((gpio_num_t)UART_USE_TX_IO, GPIO_PULLDOWN_ONLY); // disable pullup / pulldown
    gpio_set_level((gpio_num_t)UART_USE_TX_IO, 0); // set inverted level
    gpio_set_direction((gpio_num_t)UART_USE_TX_IO, GPIO_MODE_OUTPUT);
    gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, true, false);
#endif
}


IRAM_ATTR void tPin5BridgeBase::pin5_rx_enable(void)
{
#ifndef JR_PIN5_FULL_DUPLEX
    gpio_set_direction((gpio_num_t)UART_USE_TX_IO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)UART_USE_TX_IO, GPIO_PULLDOWN_ONLY); // pulldown only, since inverted
    gpio_matrix_in((gpio_num_t)UART_USE_TX_IO, U1RXD_IN_IDX, true);
#endif
}


IRAM_ATTR void tPin5BridgeBase::pin5_rx_callback(uint8_t c)
{
    // read out the buffer, put bytes in fifo
    char buf[CRSF_FRAME_LEN_MAX + 16];
    uint16_t available = pin5_bytes_available();
    available = MIN(available, CRSF_FRAME_LEN_MAX);
    
    pin5_getbuf(buf, available);
    pin5_fifo.PutBuf(buf, available);
    
    // parse for a CRSF message
    while (pin5_fifo.Available()) {
        if (state >= STATE_TRANSMIT_START) break; // read at most 1 message
        parse_nextchar(pin5_fifo.Get());
    }

    // send telemetry after every received message
    // can transmit now that entire message has been parsed
    if (state == STATE_TRANSMIT_START) {
        pin5_tx_enable();
        transmit_start();
        uart_is_transmitting = true;
    }
    
    state = STATE_IDLE;
}


//-------------------------------------------------------
// Pin5 Serial class
// used for ESP passthrough flashing
// out-commented functions are unused and thus not overridden

class tJrPin5SerialPort : public tSerialBase
{
  public:
    // void Init(void) override { uart_init(); }
    void SetBaudRate(uint32_t baud) override { uart_setprotocol(baud, XUART_PARITY_NO, UART_STOPBIT_1); }
    bool full(void) { return !uart_tx_notfull(); }
    void putbuf(uint8_t* const buf, uint16_t len) override { uart_putbuf(buf, len); }
    bool available(void) override { return uart_rx_available(); }
    char getc(void) override { return uart_getc(); }
    void flush(void) override { uart_rx_flush(); uart_tx_flush(); }
    // uint16_t bytes_available(void) override { return uart_rx_bytesavailable(); }
    // bool has_systemboot(void) override { return uart_has_systemboot(); }
};

tJrPin5SerialPort jrpin5serial;


#endif // JRPIN5_INTERFACE_ESP_H
