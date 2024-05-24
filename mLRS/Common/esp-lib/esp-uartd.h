//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP UARTD
//********************************************************
#ifndef ESPLIB_UARTD_H
#define ESPLIB_UARTD_H

#ifdef ESP32
#include "driver/uart.h"
#include "hal/uart_ll.h"
#endif

#ifndef ESPLIB_UART_ENUMS
#define ESPLIB_UART_ENUMS

typedef enum {
    XUART_PARITY_NO = 0,
    XUART_PARITY_EVEN,
    XUART_PARITY_ODD,
} UARTPARITYENUM;

typedef enum {
    UART_STOPBIT_1 = 0,
    UART_STOPBIT_2,
} UARTSTOPBITENUM;

#endif


#ifdef UARTD_USE_SERIAL
#ifdef ESP32
  #define UARTD_SERIAL_NO       UART_NUM_0
#elif defined ESP8266
  #define UARTD_SERIAL_NO       Serial
#endif
#elif defined UARTD_USE_SERIAL1
#ifdef ESP32
  #define UARTD_SERIAL_NO       UART_NUM_1
#elif defined ESP8266
  #define UARTD_SERIAL_NO       Serial1
#endif
#elif defined UARTD_USE_SERIAL2
#ifdef ESP32
  #define UARTD_SERIAL_NO       UART_NUM_2
#endif
#else
  #error UARTD_SERIAL_NO must be defined!
#endif

#ifndef UARTD_TXBUFSIZE
  #define UARTD_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UARTD_RXBUFSIZE
  #define UARTD_RXBUFSIZE       256 // MUST be 2^N
#endif


IRAM_ATTR void uartd_putbuf(uint8_t* buf, uint16_t len)
{
#ifdef ESP32
    uart_write_bytes(UARTD_SERIAL_NO, (uint8_t*)buf, len);
#elif defined ESP8266
    UARTD_SERIAL_NO.write((uint8_t*)buf, len);
#endif
}


IRAM_ATTR char uartd_getc(void)
{
#ifdef ESP32
    uint8_t c = 0;
    uart_read_bytes(UARTD_SERIAL_NO, &c, 1, 0);
    return (char)c;
#elif defined ESP8266
    return (char)UARTD_SERIAL_NO.read();
#endif
}


IRAM_ATTR void uartd_rx_flush(void)
{
#ifdef ESP32
    uart_flush(UARTD_SERIAL_NO);
#elif defined ESP8266
    while (UARTD_SERIAL_NO.available() > 0) UARTD_SERIAL_NO.read();
#endif
}


IRAM_ATTR void uartd_tx_flush(void)
{
#ifdef ESP32
    //uart_wait_tx_done(UARTD_SERIAL_NO, 100);  // 100 ms - what should be used?
#elif defined ESP8266
    UARTD_SERIAL_NO.flush();
#endif
}


IRAM_ATTR uint16_t uartd_rx_bytesavailable(void)
{
#ifdef ESP32
    uint32_t bytesAvailable = 0;
    uart_get_buffered_data_len(UARTD_SERIAL_NO, &bytesAvailable);
    return (uint16_t)bytesAvailable;
#elif defined ESP8266
    return (UARTD_SERIAL_NO.available() > 0) ? UARTD_SERIAL_NO.available() : 0;
#endif
}


IRAM_ATTR uint16_t uartd_rx_available(void)
{
#ifdef ESP32
    uint32_t bytesAvailable = 0;
    uart_get_buffered_data_len(UARTD_SERIAL_NO, &bytesAvailable);
    return ((uint16_t)bytesAvailable > 0) ? 1 : 0;
#elif defined ESP8266
    return (UARTD_SERIAL_NO.available() > 0) ? 1 : 0;
#endif
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------
// Note: ESP32 has a hardware fifo for tx, which is 128 bytes in size. However, MAVLink messages
// can be larger than this, and data would thus be lost when put only into the fifo. It is therefore
// crucial to set a Tx buffer size of sufficient size. setTxBufferSize() is not available for ESP82xx.

void _uartd_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
#ifdef ESP32

    uart_parity_t _parity = UART_PARITY_DISABLE;
    switch (parity) {
        case XUART_PARITY_NO:
            _parity = UART_PARITY_DISABLE; break;
        case XUART_PARITY_EVEN:
            _parity = UART_PARITY_EVEN; break;        
        case XUART_PARITY_ODD:
            _parity = UART_PARITY_ODD; break;
    }

    uart_stop_bits_t _stopbits = UART_STOP_BITS_1;
    switch (stopbits) {
        case UART_STOPBIT_1:
            _stopbits = UART_STOP_BITS_1; break;
        case UART_STOPBIT_2:
            _stopbits = UART_STOP_BITS_2; break;        
    }

    uart_config_t uart_config = {
        .baud_rate  = (int)baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = _parity,
        .stop_bits  = _stopbits,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UARTD_SERIAL_NO, &uart_config));

#if defined UARTD_USE_TX_IO || defined UARTD_USE_RX_IO // both need to be defined
    ESP_ERROR_CHECK(uart_set_pin(UARTD_SERIAL_NO, UARTD_USE_TX_IO, UARTD_USE_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#else
    ESP_ERROR_CHECK(uart_set_pin(UARTD_SERIAL_NO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif

    ESP_ERROR_CHECK(uart_driver_install(UARTD_SERIAL_NO, UARTD_RXBUFSIZE, UARTD_TXBUFSIZE, 0, NULL, 0));  // rx buf size needs to be > 128
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(UARTD_SERIAL_NO, 8)); // default is 120 which is too much, buffer only 128 bytes
    ESP_ERROR_CHECK(uart_set_rx_timeout(UARTD_SERIAL_NO, 1));        // wait for 1 symbol (~11 bits) to trigger Rx ISR, default 2


#elif defined ESP8266
    UARTD_SERIAL_NO.setRxBufferSize(UARTD_RXBUFSIZE);
    UARTD_SERIAL_NO.begin(baud);
#endif
}


void uartd_setbaudrate(uint32_t baud)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTD_SERIAL_NO));
#elif defined ESP8266
    UARTD_SERIAL_NO.end();
#endif
    _uartd_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}


void uartd_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTD_SERIAL_NO));
#elif defined ESP8266
    UARTD_SERIAL_NO.end();
#endif
    _uartd_initit(baud, parity, stopbits);
}


void uartd_init(void)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTD_SERIAL_NO));
#elif defined ESP8266
    UARTD_SERIAL_NO.end();
#endif
    _uartd_initit(UARTD_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uartd_init_isroff(void)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTD_SERIAL_NO));
#elif defined ESP8266
    UARTD_SERIAL_NO.end();
#endif
    _uartd_initit(UARTD_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}


#endif // ESPLIB_UARTD_H
