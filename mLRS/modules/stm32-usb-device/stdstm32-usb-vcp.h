//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// STM32_USB_Device_Library based USB VCP standard library
//*******************************************************
// ../Drivers/STM32_USB_Device_Library/Core/Inc
// ../Drivers/STM32_USB_Device_Library/Class/CDC/Inc
// "${workspace_loc:/${ProjName}/modules/stm32-usb-device}"
// STDSTM32_USE_USB
// #define HAL_PCD_MODULE_ENABLED in stm32yyxx_hal_conf.h

#ifndef STDSTM32_USB_VCP
#define STDSTM32_USB_VCP
#ifdef __cplusplus
extern "C" {
#endif
#ifndef HAL_PCD_MODULE_ENABLED
  #error HAL_PCD_MODULE_ENABLED not defined, enable it in Core\Inc\stm32yyxx_hal_conf.h!
#else


#include "usbd_cdc.h"


// pl adjust these settings in usbd_conf.h according to your needs
// - USB_RXBUFSIZE
// - USB_TXBUFSIZE
// - USBD_IRQ_PRIORITY


//-------------------------------------------------------
// User Interface
//-------------------------------------------------------

uint8_t usb_rx_available(void);
uint16_t usb_rx_bytesavailable(void);
char usb_getc(void);

void usb_putc(uint8_t c);
void usb_puts(const char *s);
void usb_putbuf(uint8_t* buf, uint16_t len);

void usb_rx_flush(void);
void usb_flush(void);

uint8_t usb_dtr_rts(void);
uint8_t usb_dtr_is_set(void);
uint8_t usb_rts_is_set(void);

void usb_init(void);

void usb_deinit(void);


//-------------------------------------------------------
#endif
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_USB_VCP

