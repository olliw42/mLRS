//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my USB VCP standard library
//*******************************************************
#ifndef STDSTM32_USB_VCP
#define STDSTM32_USB_VCP
#ifdef __cplusplus
extern "C" {
#endif
#ifndef HAL_PCD_MODULE_ENABLED
  #error HAL_PCD_MODULE_ENABLED not defined, enable it in Core\Inc\stm32yyxx_hal_conf.h!
#else


#include "usbd_cdc.h"


//-------------------------------------------------------
// User Interface
//-------------------------------------------------------

uint8_t usb_rx_available(void);
char usb_getc(void);

void usb_putc(uint8_t c);
void usb_puts(const char *s);
void usb_putbuf(uint8_t* buf, uint16_t len);


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void usb_init(void);



//-------------------------------------------------------
#endif
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_USB_VCP

