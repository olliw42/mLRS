/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
//*******************************************************
// Adapted by OlliW, OlliW42, www.olliw.eu
//*******************************************************

#ifdef STDSTM32_USE_USB

#include "usbd_cdc.h"


USBD_HandleTypeDef hUSBD_CDC;

extern USBD_DescriptorsTypeDef USBD_Desc; // declared in usbd_desc.c
extern PCD_HandleTypeDef hpcd_USB_FS; // declared in usbd_conf.c

USBD_CDC_ItfTypeDef USBD_CDC_fops; // forward declaration


// USB_HS_MAX_PACKET_SIZE is 512 !
#if !defined CDC_DATA_HS_MAX_PACKET_SIZE || !defined CDC_DATA_FS_MAX_PACKET_SIZE
#error CDC_DATA_HS_MAX_PACKET_SIZE or CDC_DATA_FS_MAX_PACKET_SIZE not defined, something is terribly bad !
#endif
#if CDC_DATA_HS_MAX_PACKET_SIZE > 64
#error CDC_DATA_HS_MAX_PACKET_SIZE too large !
#endif
#if CDC_DATA_FS_MAX_PACKET_SIZE > 64
#error CDC_DATA_FS_MAX_PACKET_SIZE too large !
#endif


#define USB_RXBUFSIZE 		256
#define USB_RXBUFSIZEMASK  	(USB_RXBUFSIZE-1)

volatile uint8_t usb_rxbuf[USB_RXBUFSIZE];
volatile uint16_t usb_rxwritepos; // pos at which the last byte was stored
volatile uint16_t usb_rxreadpos; // pos at which the next byte is to be fetched

#define USB_TXBUFSIZE 		256
#define USB_TXBUFSIZEMASK  	(USB_TXBUFSIZE-1)

volatile uint8_t usb_txbuf[USB_TXBUFSIZE];
volatile uint16_t usb_txwritepos; // pos at which the last byte was stored
volatile uint16_t usb_txreadpos; // pos at which the next byte is to be fetched

uint8_t usb_trbuf[USB_TXBUFSIZE];
void _cdc_transmit(void);

uint8_t usb_rcbuf[USB_RXBUFSIZE]; // ???? hä, why is this needed ???


void usb_init(void)
{
    // copied from SystemClock_Config()
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        //Error_Handler();
    }

    // copied from MX_USB_DEVICE_Init()
    if (USBD_Init(&hUSBD_CDC, &USBD_Desc, 0) != USBD_OK) {
        //Error_Handler();
    }
    if (USBD_RegisterClass(&hUSBD_CDC, &USBD_CDC) != USBD_OK) {
        //Error_Handler();
    }
    if (USBD_CDC_RegisterInterface(&hUSBD_CDC, &USBD_CDC_fops) != USBD_OK) {
        //Error_Handler();
    }
    if (USBD_Start(&hUSBD_CDC) != USBD_OK) {
        //Error_Handler();
    }

    usb_txwritepos = usb_txreadpos = 0;
    usb_rxwritepos = usb_rxreadpos = 0;
}


uint8_t usb_rx_available(void)
{
    if (usb_rxwritepos == usb_rxreadpos) return 0;
    return 1;
}


char usb_getc(void)
{
	  usb_rxreadpos = (usb_rxreadpos + 1) & USB_RXBUFSIZEMASK;
	  return usb_rxbuf[usb_rxreadpos];
}


uint8_t _usb_putc(uint8_t c)
{
	  uint16_t next = (usb_txwritepos + 1) & USB_TXBUFSIZEMASK;
	  if (usb_txreadpos != next) { // fifo not full
	      usb_txbuf[next] = c;
	      usb_txwritepos = next;
	      return 1;
	  }
	  return 0;
}


void usb_putc(uint8_t c)
{
	  if (_usb_putc(c)) _cdc_transmit();
}


void usb_puts(const char *s)
{
	  uint8_t written = 0;
	  while (*s) { written = _usb_putc(*s); s++; }
	  if (written) _cdc_transmit();
}


void usb_putbuf(uint8_t* buf, uint16_t len)
{
	  uint8_t written = 0;
	  for (uint16_t i = 0; i < len; i++) written = _usb_putc(buf[i]);
	  if (written) _cdc_transmit();
}



void USB_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}


static int8_t CDC_Init(void);
static int8_t CDC_DeInit(void);
static int8_t CDC_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive(uint8_t* pbuf, uint32_t* length);
static int8_t CDC_TransmitCplt(uint8_t* pbuf, uint32_t* length, uint8_t epnum);


USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
    CDC_Init,
    CDC_DeInit,
    CDC_Control,
    CDC_Receive,
    CDC_TransmitCplt
};


static int8_t CDC_Init(void)
{
	  USBD_CDC_SetRxBuffer(&hUSBD_CDC, usb_rcbuf); //usb_rxbuf); ??????
	  return (USBD_OK);
}


static int8_t CDC_DeInit(void)
{
	  return (USBD_OK);
}


static int8_t CDC_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
	  return (USBD_OK);
}


static int8_t CDC_Receive(uint8_t* pbuf, uint32_t* length)
{
	  USBD_CDC_SetRxBuffer(&hUSBD_CDC, &pbuf[0]);
	  USBD_CDC_ReceivePacket(&hUSBD_CDC);

	  for (uint8_t i = 0; i < *length; i++) {
	      uint16_t next = (usb_rxwritepos + 1) & USB_RXBUFSIZEMASK;
	      if (usb_rxreadpos != next) { // fifo not full
	          usb_rxbuf[next] = pbuf[i];
	          usb_rxwritepos = next;
	      }
	  }

	  return (USBD_OK);
}


void _cdc_transmit(void)
{
    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUSBD_CDC.pClassData;
    if (hcdc->TxState != 0) return;

    uint8_t len = 0;
    while (usb_txwritepos != usb_txreadpos) {
        usb_txreadpos = (usb_txreadpos + 1) & USB_TXBUFSIZEMASK;
        usb_trbuf[len++] = usb_txbuf[usb_txreadpos];
    }

    if (!len) return;

    USBD_CDC_SetTxBuffer(&hUSBD_CDC, usb_trbuf, len);
    USBD_CDC_TransmitPacket(&hUSBD_CDC);
}


int8_t CDC_TransmitCplt(uint8_t* pbuf, uint32_t* length, uint8_t epnum)
{
	  _cdc_transmit();
	  return (USBD_OK);
}


#endif // ifdef STDSTM32_USE_USB
