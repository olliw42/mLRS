//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// STM32_USB_Device_Library based USB VCP standard library
//*******************************************************
// Init sequence
// usb_init()
//   -> USBD_Init() -> USBD_LL_Init() -> HAL_PCD_Init() -> HAL_PCD_MspInit()
#ifdef STDSTM32_USE_USB

#include "usbd_cdc.h"


USBD_HandleTypeDef husbd_CDC;

extern USBD_DescriptorsTypeDef USBD_Desc; // declared in usbd_desc.c
extern PCD_HandleTypeDef hpcd_USB_FS; // declared in usbd_conf.c

USBD_CDC_ItfTypeDef USBD_CDC_fops; // forward declaration
USBD_CDC_LineCodingTypeDef USBD_CDC_LineCoding;


// USB_HS_MAX_PACKET_SIZE is 512! define appears to be used nowhere however
// USB_FS_MAX_PACKET_SIZE is 64. define appears to be used nowhere however
// USB_MAX_EP0_SIZE is 64. define is used in plenty of places, so relate to it
#if !defined CDC_DATA_HS_MAX_PACKET_SIZE || !defined CDC_DATA_FS_MAX_PACKET_SIZE
  #error CDC_DATA_HS_MAX_PACKET_SIZE or CDC_DATA_FS_MAX_PACKET_SIZE not defined, something is terribly bad !
#endif
#if CDC_DATA_HS_MAX_PACKET_SIZE > 64
  #error CDC_DATA_HS_MAX_PACKET_SIZE too large !
#endif
#if CDC_DATA_FS_MAX_PACKET_SIZE > 64
  #error CDC_DATA_FS_MAX_PACKET_SIZE too large !
#endif
#if USB_MAX_EP0_SIZE != 64
  #error USB_MAX_EP0_SIZE is not 64, something is terribly strange !
#endif

#if !defined STM32G431xx && !defined STM32G441xx && !defined STM32G491xx && !defined STM32G474xx
  #warning Not tested on non STM32G4 MCUs
#endif


//#define USB_RXBUFSIZE           256
#if (USB_RXBUFSIZE & (USB_RXBUFSIZE - 1)) != 0
  #error USB_RXBUFSIZE must be a power of 2!
#endif
#if USB_RXBUFSIZE < 256
  #error USB_RXBUFSIZE must be at least 256!
#endif
#define USB_RXBUFSIZEMASK       (USB_RXBUFSIZE-1)

volatile uint8_t usb_rxbuf[USB_RXBUFSIZE];
volatile uint16_t usb_rxwritepos; // pos at which the last byte was stored
volatile uint16_t usb_rxreadpos; // pos at which the next byte is to be fetched

//#define USB_TXBUFSIZE           256
#if (USB_TXBUFSIZE & (USB_TXBUFSIZE - 1)) != 0
  #error USB_TXBUFSIZE must be a power of 2!
#endif
#define USB_TXBUFSIZEMASK       (USB_TXBUFSIZE-1)

volatile uint8_t usb_txbuf[USB_TXBUFSIZE];
volatile uint16_t usb_txwritepos; // pos at which the last byte was stored
volatile uint16_t usb_txreadpos; // pos at which the next byte is to be fetched

uint8_t usb_rcbuf[CDC_DATA_FS_MAX_PACKET_SIZE]; // only needs to hold one USB packet
uint8_t usb_trbuf[CDC_DATA_FS_MAX_PACKET_SIZE];
void _cdc_transmit(void); // forward declaration

static uint8_t usbd_initialized = 0; // to track if we have initialized usb already

#define USBD_DTR_FLAG  0x01
#define USBD_RTS_FLAG  0x02
volatile uint8_t usbd_dtr_rts; // to track DTR RTS state

// NAK flow control: when RX buffer is nearly full, don't call ReceivePacket
// this causes USB to NAK incoming packets until we have space
volatile uint8_t usb_rx_nak_pending;
#define USB_RX_NAK_THRESHOLD  (2 * CDC_DATA_FS_MAX_PACKET_SIZE) // NAK when less than this many bytes free, 128 bytes


uint32_t usb_baudrate(void) { return USBD_CDC_LineCoding.bitrate; }


void usb_init(void)
{
    // copied from MX_USB_DEVICE_Init()
    if (USBD_Init(&husbd_CDC, &USBD_Desc, 0) != USBD_OK) {
        //Error_Handler();
    }
    if (USBD_RegisterClass(&husbd_CDC, &USBD_CDC) != USBD_OK) {
        //Error_Handler();
    }
    if (USBD_CDC_RegisterInterface(&husbd_CDC, &USBD_CDC_fops) != USBD_OK) {
        //Error_Handler();
    }
    if (USBD_Start(&husbd_CDC) != USBD_OK) {
        //Error_Handler();
    }

    // 115200, 8N1
    USBD_CDC_LineCoding.bitrate = 115200;
    USBD_CDC_LineCoding.format = 0;
    USBD_CDC_LineCoding.paritytype = 0;
    USBD_CDC_LineCoding.datatype = 8;

    usb_txwritepos = usb_txreadpos = 0;
    usb_rxwritepos = usb_rxreadpos = 0;

    usbd_dtr_rts = 0;
    usb_rx_nak_pending = 0;

    usbd_initialized = 1;
}


void usb_deinit(void)
{
    if (!usbd_initialized) return;

    USBD_DeInit(&husbd_CDC);

#if defined STM32G431xx || defined STM32G441xx || defined STM32G491xx || defined STM32G474xx
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE; // important, otherwise HAL_RCC_OscConfig() will modify the PLL setting
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {}
#endif
}


uint8_t usb_rx_available(void)
{
    if (usb_rxwritepos == usb_rxreadpos) return 0;
    return 1;
}


uint16_t usb_rx_bytesavailable(void)
{
int16_t d;

    d = (int16_t)usb_rxwritepos - (int16_t)usb_rxreadpos;
    return (d < 0) ? d + USB_RXBUFSIZE : d;
}


char usb_getc(void)
{
    usb_rxreadpos = (usb_rxreadpos + 1) & USB_RXBUFSIZEMASK;
    char c = usb_rxbuf[usb_rxreadpos];

    // resume reception if NAK was pending and buffer now has space
    if (usb_rx_nak_pending) {
        if (usb_rx_bytesavailable() < USB_RXBUFSIZE - USB_RX_NAK_THRESHOLD) {
            usb_rx_nak_pending = 0;
            USBD_CDC_ReceivePacket(&husbd_CDC); // ready to receive again
        }
    }

    return c;
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


uint8_t usb_tx_full(void)
{
    uint16_t next = (usb_txwritepos + 1) & USB_TXBUFSIZEMASK;
    return !(usb_txreadpos != next); // not fifo not full
}


void usb_putc(uint8_t c)
{
    if (_usb_putc(c)) _cdc_transmit();
}


void usb_puts(const char* s)
{
    uint8_t written = 0;
    while (*s) { written = _usb_putc(*s); s++; }
    if (written) _cdc_transmit();
}


void usb_putbuf(uint8_t* const buf, uint16_t len)
{
    uint8_t written = 0;
    for (uint16_t i = 0; i < len; i++) written = _usb_putc(buf[i]);
    if (written) _cdc_transmit();
}


void usb_rx_flush(void)
{
    usb_rxwritepos = usb_rxreadpos = 0;
}


void usb_flush(void)
{
    usb_txwritepos = usb_txreadpos = 0;
    usb_rxwritepos = usb_rxreadpos = 0;
}


uint8_t usb_dtr_rts(void)
{
    return usbd_dtr_rts;
}


uint8_t usb_dtr_is_set(void)
{
    return (usbd_dtr_rts & USBD_DTR_FLAG) ? 1 : 0;
}


uint8_t usb_rts_is_set(void)
{
    return (usbd_dtr_rts & USBD_RTS_FLAG) ? 1 : 0;
}


void USBD_IRQHandler(void)
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
    USBD_CDC_SetRxBuffer(&husbd_CDC, usb_rcbuf);
    USBD_CDC_SetTxBuffer(&husbd_CDC, usb_trbuf, 0);
    usb_flush();

    return USBD_OK;
}


static int8_t CDC_DeInit(void)
{
    return USBD_OK;
}


static int8_t CDC_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    switch (cmd) {
    case CDC_SET_LINE_CODING:
        USBD_CDC_LineCoding.bitrate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
        USBD_CDC_LineCoding.format = pbuf[4];
        USBD_CDC_LineCoding.paritytype = pbuf[5];
        USBD_CDC_LineCoding.datatype = pbuf[6];
        break;
    case CDC_GET_LINE_CODING:
        pbuf[0] = (uint8_t)(USBD_CDC_LineCoding.bitrate);
        pbuf[1] = (uint8_t)(USBD_CDC_LineCoding.bitrate >> 8);
        pbuf[2] = (uint8_t)(USBD_CDC_LineCoding.bitrate >> 16);
        pbuf[3] = (uint8_t)(USBD_CDC_LineCoding.bitrate >> 24);
        pbuf[4] = USBD_CDC_LineCoding.format;
        pbuf[5] = USBD_CDC_LineCoding.paritytype;
        pbuf[6] = USBD_CDC_LineCoding.datatype;
        break;
    case CDC_SET_CONTROL_LINE_STATE:
        // https://community.st.com/t5/stm32-mcus-embedded-software/usb-vcp-how-to-know-if-host-com-port-is-open/td-p/363002
        // https://www.silabs.com/documents/public/application-notes/AN758.pdf
        // https://github.com/stm32duino/Arduino_Core_STM32/pull/1382/files
        // DTR is bit 0 (0x01) in wValue
        // RTS is bit 1 (0x02) in wValue
        // wValue is the same as pbuf[2] & pbuf[3] !! see USBD_SetupReqTypedef
        usbd_dtr_rts = 0;
        if ((((USBD_SetupReqTypedef*)pbuf)->wValue & 0x0001) != 0) usbd_dtr_rts |= USBD_DTR_FLAG;
        if ((((USBD_SetupReqTypedef*)pbuf)->wValue & 0x0002) != 0) usbd_dtr_rts |= USBD_RTS_FLAG;
        //usbd_dtr_rts = ((USBD_SetupReqTypedef*)pbuf)->wValue;
        break;
    }

    return USBD_OK;
}


static int8_t CDC_Receive(uint8_t* pbuf, uint32_t* length)
{
    // pbuf is equal to usb_rcbuf, so SetRxBuffer() is not needed
    // USBD_CDC_SetRxBuffer(&husbd_CDC, &pbuf[0]);

    if (*length > sizeof(usb_rcbuf)) return USBD_OK;

    // copy received data to ring buffer
    for (uint8_t i = 0; i < *length; i++) {
        uint16_t next = (usb_rxwritepos + 1) & USB_RXBUFSIZEMASK;
        if (usb_rxreadpos != next) { // fifo not full
            usb_rxbuf[next] = usb_rcbuf[i];
            usb_rxwritepos = next;
        }
    }

    // NAK flow control: check if we have space for another packet
    if (usb_rx_bytesavailable() < USB_RXBUFSIZE - USB_RX_NAK_THRESHOLD) {
        USBD_CDC_ReceivePacket(&husbd_CDC); // ready for next packet
    } else {
        usb_rx_nak_pending = 1; // NAK until space available
    }

    return USBD_OK;
}


void _cdc_transmit(void)
{
    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)husbd_CDC.pClassData;
    if (hcdc->TxState != 0) return;

    uint8_t len = 0;

    while ((usb_txwritepos != usb_txreadpos) && (len < 64-1)) { // the -1 avoids the need for a ZLP
        usb_txreadpos = (usb_txreadpos + 1) & USB_TXBUFSIZEMASK;
        usb_trbuf[len++] = usb_txbuf[usb_txreadpos];
    }

    if (!len) return;

    USBD_CDC_SetTxBuffer(&husbd_CDC, usb_trbuf, len); // only hcdc->TxLength = len; is really needed
    USBD_CDC_TransmitPacket(&husbd_CDC);
}


int8_t CDC_TransmitCplt(uint8_t* pbuf, uint32_t* length, uint8_t epnum)
{
    _cdc_transmit();
    return USBD_OK;
}


#endif // ifdef STDSTM32_USE_USB
