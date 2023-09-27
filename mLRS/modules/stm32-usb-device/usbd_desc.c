/**
  ******************************************************************************
  * @file    usbd_desc_template.c
  * @author  MCD Application Team
  * @brief   This file provides the USBD descriptors and string formatting method.
  *          This template should be copied to the user folder,
  *          renamed and customized following user needs.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifdef STDSTM32_USE_USB

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"


#define USBD_VID                      0x0483
#define USBD_PID                      22336 // 0x5740
#define USBD_LANGID_STRING            1033 // 0x409
#define USBD_MANUFACTURER_STRING      "STMicroelectronics"
#define USBD_PRODUCT_HS_STRING        "STM32 Virtual ComPort"
#define USBD_PRODUCT_FS_STRING        "STM32 Virtual ComPort"
#define USBD_CONFIGURATION_HS_STRING  "CDC Config"
#define USBD_INTERFACE_HS_STRING      "CDC Interface"
#define USBD_CONFIGURATION_FS_STRING  "CDC Config"
#define USBD_INTERFACE_FS_STRING      "CDC Interface"


uint8_t* USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
uint8_t* USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
uint8_t* USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
uint8_t* USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
uint8_t* USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
uint8_t* USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);
uint8_t* USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t* length);


USBD_DescriptorsTypeDef USBD_Desc =
{
    USBD_FS_DeviceDescriptor,
    USBD_FS_LangIDStrDescriptor,
    USBD_FS_ManufacturerStrDescriptor,
    USBD_FS_ProductStrDescriptor,
    USBD_FS_SerialStrDescriptor,
    USBD_FS_ConfigStrDescriptor,
    USBD_FS_InterfaceStrDescriptor
};


/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
    0x12,                       /* bLength */
    USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
    0x00,                       /* bcdUSB */
    0x02,
    0x02,                       /* bDeviceClass */
    0x02,                       /* bDeviceSubClass */
    0x00,                       /* bDeviceProtocol */
    USB_MAX_EP0_SIZE,           /* bMaxPacketSize */
    LOBYTE(USBD_VID),           /* idVendor */
    HIBYTE(USBD_VID),           /* idVendor */
    LOBYTE(USBD_PID),           /* idProduct */
    HIBYTE(USBD_PID),           /* idProduct */
    0x00,                       /* bcdDevice rel. 2.00 */
    0x02,
    USBD_IDX_MFC_STR,           /* Index of manufacturer  string */
    USBD_IDX_PRODUCT_STR,       /* Index of product string */
    USBD_IDX_SERIAL_STR,        /* Index of serial number string */
    USBD_MAX_NUM_CONFIGURATION  /* bNumConfigurations */
}; /* USB_DeviceDescriptor */


/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING),
    HIBYTE(USBD_LANGID_STRING),
};


__ALIGN_BEGIN uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END =
{
    USB_SIZ_STRING_SERIAL,
    USB_DESC_TYPE_STRING,
};


__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;


static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);
static void Get_SerialNum(void);


uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    UNUSED(speed);

    *length = sizeof(USBD_FS_DeviceDesc);
    return (uint8_t *)USBD_FS_DeviceDesc;
}


uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    UNUSED(speed);

    *length = sizeof(USBD_LangIDDesc);
    return (uint8_t *)USBD_LangIDDesc;
}


uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    if (speed == USBD_SPEED_HIGH) {
        USBD_GetString((uint8_t *)USBD_PRODUCT_HS_STRING, USBD_StrDesc, length);
    } else {
        USBD_GetString((uint8_t *)USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);
    }
    return USBD_StrDesc;
}


uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    UNUSED(speed);

    USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}


uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    UNUSED(speed);

    *length = USB_SIZ_STRING_SERIAL;

    /* Update the serial number string descriptor with the data from the unique ID*/
    Get_SerialNum();

    return (uint8_t *)USBD_StringSerial;
}


uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    if (speed == USBD_SPEED_HIGH) {
        USBD_GetString((uint8_t *)USBD_CONFIGURATION_HS_STRING, USBD_StrDesc, length);
    } else {
        USBD_GetString((uint8_t *)USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length);
    }
    return USBD_StrDesc;
}


uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    if (speed == USBD_SPEED_HIGH) {
        USBD_GetString((uint8_t *)USBD_INTERFACE_HS_STRING, USBD_StrDesc, length);
    } else {
        USBD_GetString((uint8_t *)USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);
    }
    return USBD_StrDesc;
}


static void Get_SerialNum(void)
{
    uint32_t deviceserial0;
    uint32_t deviceserial1;
    uint32_t deviceserial2;

    deviceserial0 = *(uint32_t *)DEVICE_ID1;
    deviceserial1 = *(uint32_t *)DEVICE_ID2;
    deviceserial2 = *(uint32_t *)DEVICE_ID3;

    deviceserial0 += deviceserial2;

    if (deviceserial0 != 0U) {
        IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8U);
        IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4U);
    }
}


static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
    uint8_t idx = 0U;

    for (idx = 0U; idx < len; idx++) {
        if (((value >> 28)) < 0xAU) {
            pbuf[2U * idx] = (value >> 28) + '0';
        } else {
            pbuf[2U * idx] = (value >> 28) + 'A' - 10U;
        }

        value = value << 4;

        pbuf[2U * idx + 1] = 0U;
    }
}


#endif // #ifdef STDSTM32_USE_USB
