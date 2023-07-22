/**
  ******************************************************************************
  * @file           : usbd_desc.c
  * @version        : v2.0_Cube
  * @brief          : Header for usbd_conf.c file.
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
#ifndef __USBD_DESC__C__
#define __USBD_DESC__C__

#ifdef __cplusplus
 extern "C" {
#endif


#include "usbd_def.h"


#define DEVICE_ID1              (UID_BASE)
#define DEVICE_ID2              (UID_BASE + 0x4)
#define DEVICE_ID3              (UID_BASE + 0x8)

#define USB_SIZ_STRING_SERIAL   0x1A


/** Descriptor for the Usb device. */
extern USBD_DescriptorsTypeDef FS_Desc;



#ifdef __cplusplus
}
#endif

#endif /* __USBD_DESC__C__ */

