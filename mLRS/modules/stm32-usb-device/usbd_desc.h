/**
  ******************************************************************************
  * @file    usbd_desc_template.h
  * @author  MCD Application Team
  * @brief   Header for usbd_desc_template.c module
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
#ifndef __USBD_DESC_H
#define __USBD_DESC_H

#ifdef __cplusplus
extern "C" {
#endif


#include "usbd_def.h"


#define DEVICE_ID1              (UID_BASE)
#define DEVICE_ID2              (UID_BASE + 0x4U)
#define DEVICE_ID3              (UID_BASE + 0x8U)

#define USB_SIZ_STRING_SERIAL   0x1AU



#ifdef __cplusplus
}
#endif

#endif /* __USBD_DESC_H */

