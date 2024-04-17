//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP HAL
//********************************************************
#ifndef ESP_HAL_H
#define ESP_HAL_H
#pragma once

#define HAL_I2C_MODULE_ENABLED

/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

#endif // ESP_HAL