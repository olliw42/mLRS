//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// STM32Cube HAL based I2C standard library
//*******************************************************
// very limited so far !!!
//
// Interface:
//
// #define I2C_USE_I2C1, I2C_USE_I2C2, I2C_USE_I2C3
// #define I2C_USE_DMAMODE
// #define I2C_DMA_PRIORITY
// #define I2C_DMA_IRQ_PRIORITY
//
// #defined I2C_USE_CLOCKSPEED_100KHZ
// #defined I2C_USE_CLOCKSPEED_400KHZ
// #defined I2C_USE_CLOCKSPEED_1000KHZ
//
//*******************************************************
#ifndef STDSTM32_I2C_H
#define STDSTM32_I2C_H
#ifdef __cplusplus
extern "C" {
#endif
#if !defined HAL_I2C_MODULE_ENABLED //&& !defined __STM32F1xx_HAL_I2C_H
  #error HAL_I2C_MODULE_ENABLED is not defined!
#else


#ifdef I2C_USE_DMAMODE
  #ifndef I2C_DMA_PRIORITY
    #define I2C_DMA_PRIORITY  DMA_PRIORITY_MEDIUM
  #endif
  #ifndef I2C_DMA_IRQ_PRIORITY
    #define I2C_DMA_IRQ_PRIORITY  15
  #endif
#endif

#ifndef I2C_BLOCKING_TMO_MS
  #define I2C_BLOCKING_TMO_MS  HAL_MAX_DELAY
#endif


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

//#include "stdstm32-peripherals.h"

#ifdef I2C_USE_I2C1
  #define I2C_I2Cx               I2C1
  #define I2C_SCL_IO             IO_PB6
  #define I2C_SDA_IO             IO_PB7
  #define I2C_SCL_IO_AF          IO_AF_DEFAULT
  #define I2C_SDA_IO_AF          IO_AF_DEFAULT

  #define I2C_TX_DMAx_Channely_IRQHandler  DMA1_Channel6_IRQHandler
  #define I2C_RX_DMAx_Channely_IRQHandler  DMA1_Channel7_IRQHandler

#elif defined I2C2 && defined I2C_USE_I2C2
  #define I2C_I2Cx               I2C2

#elif defined I2C3 && defined I2C_USE_I2C3
  #define I2C_I2Cx               I2C3
  #define I2C_SCL_IO             IO_PC9
  #define I2C_SDA_IO             IO_PA8
  #define I2C_SCL_IO_AF          IO_AF_8 // GPIO_AF8_I2C3
  #define I2C_SDA_IO_AF          IO_AF_2 // GPIO_AF2_I2C3

  #define I2C_TX_DMAx_Channely_IRQHandler  DMA1_Channel2_IRQHandler
  #define I2C_RX_DMAx_Channely_IRQHandler  DMA1_Channel1_IRQHandler

#else
#error Error in stdstm32-i2c.h
#endif


//-------------------------------------------------------
//  HAL interface
//-------------------------------------------------------

I2C_HandleTypeDef hi2c;
#ifdef I2C_USE_DMAMODE
DMA_HandleTypeDef hdma_i2c_rx;
DMA_HandleTypeDef hdma_i2c_tx;
#endif


// MspInit(), called by HAL
#ifndef STDSTM32_I2C_MSPINIT
#define STDSTM32_I2C_MSPINIT

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
#if defined I2C1 && defined I2C_USE_I2C1
  if (hi2c->Instance == I2C1) {
    gpio_init_af(I2C_SCL_IO, IO_MODE_OUTPUT_ALTERNATE_OD, I2C_SCL_IO_AF, IO_SPEED_FAST);
    gpio_init_af(I2C_SDA_IO, IO_MODE_OUTPUT_ALTERNATE_OD, I2C_SDA_IO_AF, IO_SPEED_FAST);

    __HAL_RCC_I2C1_CLK_ENABLE();

    #ifdef I2C_USE_DMAMODE
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_i2c_rx.Instance = DMA1_Channel7;
    hdma_i2c_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c_rx) != HAL_OK) {}

    __HAL_LINKDMA(hi2c, hdmarx, hdma_i2c_rx);

    hdma_i2c_tx.Instance = DMA1_Channel6;
    hdma_i2c_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c_tx) != HAL_OK) { }

    __HAL_LINKDMA(hi2c, hdmatx, hdma_i2c_tx);
    #endif
  }
#endif

#if defined I2C3 && defined I2C_USE_I2C3
  if (hi2c->Instance == I2C3) {

    gpio_init_af(I2C_SCL_IO, IO_MODE_OUTPUT_ALTERNATE_OD, I2C_SCL_IO_AF, IO_SPEED_FAST);
    gpio_init_af(I2C_SDA_IO, IO_MODE_OUTPUT_ALTERNATE_OD, I2C_SDA_IO_AF, IO_SPEED_FAST);

    __HAL_RCC_I2C3_CLK_ENABLE();

    #ifdef I2C_USE_DMAMODE
    __HAL_RCC_DMAMUX1_CLK_ENABLE(); // STM32CubeMX placed these incorrectly !
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_i2c_rx.Instance = DMA1_Channel1;
    hdma_i2c_rx.Init.Request = DMA_REQUEST_I2C3_RX;
    hdma_i2c_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c_rx.Init.Priority = I2C_DMA_PRIORITY;
    if (HAL_DMA_Init(&hdma_i2c_rx) != HAL_OK) {}

    __HAL_LINKDMA(hi2c, hdmarx, hdma_i2c3_rx);

    hdma_i2c_tx.Instance = DMA1_Channel2;
    hdma_i2c_tx.Init.Request = DMA_REQUEST_I2C3_TX;
    hdma_i2c_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c_tx.Init.Priority = I2C_DMA_PRIORITY;
    if (HAL_DMA_Init(&hdma_i2c_tx) != HAL_OK) {}

    __HAL_LINKDMA(hi2c, hdmatx, hdma_i2c_tx);
    #endif
  }
#endif
}

#endif


// peripherals

void i2c_init_pheripherals(void)
{
  hi2c.Instance = I2C_I2Cx;

  hi2c.Init.OwnAddress1 = 0;
  hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c.Init.OwnAddress2 = 0;
  hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

#ifdef STM32F1
#if defined I2C_USE_CLOCKSPEED_100KHZ
  hi2c.Init.ClockSpeed = 100000;
#elif defined I2C_USE_CLOCKSPEED_400KHZ
  hi2c.Init.ClockSpeed = 400000;
#elif defined I2C_USE_CLOCKSPEED_1000KHZ
#else
  hi2c.Init.ClockSpeed = 100000;
  #warning I2C: no clock speed defined, set to 100 kHz!
#endif
  hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;

#elif defined STM32G4
#if defined I2C_USE_CLOCKSPEED_100KHZ
  hi2c.Init.Timing = 0x30A0A7FB; // StandardMode 100 kHz
#elif defined I2C_USE_CLOCKSPEED_400KHZ
  hi2c.Init.Timing = 0x10802D9B; // FastdMode 400 kHz
#elif defined I2C_USE_CLOCKSPEED_1000KHZ
  hi2c.Init.Timing = 0x00802172; // FastdMode 1000 kHz
#endif
  hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
#endif

  if (HAL_I2C_Init(&hi2c) != HAL_OK) return; // abort

#if defined STM32G4
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) return;
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c, 0) != HAL_OK) return;
#endif

// we don't support IT mode
//  HAL_NVIC_SetPriority(I2C3_EV_IRQn, 15, 0);
//  HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
//  HAL_NVIC_SetPriority(I2C3_ER_IRQn, 15, 0);
//  HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);

#ifdef I2C_USE_DMAMODE
#ifdef STM32F1
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, I2C_DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, I2C_DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
#endif
#endif
}

// DMA interrupt routines

#ifdef I2C_USE_DMAMODE
void I2C_TX_DMAx_Channely_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_i2c_tx);
}

void I2C_RX_DMAx_Channely_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_i2c_rx);
}
#endif


//-------------------------------------------------------
//  I2C user routines
//-------------------------------------------------------

uint8_t i2c_dev_adr;


// call this before transaction
void i2c_setdeviceadr(uint8_t dev_adr)
{
  i2c_dev_adr = dev_adr;
}


HAL_StatusTypeDef i2c_put_blocked(uint8_t reg_adr, uint8_t* buf, uint16_t len)
{
  return HAL_I2C_Mem_Write(&hi2c, i2c_dev_adr << 1, reg_adr, 1, buf, len, I2C_BLOCKING_TMO_MS);
}


HAL_StatusTypeDef i2c_put_buf_blocked(uint8_t* buf, uint16_t len)
{
  return HAL_I2C_Master_Transmit(&hi2c, i2c_dev_adr << 1, buf, len, I2C_BLOCKING_TMO_MS);
}


HAL_StatusTypeDef i2c_put(uint8_t reg_adr, uint8_t* buf, uint16_t len)
{
#ifdef I2C_USE_DMAMODE
  return HAL_I2C_Mem_Write_DMA(&hi2c, i2c_dev_adr << 1, reg_adr, 1, buf, len);
#else
  return i2c_put_blocked(reg_adr, buf, len);
#endif
}


HAL_StatusTypeDef i2c_put_buf(uint8_t* buf, uint16_t len)
{
#ifdef I2C_USE_DMAMODE
  return HAL_I2C_Master_Transmit_DMA(&hi2c, i2c_dev_adr << 1, buf, len);
#else
  return i2c_put_buf_blocked(buf, len);
#endif
}


HAL_StatusTypeDef i2c_device_ready(void)
{
HAL_StatusTypeDef res;

  res = HAL_I2C_IsDeviceReady(&hi2c, i2c_dev_adr << 1, 10, HAL_MAX_DELAY);

  return res;
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void i2c_init(void)
{
  i2c_init_pheripherals();

  i2c_dev_adr = 0;
}



//-------------------------------------------------------
#endif
#ifdef __cplusplus
}
#endif
#endif //STDSTM32_I2C_H
