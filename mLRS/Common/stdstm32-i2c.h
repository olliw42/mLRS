//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// STM32Cube HAL based I2C standard library
//*******************************************************
// a bit limited so far !!!
//
// Interface:
//
// #define I2C_USE_I2C1, I2C_USE_I2C2, I2C_USE_I2C3
//
// #define I2C_USE_ITMODE
// #define I2C_IT_IRQ_PRIORITY // may also be needed for DMA mode !
//
// #define I2C_USE_DMAMODE
// #define I2C_DMA_PRIORITY
// #define I2C_DMA_IRQ_PRIORITY
//
// #defined I2C_CLOCKSPEED_100KHZ
// #defined I2C_CLOCKSPEED_400KHZ
// #defined I2C_CLOCKSPEED_1000KHZ
//
//*******************************************************
#ifndef STDSTM32_I2C_H
#define STDSTM32_I2C_H
#ifdef __cplusplus
extern "C" {
#endif
#ifndef HAL_I2C_MODULE_ENABLED
  #error HAL_I2C_MODULE_ENABLED not defined, enable it in Core\Inc\stm32yyxx_hal_conf.h!
#else


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#ifndef I2C_IT_IRQ_PRIORITY
  #define I2C_IT_IRQ_PRIORITY  15
#endif

#ifndef I2C_DMA_PRIORITY
  #define I2C_DMA_PRIORITY  DMA_PRIORITY_MEDIUM
#endif
#ifndef I2C_DMA_IRQ_PRIORITY
  #define I2C_DMA_IRQ_PRIORITY  15
#endif

#ifndef I2C_BLOCKING_TMO_MS
  #define I2C_BLOCKING_TMO_MS  HAL_MAX_DELAY
#endif


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

//#include "stdstm32-peripherals.h"

#if defined I2C1 && defined I2C_USE_I2C1
  #define I2C_I2Cx               I2C1
#ifdef STM32F1  
  #define I2C_SCL_IO             IO_PB6
  #define I2C_SDA_IO             IO_PB7
  #define I2C_SCL_IO_AF          IO_AF_DEFAULT
  #define I2C_SDA_IO_AF          IO_AF_DEFAULT
  
  #define I2C_TX_DMAx_Channely_IRQn        DMA1_Channel6_IRQn
  #define I2C_RX_DMAx_Channely_IRQn        DMA1_Channel7_IRQn
  #define I2C_TX_DMAx_Channely_IRQHandler  DMA1_Channel6_IRQHandler
  #define I2C_RX_DMAx_Channely_IRQHandler  DMA1_Channel7_IRQHandler
#endif  

#elif defined I2C2 && defined I2C_USE_I2C2
  #define I2C_I2Cx               I2C2
  #error I2C: Using I2C2 not yet supported!

#elif defined I2C3 && defined I2C_USE_I2C3
  #define I2C_I2Cx               I2C3
#ifdef STM32G4
  #define I2C_SCL_IO             IO_PC9
  #define I2C_SDA_IO             IO_PA8
  #define I2C_SCL_IO_AF          IO_AF_8 // GPIO_AF8_I2C3
  #define I2C_SDA_IO_AF          IO_AF_2 // GPIO_AF2_I2C3

  #define I2C_EV_IRQn            I2C3_EV_IRQn
  #define I2C_ER_IRQn            I2C3_ER_IRQn
  #define I2C_EV_IRQHandler      I2C3_EV_IRQHandler
  #define I2C_ER_IRQHandler      I2C3_ER_IRQHandler

  #define I2C_TX_DMAx_Channely_IRQn        DMA1_Channel2_IRQn
  #define I2C_RX_DMAx_Channely_IRQn        DMA1_Channel1_IRQn
  #define I2C_TX_DMAx_Channely_IRQHandler  DMA1_Channel2_IRQHandler
  #define I2C_RX_DMAx_Channely_IRQHandler  DMA1_Channel1_IRQHandler
#endif

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


// MspInit(), called in HAL_I2C_Init()
// ST recommends to not mix HAL and LL for a peripheral, so let's use HAL for i2c & dma
#ifndef STDSTM32_I2C_MSPINIT
#define STDSTM32_I2C_MSPINIT

void HAL_I2C_MspInit(I2C_HandleTypeDef* _hi2c)
{
#if defined I2C1 && defined I2C_USE_I2C1 && defined STM32F1
  if (_hi2c->Instance == I2C1) {
      
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

    __HAL_LINKDMA(_hi2c, hdmarx, hdma_i2c_rx);

    hdma_i2c_tx.Instance = DMA1_Channel6;
    hdma_i2c_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c_tx) != HAL_OK) { }

    __HAL_LINKDMA(_hi2c, hdmatx, hdma_i2c_tx);
    #endif
  }
#endif

#if defined I2C3 && defined I2C_USE_I2C3 && defined STM32G4
  if (_hi2c->Instance == I2C3) {

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

    __HAL_LINKDMA(_hi2c, hdmarx, hdma_i2c_rx);

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

    __HAL_LINKDMA(_hi2c, hdmatx, hdma_i2c_tx);
    #endif
  }
#endif
}

#endif


void MX_I2C_Init(void)
{
  hi2c.Instance = I2C_I2Cx;

  hi2c.Init.OwnAddress1 = 0;
  hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c.Init.OwnAddress2 = 0;
  hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

#ifdef STM32F1
#if defined I2C_CLOCKSPEED_100KHZ
  hi2c.Init.ClockSpeed = 100000;
#elif defined I2C_CLOCKSPEED_400KHZ
  hi2c.Init.ClockSpeed = 400000;
#elif defined I2C_CLOCKSPEED_1000KHZ
  #error I2C: 1000 kHz not supported!
#else
  hi2c.Init.ClockSpeed = 100000;
  #warning I2C: no clock speed defined, set to 100 kHz!
#endif
  hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;

#elif defined STM32G4
#if defined I2C_CLOCKSPEED_100KHZ
  hi2c.Init.Timing = 0x30A0A7FB; // StandardMode 100 kHz
#elif defined I2C_CLOCKSPEED_400KHZ
  hi2c.Init.Timing = 0x10802D9B; // FastdMode 400 kHz
#elif defined I2C_CLOCKSPEED_1000KHZ
  hi2c.Init.Timing = 0x00802172; // FastdMode 1000 kHz
#else
  hi2c.Init.Timing = 0x30A0A7FB;
  #warning I2C: no clock speed defined, set to 100 kHz!
#endif
  hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
#endif

  if (HAL_I2C_Init(&hi2c) != HAL_OK) return; // abort

#if defined STM32G4
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) return;
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c, 0) != HAL_OK) return;
#endif

#if defined I2C_USE_ITMODE || (defined I2C_USE_DMAMODE && defined STM32G4)
  // somehow G4 seems to need isr also for DMA mode
  nvic_irq_enable_w_priority(I2C_EV_IRQn, I2C_IT_IRQ_PRIORITY);
  nvic_irq_enable_w_priority(I2C_ER_IRQn, I2C_IT_IRQ_PRIORITY);
#endif

#ifdef I2C_USE_DMAMODE
  nvic_irq_enable_w_priority(I2C_TX_DMAx_Channely_IRQn, I2C_DMA_IRQ_PRIORITY);
  nvic_irq_enable_w_priority(I2C_RX_DMAx_Channely_IRQn, I2C_DMA_IRQ_PRIORITY);
#endif
}


//-------------------------------------------------------
// ISR routines
//-------------------------------------------------------

#if defined I2C_USE_ITMODE || (defined I2C_USE_DMAMODE && defined STM32G4)
void I2C_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c);
}

void I2C_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c);
}
#endif

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
#elif defined I2C_USE_ITMODE
  return HAL_I2C_Mem_Write_IT(&hi2c, i2c_dev_adr << 1, reg_adr, 1, buf, len);
#else
  return HAL_I2C_Mem_Write(&hi2c, i2c_dev_adr << 1, reg_adr, 1, buf, len, I2C_BLOCKING_TMO_MS);
#endif
}


HAL_StatusTypeDef i2c_put_buf(uint8_t* buf, uint16_t len)
{
#ifdef I2C_USE_DMAMODE
  return HAL_I2C_Master_Transmit_DMA(&hi2c, i2c_dev_adr << 1, buf, len);
#elif defined I2C_USE_ITMODE
  return HAL_I2C_Master_Transmit_IT(&hi2c, i2c_dev_adr << 1, buf, len);
#else
  return i2c_put_buf_blocked(buf, len);
#endif
}


HAL_StatusTypeDef i2c_device_ready(void)
{
  return HAL_I2C_IsDeviceReady(&hi2c, i2c_dev_adr << 1, 10, HAL_MAX_DELAY);
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void i2c_init(void)
{
  MX_I2C_Init();

  i2c_dev_adr = 0;
}



//-------------------------------------------------------
#endif
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_I2C_H
