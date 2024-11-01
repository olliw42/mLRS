//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// CAN standard library
// only init functions
//*******************************************************
// Interface:
//
//
//*******************************************************
#ifndef STDSTM32_CAN_H
#define STDSTM32_CAN_H
#ifdef __cplusplus
extern "C" {
#endif


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

//#include "stdstm32-peripherals.h"

#ifdef STM32G4
#if defined CAN_USE_FDCAN1_PA11PA12
    #define CAN_DC_HAL_INTFC    DC_HAL_CAN1
    #define CAN_RX_IO           IO_PA11
    #define CAN_TX_IO           IO_PA12
#elif defined CAN_USE_FDCAN2_PB5PB6
    #define CAN_DC_HAL_INTFC    DC_HAL_CAN2
    #define CAN_RX_IO           IO_PB5
    #define CAN_TX_IO           IO_PB6
    #ifndef FDCAN2
      #error CAN_USE_FDCAN2_xxxx defined buf FDCAN2 not available!
    #endif
#else
    #warning CAN_USE_FDCANx_xxxx not defined! CAN_USE_FDCAN1_PA11PA12 assumed.
    #defined CAN_USE_FDCAN1_PA11PA12
    #define CAN_DC_HAL_INTFC    DC_HAL_CAN1
    #define CAN_RX_IO           IO_PA11
    #define CAN_TX_IO           IO_PA12
#endif
#endif


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

#ifdef STM32F1
//-- STM32F1

// This code has once worked, but may not anymore today, hasn't been tested/used for a while

#ifndef HAL_CAN_MODULE_ENABLED
  #error HAL_CAN_MODULE_ENABLED not defined, enable it in Core\Inc\stm32f1xx_hal_conf.h!
#endif


void can_init(void)
{
    // CAN peripheral initialization

    gpio_init(IO_PA11, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init_af(IO_PA12, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_9, IO_SPEED_VERYFAST);

    //__HAL_RCC_CAN1_CLK_ENABLE();
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN1);

    // DroneCAN Hal initialization

    tDcHalCanTimings timings;
    int16_t res = dc_hal_compute_timings(HAL_RCC_GetPCLK1Freq(), 1000000, &timings); // = 36000000, CAN is on slow APB1 bus
    if (res < 0) {
        dbg.puts("\nERROR: Solution for CAN timings could not be found");
        return;
    }
    dbg.puts("\n  PCLK1: ");dbg.puts(u32toBCD_s(HAL_RCC_GetPCLK1Freq()));
    dbg.puts("\n  Prescaler: ");dbg.puts(u16toBCD_s(timings.bit_rate_prescaler));
    dbg.puts("\n  BS1: ");dbg.puts(u8toBCD_s(timings.bit_segment_1));
    dbg.puts("\n  BS2: ");dbg.puts(u8toBCD_s(timings.bit_segment_2));
    dbg.puts("\n  SJW: ");dbg.puts(u8toBCD_s(timings.sync_jump_width));
    // 4, 7, 1, 1

    //res = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
    res = dc_hal_init(&timings, DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);
    if (res < 0) {
        dbg.puts("\nERROR: Failed to open CAN iface ");dbg.puts(s16toBCD_s(res));
        return;
    }
}


#elif defined STM32G4
//-- STM32G4

#ifndef HAL_FDCAN_MODULE_ENABLED
  #error HAL_FDCAN_MODULE_ENABLED not defined, enable it in Core\Inc\stm32g4xx_hal_conf.h!
#endif


void can_init(void)
{
    // GPIO initialization
    // PA11 = FDCAN1_RX
    // PA12 = FDCAN1_TX
    gpio_init_af(CAN_RX_IO, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_9, IO_SPEED_VERYFAST);
    gpio_init_af(CAN_TX_IO, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_9, IO_SPEED_VERYFAST);

    // FDCAN clock initialization

    LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PCLK1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_FDCAN);
    //LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_FDCAN);
    //LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_FDCAN);

    // DroneCAN HAL initialization

    tDcHalCanTimings timings;
    //int16_t res = dc_hal_compute_timings(HAL_RCC_GetPCLK1Freq(), 1000000, &timings);
    int16_t res = dc_hal_compute_timings(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN), 1000000, &timings);
    if (res < 0) {
        DBG_DC(dbg.puts("\nERROR: Solution for CAN timings could not be found");)
        return;
    }

    DBG_DC(dbg.puts("\n  PCLK1: ");dbg.puts(u32toBCD_s(HAL_RCC_GetPCLK1Freq()));
    dbg.puts("\n  FDCAN CLK: ");dbg.puts(u32toBCD_s(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN)));
    dbg.puts("\n  Prescaler: ");dbg.puts(u16toBCD_s(timings.bit_rate_prescaler));
    dbg.puts("\n  BS1: ");dbg.puts(u8toBCD_s(timings.bit_segment_1));
    dbg.puts("\n  BS2: ");dbg.puts(u8toBCD_s(timings.bit_segment_2));
    dbg.puts("\n  SJW: ");dbg.puts(u8toBCD_s(timings.sync_jump_width));)

    res = dc_hal_init(CAN_DC_HAL_INTFC, &timings, DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);
    if (res < 0) {
        DBG_DC(dbg.puts("\nERROR: Failed to open CAN iface ");dbg.puts(s16toBCD_s(res));)
        return;
    }
}

#endif // STM32G4


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_CAN_H
