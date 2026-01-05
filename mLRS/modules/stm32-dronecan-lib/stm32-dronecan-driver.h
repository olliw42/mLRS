//*******************************************************
// STM32 DroneCAN Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// DroneCAN Driver Library for STM32 using HAL
// for use with libcanard
//*******************************************************
// This driver follows very closely the original UAVCAN/DroneCAN driver
// for use with libcanard to make it essentially a drop in, i.e., to
// allow use without too much changes in existing code.
//*******************************************************
#ifndef STM32_DRONECAN_DRIVER_H
#define STM32_DRONECAN_DRIVER_H

// enable CAN FD support in libcanard (64-byte frames)
// disable TAO option to avoid changing encode function signatures
#define CANARD_ENABLE_TAO_OPTION 0
#define CANARD_ENABLE_CANFD 1

#include "libcanard/canard.h"

// library configuration
#define DRONECAN_USE_RX_ISR
#define DRONECAN_RXFRAMEBUFSIZE   64 // actual size is RXFRAMEBUFSIZE * sizeof(tDcRxFifoElement)
#define DRONECAN_IRQ_PRIORITY     14

// DRONECAN_RXFRAMEBUFSIZE = 64 is hopefully sufficient, catches CAN-29bit frames for 4.2 ms


#ifdef __cplusplus
extern "C"
{
#endif


typedef enum
{
    DC_HAL_ERROR_INVALID_ARGUMENT             = 1000,
    DC_HAL_ERROR_UNSUPPORTED_BIT_RATE         = 1001,
    DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT     = 1002,

    DC_HAL_ERROR_CAN_INIT                     = 2000,
    DC_HAL_ERROR_CAN_CONFIG_FILTER            = 2001,
    DC_HAL_ERROR_CAN_CONFIG_GLOBAL_FILTER     = 2002,
    DC_HAL_ERROR_CAN_START                    = 2003,
    DC_HAL_ERROR_CAN_ADD_TX_MESSAGE           = 2004,
    DC_HAL_ERROR_CAN_GET_RX_MESSAGE           = 2005,

    DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY  = 3000,
    DC_HAL_ERROR_TIMING                       = 3001,

    DC_HAL_ERROR_ISR_CONFIG                   = 4000,
} DC_HAL_ERROR_ENUM;


typedef enum
{
    DC_HAL_IFACE_MODE_NORMAL = 0,                   // Normal mode
    DC_HAL_IFACE_MODE_SILENT,                       // Do not affect the bus, only listen
    DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR   // Abort pending TX if a bus error has occurred
} DC_HAL_IFACE_MODE_ENUM;


typedef struct
{
    uint32_t bo_count; // bus off
    uint32_t lec_count; // PSR reg: LEC
    uint32_t ack_err_count; // PSR reg: LEC = 3 (ACK Error)
    uint32_t dlec_count; // PSR reg: DLEC
    uint32_t dlec_stuff_count; // DLEC = 1 (Stuff Error)
    uint32_t dlec_form_count;  // DLEC = 2 (Form Error)
    uint32_t dlec_ack_count;   // DLEC = 3 (ACK Error)
    uint32_t dlec_bit1_count;  // DLEC = 4 (Bit1 Error - recessive sent, dominant read)
    uint32_t dlec_bit0_count;  // DLEC = 5 (Bit0 Error - dominant sent, recessive read)
    uint32_t dlec_crc_count;   // DLEC = 6 (CRC Error)
    uint32_t pxd_count; // PSR reg: PXD
    uint32_t cel_count; // ECR reg: CEL
#ifdef DRONECAN_USE_RX_ISR
    uint32_t rx_overflow_count; // rx fifo overflow
    uint32_t isr_xtd_count; // XTD
    uint32_t isr_rtr_count; // RTR
    uint32_t isr_fdf_count; // FDF
    uint32_t isr_brs_count; // BRS
    uint32_t isr_dlc_count; // DLC
    uint32_t isr_rf0f_count; // RF0F
    uint32_t isr_rf0l_count; // RF0L
    uint32_t isr_rf1f_count; // RF1F
    uint32_t isr_rf1l_count; // RF1L
    uint32_t isr_errors_count;
    uint32_t isr_errorstatus_count;
    uint32_t tffl_count; // TFFL
    uint32_t tfqf_count; // TFQF
    uint32_t fd_rx_count; // FD frames received
    uint32_t fd_dlc_gt8_count; // FD frames received with DLC > 8 bytes
    uint32_t dlc_gt64_count; // dc_bytes_to_dlc called with len > 64 (should not happen)
#endif
    uint32_t error_sum_count;
    uint32_t last_psr;
    uint32_t last_ecr;
    uint32_t last_cccr;
    uint8_t tec;
    uint8_t rec;
    uint8_t last_lec;
    uint8_t last_dlec;
} tDcHalStatistics;


typedef enum
{
    DC_HAL_RX_FIFO_DEFAULT = 0,
    DC_HAL_RX_FIFO0,
    DC_HAL_RX_FIFO1,
} DC_HAL_RX_FIFO_ENUM;


typedef struct
{
    uint32_t id;
    uint32_t mask;
    uint8_t rx_fifo; // can be DC_HAL_RX_FIFO
} tDcHalAcceptanceFilterConfiguration;


typedef struct
{
    uint16_t bit_rate_prescaler;
    uint8_t bit_segment_1;
    uint8_t bit_segment_2;
    uint8_t sync_jump_width;
} tDcHalCanTimings;


typedef struct
{
    uint16_t bit_rate_prescaler;
    uint8_t bit_segment_1;
    uint8_t bit_segment_2;
    uint8_t sync_jump_width;
} tDcHalDataTimings;


typedef enum
{
    DC_HAL_CAN1 = 0,
    DC_HAL_CAN2,
} DC_HAL_CAN_ENUM;


int16_t dc_hal_init(
    DC_HAL_CAN_ENUM can_instance,
    const tDcHalCanTimings* const timings,
    const tDcHalDataTimings* const data_timings,
    const DC_HAL_IFACE_MODE_ENUM iface_mode);

int16_t dc_hal_start(void);

int16_t dc_hal_transmit(const CanardCANFrame* const frame, uint32_t tnow_ms);

int16_t dc_hal_receive(CanardCANFrame* const frame);

int16_t dc_hal_config_acceptance_filters(
    const tDcHalAcceptanceFilterConfiguration* const filter_configs,
    const uint8_t num_filter_configs);

tDcHalStatistics dc_hal_get_stats(void);

int16_t dc_hal_compute_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_bitrate,
    tDcHalCanTimings* const timings);

int16_t dc_hal_compute_data_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_data_bitrate,
    tDcHalDataTimings* const timings);

bool dc_hal_is_fd_mode(void);
void dc_hal_reset_fd_detection(void);

uint32_t dc_hal_debug_get_rxesc_reg(void);
uint32_t dc_hal_debug_get_cccr_reg(void);
uint32_t dc_hal_debug_get_dbtp_reg(void);
uint32_t dc_hal_debug_get_nbtp_reg(void);
uint32_t dc_hal_debug_get_tdcr_reg(void);
bool dc_hal_is_fd_enabled(void);


#ifdef DRONECAN_USE_RX_ISR
int16_t dc_hal_enable_isr(void);
void dc_hal_rx_flush(void);
#endif


#ifdef __cplusplus
}
#endif
#endif // STM32_DRONECAN_DRIVER_H
