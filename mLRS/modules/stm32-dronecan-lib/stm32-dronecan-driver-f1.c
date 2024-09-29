//*******************************************************
// STM32 DroneCAN Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// DroneCAN Driver Library for STM32 using HAL
// for use with libcanard
//*******************************************************
#ifdef STM32F103xB

#include "stm32f1xx_hal.h"

#ifdef HAL_CAN_MODULE_ENABLED

#error The F1 CAN driver is most likely out of date and needs revisting !

#include "stm32-dronecan-driver.h"
#include <string.h>


#define DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX  14

#define DC_HAL_PRESCALER_MIN  1     // we could use IS_CAN_PRESCALER()
#define DC_HAL_PRESCALER_MAX  1024
#define DC_HAL_BS1_MIN        1     // we can't use IS_CAN_BS1(), etc.
#define DC_HAL_BS1_MAX        16
#define DC_HAL_BS2_MIN        1
#define DC_HAL_BS2_MAX        8
#define DC_HAL_SJW_MIN        1
#define DC_HAL_SJW_MAX        4


static tDcHalStatistics dc_hal_stats;

static bool dc_hal_abort_tx_on_error;

static CAN_HandleTypeDef hcan;


//-------------------------------------------------------
// Helper
//-------------------------------------------------------

static void _process_error_status(void)
{
    uint32_t esr = READ_REG(hcan.Instance->ESR);

    if ((esr & CAN_ESR_LEC) != 0) {
        CLEAR_BIT(hcan.Instance->ESR, CAN_ESR_LEC);
        dc_hal_stats.error_count++;

        if (dc_hal_abort_tx_on_error || ((esr & CAN_ESR_BOFF) != 0)) {
            HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        }
    }
}


//-------------------------------------------------------
// Init
//-------------------------------------------------------

int16_t dc_hal_init(
    const tDcHalCanTimings* const timings,
    const DC_HAL_IFACE_MODE_ENUM iface_mode)
{
    if ((iface_mode != DC_HAL_IFACE_MODE_NORMAL) &&
        (iface_mode != DC_HAL_IFACE_MODE_SILENT) &&
        (iface_mode != DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    if ((timings == NULL) ||
        (timings->bit_rate_prescaler < DC_HAL_PRESCALER_MIN) || (timings->bit_rate_prescaler > DC_HAL_PRESCALER_MAX) ||
        (timings->bit_segment_1 < DC_HAL_BS1_MIN) || (timings->bit_segment_1 > DC_HAL_BS1_MAX) ||
        (timings->bit_segment_2 < DC_HAL_BS2_MIN) || (timings->bit_segment_2 > DC_HAL_BS2_MAX) ||
        (timings->sync_jump_width < DC_HAL_SJW_MIN) || (timings->sync_jump_width > DC_HAL_SJW_MAX)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    memset(&dc_hal_stats, 0, sizeof(dc_hal_stats));
    dc_hal_abort_tx_on_error = (iface_mode == DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);

    hcan.Instance = CAN1;
    __HAL_CAN_DISABLE_IT(&hcan, 0);

    hcan.Init.Prescaler = timings->bit_rate_prescaler;
    hcan.Init.Mode = CAN_MODE_NORMAL;

    uint32_t sync_jump_width[4] = {
        CAN_SJW_1TQ, CAN_SJW_2TQ, CAN_SJW_3TQ, CAN_SJW_4TQ
    };
    hcan.Init.SyncJumpWidth = sync_jump_width[timings->sync_jump_width - 1];

    uint32_t time_seg_1[16] = {
        CAN_BS1_1TQ, CAN_BS1_2TQ, CAN_BS1_3TQ, CAN_BS1_4TQ,
        CAN_BS1_5TQ, CAN_BS1_6TQ, CAN_BS1_7TQ, CAN_BS1_8TQ,
        CAN_BS1_9TQ, CAN_BS1_10TQ, CAN_BS1_11TQ, CAN_BS1_12TQ,
        CAN_BS1_13TQ, CAN_BS1_14TQ, CAN_BS1_15TQ, CAN_BS1_16TQ,
    };
    hcan.Init.TimeSeg1 = time_seg_1[timings->bit_segment_1 - 1];

    uint32_t time_seg_2[16] = {
        CAN_BS2_1TQ, CAN_BS2_2TQ, CAN_BS2_3TQ, CAN_BS2_4TQ,
        CAN_BS2_5TQ, CAN_BS2_6TQ, CAN_BS2_7TQ, CAN_BS2_8TQ
    };
    hcan.Init.TimeSeg2 = time_seg_2[timings->bit_segment_2 - 1];

    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = ENABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    HAL_StatusTypeDef hres = HAL_CAN_Init(&hcan);
    if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_INIT; }

    // libcanard's default filter setup is to set all filters to
    // - identifier mask mode
    // - 32-bit
    // - alternate between FIFO0 and FIFO1
    // HAL_CAN_ConfigFilter() sets the filter registers even when it is disabled

    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterIdHigh = 0;
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // CAN_FILTERSCALE_16BIT
    sFilterConfig.SlaveStartFilterBank = 0; // for single CAN instances, this parameter is meaningless

    for (uint8_t n = 0; n < DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX; n++) {
        sFilterConfig.FilterFIFOAssignment = ((n & 0x01) == 0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
        sFilterConfig.FilterBank = n;
        sFilterConfig.FilterActivation = (n == 0) ? CAN_FILTER_ENABLE : CAN_FILTER_DISABLE;
        hres = HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
        if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_CONFIG_FILTER; }
    }

    return 0;
}


int16_t dc_hal_start(void)
{
    HAL_StatusTypeDef hres = HAL_CAN_Start(&hcan);
    if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_START; }

    return 0;
}


//-------------------------------------------------------
// Transmit
//-------------------------------------------------------

int16_t dc_hal_transmit(const CanardCANFrame* const frame)
{
    if (frame == NULL) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    if (frame->id & CANARD_CAN_FRAME_ERR) {
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }
    if (frame->id & CANARD_CAN_FRAME_RTR) { // DroneCAN does not use REMOTE frames
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }
    if (!(frame->id & CANARD_CAN_FRAME_EFF)) { // DroneCAN does not use STD ID, uses only EXT frames
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }    

    _process_error_status();

    // IMPORTANT: We must not put the new frame into the mailboxes if there is already a
    // frame with higher or equal priority of the new frame.
    // Otherwise we would get screwed up transmission. E.g. even GetNodeInfo fails.
    // HAL_CAN_AddTxMessage() chooses the lowest free mailbox.
    // The crudest method is to just use always only one mailbox
    if (HAL_CAN_IsTxMessagePending(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) > 0) {
        return 0;
    }

/*
    uint32_t free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    if (free == 0) return 0; // all mailboxes are busy, so we must skip out
    if (free < 3) { // at least one mailbox is used, detailed check is needed
        return 0; // that's the crudest option
    }
*/

    // note: DroneCAN uses only EXT frames, so no handling needed here
    // libcanard has the CANARD_CAN_FRAME_EFF flag set in frame->id,
    // but we filter it out here anyway
    CAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.StdId = 0; // irrelevant
    pTxHeader.ExtId = (frame->id & CANARD_CAN_EXT_ID_MASK);
    pTxHeader.IDE = CAN_ID_EXT;
    pTxHeader.RTR = CAN_RTR_DATA;

    pTxHeader.DLC = frame->data_len;
    pTxHeader.TransmitGlobalTime = DISABLE;

    uint32_t pTxMailbox;

    HAL_StatusTypeDef hres = HAL_CAN_AddTxMessage(&hcan, &pTxHeader, frame->data, &pTxMailbox);
    if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_ADD_TX_MESSAGE; }

    return 1;
}


//-------------------------------------------------------
// Receive
//-------------------------------------------------------

int16_t dc_hal_receive(CanardCANFrame* const frame)
{
    if (frame == NULL) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    _process_error_status();

    uint32_t rx_fifo[2] = { CAN_RX_FIFO0, CAN_RX_FIFO1 };

    for (uint8_t i = 0; i < 2; i++) {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan, rx_fifo[i]) > 0) { // message in Rx fifo
            CAN_RxHeaderTypeDef pRxHeader;
            HAL_StatusTypeDef hres = HAL_CAN_GetRxMessage(&hcan, rx_fifo[i], &pRxHeader, frame->data);
            if (hres != HAL_OK) {
                continue; // return -DC_HAL_ERROR_CAN_GET_RX_MESSAGE;
            }

            // DroneCAN uses only EXT frames, so these should be errors
            if (pRxHeader.IDE != CAN_ID_EXT) {
                continue; // return -DC_HAL_ERROR_CAN_GET_RX_MESSAGE;
            }
            if (pRxHeader.RTR == CAN_RTR_REMOTE) {
                continue; // return -DC_HAL_ERROR_CAN_GET_RX_MESSAGE;
            }

            frame->id = (pRxHeader.ExtId & CANARD_CAN_EXT_ID_MASK);
            frame->id |= CANARD_CAN_FRAME_EFF; // libcanard wants the CANARD_CAN_FRAME_EFF flag be set

            frame->data_len = pRxHeader.DLC;
            frame->iface_id = 0;

            return 1;
        }
    }

    return 0;
}


//-------------------------------------------------------
// Filter
//-------------------------------------------------------

// num_filter_configs = 0 rejects all frames
int16_t dc_hal_config_acceptance_filters(
    const tDcHalAcceptanceFilterConfiguration* const filter_configs,
    const uint8_t num_filter_configs)
{
    if ((filter_configs == NULL) || (num_filter_configs > DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // CAN_FILTERSCALE_16BIT
    sFilterConfig.SlaveStartFilterBank = 0; // for single CAN instances, this parameter is meaningless

    for (uint8_t n = 0; n < num_filter_configs; n++) {
        if (filter_configs[n].rx_fifo == DC_HAL_RX_FIFO0) {
            sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        } else
        if (filter_configs[n].rx_fifo == DC_HAL_RX_FIFO1) {
            sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
        } else {
            sFilterConfig.FilterFIFOAssignment = ((n & 0x01) == 0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
        }
        sFilterConfig.FilterBank = n;

        // in the STM32F103 the CAN id's are store left aligned
        // the filter doesn't distinguish between STD and EXT ID, so we need to correct here
        // DroneCAN uses only EXT frames, so nothing to do else here
        // libcanard has the CANARD_CAN_FRAME_EFF flag set, but it is filtered out here anyway
        uint32_t id   = (filter_configs[n].id   & CANARD_CAN_EXT_ID_MASK) << 3;
        uint32_t mask = (filter_configs[n].mask & CANARD_CAN_EXT_ID_MASK) << 3;

        sFilterConfig.FilterIdHigh = (id >> 16) & 0x0000FFFF;
        sFilterConfig.FilterIdLow = id & 0x0000FFFF;
        sFilterConfig.FilterMaskIdHigh = (mask >> 16) & 0x0000FFFF;
        sFilterConfig.FilterMaskIdLow = mask & 0x0000FFFF;

        sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;

        HAL_StatusTypeDef hres = HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
        if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_CONFIG_FILTER; }
    }

    // fill remaining filters with default
    sFilterConfig.FilterIdHigh = 0;
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    for (uint8_t n = num_filter_configs; n < DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX; n++) {
        sFilterConfig.FilterFIFOAssignment = ((n & 0x01) == 0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
        sFilterConfig.FilterBank = n;
        sFilterConfig.FilterActivation = CAN_FILTER_DISABLE;
        HAL_StatusTypeDef hres = HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
        if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_CONFIG_FILTER; }
    }

    return 0;
}


//-------------------------------------------------------
// More Helper
//-------------------------------------------------------

tDcHalStatistics dc_hal_get_stats(void)
{
    return dc_hal_stats;
}


int16_t dc_hal_compute_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_bitrate,
    tDcHalCanTimings* const timings)
{
    if (target_bitrate != 1000000) {
        return -DC_HAL_ERROR_UNSUPPORTED_BIT_RATE;
    }
    if (peripheral_clock_rate != 36000000) { // 36 MHz, CAN is on slower APB1 bus
        return -DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY;
    }

    // determined using original libcanard function
    timings->bit_rate_prescaler = 4;
    timings->bit_segment_1 = 7;
    timings->bit_segment_2 = 1;
    timings->sync_jump_width = 1;

    return 0;
}


#endif // HAL_PCD_MODULE_ENABLED
#endif // STM32F1
