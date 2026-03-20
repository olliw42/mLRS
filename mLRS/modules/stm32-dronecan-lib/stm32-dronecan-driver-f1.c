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

static uint32_t tx_tlast_ms; // for TX timeout error counting in dc_hal_transmit()
static uint8_t was_bo; // for counting bus off errors only when state changes


//-------------------------------------------------------
// Helper
//-------------------------------------------------------

static void _process_error_status(void)
{
    uint32_t esr = READ_REG(hcan.Instance->ESR);

    // bus off handling with toggle guard
    if ((esr & CAN_ESR_BOFF) != 0) {
        HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        if (!was_bo) dc_hal_stats.bo_count++; // BO, bus off, only count when toggled from bus on to bus off
        was_bo = 1;
    } else {
        was_bo = 0;
    }

    // last error code
    uint32_t lec = (esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;
    if (lec != 0 && lec != 7) { // 0 = no error, 7 = no change (unused on bxCAN, but be safe)
        CLEAR_BIT(hcan.Instance->ESR, CAN_ESR_LEC);
        dc_hal_stats.lec_count++;
        if (dc_hal_abort_tx_on_error) {
            HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        }
    }

    // transmit/receive error counters from ESR
    dc_hal_stats.tec_count = (esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
    dc_hal_stats.rec_count = (esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos;

    // bxCAN has no PXE, CEL, or DLEC equivalents — those stay at 0

    // record last values for diagnostics
    if (lec != 0) dc_hal_stats.last_lec = lec;
    dc_hal_stats.last_psr = esr; // ESR is the bxCAN equivalent of FDCAN's PSR
    dc_hal_stats.last_ecr = esr; // TEC/REC are in ESR on bxCAN (no separate ECR register)
    dc_hal_stats.last_cccr = READ_REG(hcan.Instance->MSR); // MSR is closest to FDCAN's CCCR
}


//-------------------------------------------------------
// Init
//-------------------------------------------------------

int16_t dc_hal_init(
    DC_HAL_CAN_ENUM can_instance, // irrelevant for STM32F1
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
    tx_tlast_ms = 0;
    was_bo = 0;

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

int16_t dc_hal_transmit(const CanardCANFrame* const frame, uint32_t tnow_ms)
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

    if (frame->data_len > 8) { // don't allow oversized frames
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }

    _process_error_status();

    // IMPORTANT: We must not put the new frame into the mailboxes if there is already a
    // frame with higher or equal priority of the new frame.
    // Otherwise we would get screwed up transmission. E.g. even GetNodeInfo fails.
    // HAL_CAN_AddTxMessage() chooses the lowest free mailbox.
    // The crudest method is to just use always only one mailbox
    if (HAL_CAN_IsTxMessagePending(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) > 0) {
        if (tx_tlast_ms > 0 && (tnow_ms - tx_tlast_ms) > 10) {
            dc_hal_stats.tffl_count++; // TFFL, Tx mailbox busy timeout
        }
        return 0;
    }

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

    tx_tlast_ms = tnow_ms;
    dc_hal_stats.transmitted_frame_count++;

    return 1;
}


//-------------------------------------------------------
// Receive
//-------------------------------------------------------
#ifndef DRONECAN_USE_RX_ISR
#error CAN polling not supported, DRONECAN_USE_RX_ISR must be defined !
#else
//-- ISR

// bxCAN RX element: id + dlc + 8 bytes data
typedef struct
{
    uint32_t id;     // extended CAN id (29 bits, no flags)
    uint8_t dlc;
    uint8_t data[CANARD_CAN_FRAME_MAX_DATA_LEN];
} tDcRxFifoElement;


#define DRONECAN_RXFRAMEBUFSIZEMASK  (DRONECAN_RXFRAMEBUFSIZE - 1)


volatile tDcRxFifoElement dronecan_rxbuf[DRONECAN_RXFRAMEBUFSIZE];
volatile uint16_t dronecan_rxwritepos; // pos at which the last frame was stored
volatile uint16_t dronecan_rxreadpos; // pos at which the next frame is to be fetched


// read one frame from the bxCAN hardware FIFO and push into the ring buffer
static void _dc_hal_receive_from_fifo(uint32_t rx_fifo)
{
    CAN_RxHeaderTypeDef pRxHeader;
    uint8_t data[CANARD_CAN_FRAME_MAX_DATA_LEN];

    HAL_StatusTypeDef hres = HAL_CAN_GetRxMessage(&hcan, rx_fifo, &pRxHeader, data);
    if (hres != HAL_OK) {
        return;
    }

    // DroneCAN uses only EXT frames, so these should be errors
    if (pRxHeader.IDE != CAN_ID_EXT) {
        dc_hal_stats.isr_xtd_count++;
        return;
    }
    if (pRxHeader.RTR == CAN_RTR_REMOTE) {
        dc_hal_stats.isr_rtr_count++;
        return;
    }

    uint16_t next = (dronecan_rxwritepos + 1) & DRONECAN_RXFRAMEBUFSIZEMASK;
    if (dronecan_rxreadpos != next) { // fifo not full
        dronecan_rxwritepos = next;

        dronecan_rxbuf[next].id = pRxHeader.ExtId;
        dronecan_rxbuf[next].dlc = pRxHeader.DLC;
        memcpy((void*)dronecan_rxbuf[next].data, data, CANARD_CAN_FRAME_MAX_DATA_LEN);

    } else {
        dc_hal_stats.rx_overflow_count++; // rx frame buffer overflow
    }
}


// ISR handler shared by FIFO0 and FIFO1
static void _dc_hal_isr_handler(void)
{
    // drain FIFO0
    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
        _dc_hal_receive_from_fifo(CAN_RX_FIFO0);
    }

    // drain FIFO1
    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO1) > 0) {
        _dc_hal_receive_from_fifo(CAN_RX_FIFO1);
    }
}


// bxCAN RX FIFO0 interrupt — shared with USB LP on STM32F103
// is already C context, not C++ !
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    _dc_hal_isr_handler();
}


// bxCAN RX FIFO1 interrupt
void CAN1_RX1_IRQHandler(void)
{
    _dc_hal_isr_handler();
}


//-- API

int16_t dc_hal_enable_isr(void)
{
    dronecan_rxwritepos = 0;
    dronecan_rxreadpos = 0;
    memset(&dc_hal_stats, 0, sizeof(dc_hal_stats));

    // enable FIFO0 and FIFO1 message pending interrupts
    HAL_StatusTypeDef hres = HAL_CAN_ActivateNotification(
        &hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
    if (hres != HAL_OK) { return -DC_HAL_ERROR_ISR_CONFIG; }

    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, DRONECAN_IRQ_PRIORITY);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    NVIC_SetPriority(CAN1_RX1_IRQn, DRONECAN_IRQ_PRIORITY);
    NVIC_EnableIRQ(CAN1_RX1_IRQn);

    return 0;
}


int16_t dc_hal_receive(CanardCANFrame* const frame)
{
    if (frame == NULL) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    _process_error_status();

    if (dronecan_rxwritepos == dronecan_rxreadpos) {
        return 0; // fifo empty
    }

    uint16_t rxreadpos = (dronecan_rxreadpos + 1) & DRONECAN_RXFRAMEBUFSIZEMASK;
    dronecan_rxreadpos = rxreadpos;

    frame->id = (dronecan_rxbuf[rxreadpos].id & CANARD_CAN_EXT_ID_MASK);
    frame->id |= CANARD_CAN_FRAME_EFF;

    frame->data_len = dronecan_rxbuf[rxreadpos].dlc;
    if (frame->data_len > CANARD_CAN_FRAME_MAX_DATA_LEN) frame->data_len = CANARD_CAN_FRAME_MAX_DATA_LEN; // should not happen, but play it safe

    // copy data bytes, and zero-fill
    for (uint8_t n = 0; n < CANARD_CAN_FRAME_MAX_DATA_LEN; n++) {
        frame->data[n] = (n < frame->data_len) ? dronecan_rxbuf[rxreadpos].data[n] : 0;
    }

    frame->iface_id = 0;

    dc_hal_stats.received_frame_count++;

    return 1;
}


void dc_hal_rx_flush(void)
{
    dronecan_rxwritepos = 0;
    dronecan_rxreadpos = 0;
    dc_hal_stats.rx_overflow_count = 0;
}


#endif // DRONECAN_USE_RX_ISR


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
    _process_error_status(); // to ensure it is latest
    dc_hal_stats.error_sum_count =
        dc_hal_stats.bo_count + dc_hal_stats.lec_count +
        dc_hal_stats.pxe_count + dc_hal_stats.cel_count;
#ifdef DRONECAN_USE_RX_ISR
    dc_hal_stats.error_sum_count += dc_hal_stats.rx_overflow_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_xtd_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_rtr_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_fdf_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_brs_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_dlc_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_rf0l_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_rf0f_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_rf1l_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_rf1f_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_errors_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_errorstatus_count;
#endif
    dc_hal_stats.error_sum_count += dc_hal_stats.tffl_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.tfqf_count;
    return dc_hal_stats;
}


// decode LEC from ESR — same 0–7 encoding as FDCAN PSR.LEC
const char* dc_hal_psr_lec_to_str(uint32_t psr)
{
    uint32_t lec = (psr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;
    switch (lec) {
        case 0: return "ok";
        case 1: return "STUFF";
        case 2: return "FORM";
        case 3: return "ACK";
        case 4: return "BIT1";
        case 5: return "BIT0";
        case 6: return "CRC";
        case 7: return "NC"; // no change
        default: return "??";
    }
}


// bxCAN has no Activity field — return fixed string
const char* dc_hal_psr_act_to_str(uint32_t psr)
{
    (void)psr;
    return "n/a";
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
    timings->bit_rate_prescaler = 4; // -> tq = 36/4 = 9
    timings->bit_segment_1 = 7;
    timings->bit_segment_2 = 1; // -> SP = (1 + BS1)/(1 + BS1 + BS2) = 8/9 = 88.89%
    timings->sync_jump_width = 1;

    // sanity check
    const uint32_t bit_time = (1 + timings->bit_segment_1 + timings->bit_segment_2) * timings->bit_rate_prescaler;
    const uint32_t f_clk_MHz = peripheral_clock_rate / 1000000;
    if (bit_time != f_clk_MHz) {
        return -DC_HAL_ERROR_TIMING;
    }

    return 0;
}


#endif // HAL_CAN_MODULE_ENABLED
#endif // STM32F103xB
