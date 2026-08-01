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

//XX#error The F1 CAN driver is likely out of date and needs revisting !

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

static uint8_t abort_tx_on_error;

static CAN_HandleTypeDef hcan;


//-------------------------------------------------------
// Helper
//-------------------------------------------------------

static void _process_error_status(void)
{
    uint32_t esr = READ_REG(hcan.Instance->ESR);

    if ((esr & CAN_ESR_LEC) != 0) {
        CLEAR_BIT(hcan.Instance->ESR, CAN_ESR_LEC);
        dc_hal_stats.error_sum_count++;

        if (abort_tx_on_error || ((esr & CAN_ESR_BOFF) != 0)) {
            //HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
            SET_BIT(hcan.Instance->TSR, CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2);
        }
    }
}


//-------------------------------------------------------
// Init
//-------------------------------------------------------

int16_t dc_hal_init(
    DC_HAL_CAN_ENUM can_instance, // irrelevant for STM32F1
    const tDcHalCanTimings* const timings,
    const tDcHalCanDataTimings* const data_timings, // irrelevant for STM32F1
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
    abort_tx_on_error = (iface_mode == DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);

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
    if (hres != HAL_OK) {
        return -DC_HAL_ERROR_CAN_INIT;
    }

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


uint8_t dc_hal_is_canfd(void)
{ 
    return 0; // bxCAN is classic CAN only
}


//-------------------------------------------------------
// Transmit
//-------------------------------------------------------

int16_t dc_hal_transmit(const CanardCANFrame* const frame, uint32_t tnow_ms) // tnow_ms irrelevant for STM32F1
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
    if (hres != HAL_OK) {
        return -DC_HAL_ERROR_CAN_ADD_TX_MESSAGE;
    }

    return 1;
}


//-------------------------------------------------------
// Receive
//-------------------------------------------------------
#ifndef DRONECAN_USE_RX_ISR
//-- Polling

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

            frame->id = pRxHeader.ExtId; // HAL_CAN_GetRxMessage() ensures that it is in range, no masking needed
            frame->id |= CANARD_CAN_FRAME_EFF; // libcanard wants the CANARD_CAN_FRAME_EFF flag be set

            frame->data_len = pRxHeader.DLC;
            frame->iface_id = 0;
            frame->canfd = 0; // bxCAN is classic CAN only

            return 1;
        }
    }

    return 0;
}

#else
//-- ISR

typedef struct
{
    uint32_t rir;
    uint32_t rdtr;
    union {
        uint8_t data[CANARD_CAN_FRAME_MAX_DATA_LEN];
        uint32_t data_32[CANARD_CAN_FRAME_MAX_DATA_LEN / 4]; // CANARD_CAN_FRAME_MAX_DATA_LEN should be divisible by 4
    };
} tDcRxFifoElement;


typedef enum // see RM0008, CAN receive FIFO mailbox identifier register & data length control register
{
    DC_RX_FIFO_RIR_RTR_BIT    = 0x00000002U, // remote transmission request, bit 1 (RTR), equals CAN_RTR_REMOTE, CAN_RIxR_RTR
    DC_RX_FIFO_RIR_XTD_BIT    = 0x40000004U, // extended identifier, bit 2 (IDE), equals CAN_ID_EXT, CAN_RIxR_IDE
    DC_RX_FIFO_RIR_EXTID_MASK = 0xFFFFFFF8U, // remote transmission request, bits 3-31
    DC_RX_FIFO_RDTR_DLC_MASK  = 0x0000000FU, // data length code, bits 0-3, equals CAN_RDTxR_DLC
} DC_RX_FIFO_ELEMENT_ENUM;


#define DRONECAN_RXFRAMEBUFSIZEMASK  (DRONECAN_RXFRAMEBUFSIZE - 1)


volatile tDcRxFifoElement dronecan_rxbuf[DRONECAN_RXFRAMEBUFSIZE];
volatile uint16_t dronecan_rxwritepos; // pos at which the last frame was stored
volatile uint16_t dronecan_rxreadpos; // pos at which the next frame is to be fetched


void _dc_hal_receive_isr(uint32_t rxfifo)
{
    CAN_FIFOMailBox_TypeDef* rx = &hcan.Instance->sFIFOMailBox[rxfifo];

    uint32_t rir = rx->RIR;
    uint32_t rdtr = rx->RDTR;

    if ((rir & DC_RX_FIFO_RIR_XTD_BIT) == 0) { // DroneCAN uses only EXT frames, so this should be an error
        dc_hal_stats.isr_xtd_count++;
        return;
    }
    if ((rir & DC_RX_FIFO_RIR_RTR_BIT) != 0) { // DroneCAN does not use RTR frames, so this should be an error
        dc_hal_stats.isr_rtr_count++;
        return;
    }

    // reject frames with DLC > 8
    if ((rdtr & DC_RX_FIFO_RDTR_DLC_MASK) > 8) { // shouldn't it be also > 0, as 0 is remote frame??
        dc_hal_stats.isr_dlc_count++;
        return;
    }

    uint16_t next = (dronecan_rxwritepos + 1) & DRONECAN_RXFRAMEBUFSIZEMASK;
    if (dronecan_rxreadpos != next) { //fifo not full
        dronecan_rxwritepos = next;

        dronecan_rxbuf[next].rir = rir;
        dronecan_rxbuf[next].rdtr = rdtr;
        dronecan_rxbuf[next].data_32[0] = rx->RDLR;
        dronecan_rxbuf[next].data_32[1] = rx->RDHR;

        //old: if (BXCAN->RFxR[0] & CANARD_STM32_CAN_RFR_FOVR) dc_hal_stats.rx_overflow_count++; // rx frame buffer overflow
        //?? if (READ_BIT(CAN1->RF0R, CAN_RF0R_FOVR0)) dc_hal_stats.rx_overflow_count++; // rx frame buffer overflow

        // fill can reach DRONECAN_RXFRAMEBUFSIZE - 1 at most, one slot is always kept free
        uint16_t fill = (next - dronecan_rxreadpos) & DRONECAN_RXFRAMEBUFSIZEMASK;
        if (fill > dc_hal_stats.rx_fifo_peak) dc_hal_stats.rx_fifo_peak = fill;

    } else {
        dc_hal_stats.rx_overflow_count++; // rx frame buffer overflow
        dc_hal_stats.rx_fifo_peak = DRONECAN_RXFRAMEBUFSIZE; // full
    }
}


// is already C context, not C++ !
void USB_LP_CAN1_RX0_IRQHandler(void)
{
// see HAL_CAN_IRQHandler()
// user should call HAL_StatusTypeDef HAL_CAN_GetRxMessage() in callback
    if ((hcan.Instance->IER & CAN_IT_RX_FIFO0_MSG_PENDING) != 0) { // ISR is enabled
        if ((hcan.Instance->RF0R & CAN_RF0R_FMP0) != 0) { // message still pending, Rx FIFO 0 not empty
            _dc_hal_receive_isr(CAN_RX_FIFO0);
        }
    }

    if ((hcan.Instance->IER & CAN_IT_RX_FIFO0_FULL) != 0 && (hcan.Instance->RF0R & CAN_RF0R_FULL0) != 0) {
        dc_hal_stats.isr_rf0f_count++; // RF0F, Rx Fifo 0 Full
    }
    if ((hcan.Instance->IER & CAN_IT_RX_FIFO0_OVERRUN) != 0 && (hcan.Instance->RF0R & CAN_RF0R_FOVR0) != 0) {
        dc_hal_stats.isr_rf0l_count++; // RF0L, Rx Fifo 0 Message Lost
    }

    // release FIFO entry we just read
    SET_BIT(CAN1->RF0R, CAN_RF0R_RFOM0 | CAN_RF0R_FOVR0 | CAN_RF0R_FULL0);

    // TODO: handle errors, BUSOFF etc
}


void CAN1_RX1_IRQHandler(void)
{
    if ((hcan.Instance->IER & CAN_IT_RX_FIFO1_MSG_PENDING) != 0) { // ISR is enabled
        if ((hcan.Instance->RF1R & CAN_RF1R_FMP1) != 0) { // message still pending, Rx FIFO 1 not empty
            _dc_hal_receive_isr(CAN_RX_FIFO1);
        }
    }

    if ((hcan.Instance->IER & CAN_IT_RX_FIFO1_FULL) != 0 && (hcan.Instance->RF1R & CAN_RF1R_FULL1) != 0) {
        dc_hal_stats.isr_rf1f_count++; // RF1F, Rx Fifo 1 Full
    }
    if ((hcan.Instance->IER & CAN_IT_RX_FIFO1_OVERRUN) != 0 && (hcan.Instance->RF1R & CAN_RF1R_FOVR1) != 0) {
        dc_hal_stats.isr_rf1l_count++; // RF1L, Rx Fifo 1 Message Lost
    }

    // release FIFO entry we just read
    SET_BIT(CAN1->RF1R, CAN_RF1R_RFOM1 | CAN_RF1R_FOVR1 | CAN_RF1R_FULL1);

    // TODO: handle errors, BUSOFF etc
}


//-- API

int16_t dc_hal_enable_isr(void)
{
HAL_StatusTypeDef hres;

    dronecan_rxwritepos = 0;
    dronecan_rxreadpos = 0;
    memset(&dc_hal_stats, 0, sizeof(dc_hal_stats));

    hres = HAL_CAN_ActivateNotification(
        &hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL |
        CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL |
        CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN
//        CAN_IT_BUSOFF
//        CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_ERROR
//        CAN_IT_LAST_ERROR_CODE
        );
    if (hres != HAL_OK) {
        return -DC_HAL_ERROR_ISR_CONFIG;
    }

    __HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_FF0 | CAN_FLAG_FOV0); // clear RxFIFO0 flags
    __HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_FF1 | CAN_FLAG_FOV1); // clear RxFIFO1 flags

    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, DRONECAN_IRQ_PRIORITY);
    NVIC_SetPriority(CAN1_RX1_IRQn, DRONECAN_IRQ_PRIORITY);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
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

    frame->id = (dronecan_rxbuf[rxreadpos].rir & DC_RX_FIFO_RIR_EXTID_MASK) >> 3;
    frame->id |= CANARD_CAN_FRAME_EFF; // libcanard wants the CANARD_CAN_FRAME_EFF flag be set

    // convert DLC to actual byte count
    uint32_t dlc = (dronecan_rxbuf[rxreadpos].rdtr & DC_RX_FIFO_RDTR_DLC_MASK) >> 0;
    frame->data_len = dlc;
    if (frame->data_len > CANARD_CAN_FRAME_MAX_DATA_LEN) frame->data_len = CANARD_CAN_FRAME_MAX_DATA_LEN; // should not happen, but play it safe

    // copy data bytes, and zero-fill
    for (uint8_t n = 0; n < CANARD_CANFD_FRAME_MAX_DATA_LEN; n++) {
        frame->data[n] = (n < frame->data_len) ? dronecan_rxbuf[rxreadpos].data[n] : 0;
    }

    frame->iface_id = 0;
    frame->canfd = 0; // bxCAN is classic CAN only

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

        // in the STM32F103 the CAN id's are stored left aligned
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
    timings->bit_rate_prescaler = 4; // -> tq = 36/4 = 9
    timings->bit_segment_1 = 7;
    timings->bit_segment_2 = 1; // -> SP = (1 + BS1)/(1 + BS1 + BS2) = 8/9 = 88.89%
    timings->sync_jump_width = 1;

    return 0;
}


int16_t dc_hal_compute_data_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_data_bit_rate,
    tDcHalCanDataTimings* const data_timings)
{
    return -DC_HAL_ERROR_TIMING;
}


#endif // HAL_PCD_MODULE_ENABLED
#endif // STM32F1
