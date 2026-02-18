//*******************************************************
// STM32 DroneCAN Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// DroneCAN Driver Library for STM32 using HAL
// for use with libcanard
//*******************************************************
#if defined STM32G431xx ||defined STM32G441xx || defined STM32G491xx || defined STM32G474xx

#include "stm32g4xx_hal.h"

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "stm32-dronecan-driver.h"
#include <string.h>


#define DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX  8 // must be equal to SRAMCAN_FLE_NBR, for FDCAN_EXTENDED_ID

// we can use IS_FDCAN_NOMINAL_PRESCALER(), IS_FDCAN_NOMINAL_TSEG1(), IS_FDCAN_NOMINAL_TSEG2(), IS_FDCAN_NOMINAL_SJW() !
#define DC_HAL_PRESCALER_MIN        1
#define DC_HAL_PRESCALER_MAX        512

#define DC_HAL_NOMINAL_BS1_MIN      1     // datasheet: bit_time must be 4 .. 81 tq
#define DC_HAL_NOMINAL_BS1_MAX      256
#define DC_HAL_NOMINAL_BS2_MIN      1
#define DC_HAL_NOMINAL_BS2_MAX      128
#define DC_HAL_NOMINAL_SJW_MIN      1
#define DC_HAL_NOMINAL_SJW_MAX      128

#define DC_HAL_DATA_PRESCALER_MIN   1
#define DC_HAL_DATA_PRESCALER_MAX   32
#define DC_HAL_DATA_BS1_MIN         1
#define DC_HAL_DATA_BS1_MAX         32
#define DC_HAL_DATA_BS2_MIN         1
#define DC_HAL_DATA_BS2_MAX         16
#define DC_HAL_DATA_SJW_MIN         1
#define DC_HAL_DATA_SJW_MAX         16    // datasheet: must always be smaller than BS2


static tDcHalStatistics dc_hal_stats = {};

static bool dc_hal_abort_tx_on_error;

static FDCAN_HandleTypeDef hfdcan;

static uint32_t tx_tlast_ms = 0; // for TX timeout error counting in dc_hal_transmit()
static uint8_t was_bo = 0; // for counting bus off errors only when state changes


//-------------------------------------------------------
// Helper
//-------------------------------------------------------

static void _process_error_status(void)
{
//    FDCAN_ProtocolStatusTypeDef protocolStatus;
//    HAL_FDCAN_GetProtocolStatus(&hfdcan, &protocolStatus);
//    if (protocolStatus.BusOff) {
//        CLEAR_BIT(hfdcan.Instance->CCCR, FDCAN_CCCR_INIT);
//    }
    uint32_t psr = READ_REG(hfdcan.Instance->PSR); // read the protocol status register

    if ((psr & FDCAN_PSR_BO) != 0) { // is bus off
        CLEAR_BIT(hfdcan.Instance->CCCR, FDCAN_CCCR_INIT);
        HAL_FDCAN_AbortTxRequest(&hfdcan, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
        if (!was_bo) dc_hal_stats.bo_count++; // BO, bus off, only count when toggled from bus on to bus off
        was_bo = 1;
    } else {
        was_bo = 0;
    }

    uint32_t lec = (psr & FDCAN_PSR_LEC) >> FDCAN_PSR_LEC_Pos; // Last Error Code
    uint32_t dlec = (psr & FDCAN_PSR_DLEC) >> FDCAN_PSR_DLEC_Pos; // Data Last Error Code
    if ((lec != FDCAN_PROTOCOL_ERROR_NONE && lec != FDCAN_PROTOCOL_ERROR_NO_CHANGE) ||
        (dlec != FDCAN_PROTOCOL_ERROR_NONE && dlec != FDCAN_PROTOCOL_ERROR_NO_CHANGE)) {
        CLEAR_BIT(hfdcan.Instance->PSR, FDCAN_PSR_LEC | FDCAN_PSR_DLEC);
        if (dc_hal_abort_tx_on_error) {
            HAL_FDCAN_AbortTxRequest(&hfdcan, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
        }
        dc_hal_stats.lec_count++; // LEC, Last Error Code
    }

    if ((psr & FDCAN_PSR_PXE) != 0) { // protocol exception event occurred, happens when fc is not yet doing CAN
        CLEAR_BIT(hfdcan.Instance->PSR, FDCAN_PSR_PXE);
        dc_hal_stats.pxe_count++; // PXE, Protocol Exception Event
    }

    uint32_t ecr = READ_REG(hfdcan.Instance->ECR); // read the error counter register, read clears CEL
    dc_hal_stats.tec_count = (ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos; // TEC, Transmit Error Counter
    dc_hal_stats.rec_count = (ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos; // REC, Receive Error Counter
    if ((ecr & FDCAN_ECR_CEL) != 0) { // Can Error Logging
        dc_hal_stats.cel_count += (ecr & FDCAN_ECR_CEL) >> FDCAN_ECR_CEL_Pos; // CEL, Can Error Logging
    }

    // record some more
    if (lec != FDCAN_PROTOCOL_ERROR_NO_CHANGE) dc_hal_stats.last_lec = lec;
    dc_hal_stats.last_psr = psr;
    dc_hal_stats.last_ecr = ecr;
    dc_hal_stats.last_cccr = READ_REG(hfdcan.Instance->CCCR);
}


// convert DLC field to data length in bytes
uint8_t _data_len_from_dlc(uint8_t dlc)
{
    return (dlc <= 8) ? dlc : 8;
}


// convert data length in bytes to DLC field
uint8_t _dlc_from_data_len(uint8_t data_len)
{
    return (data_len <= 8) ? data_len : 8;
}


//-------------------------------------------------------
// Init
//-------------------------------------------------------

int16_t dc_hal_init(
    DC_HAL_CAN_ENUM can_instance,
    const tDcHalCanTimings* const timings,
    const DC_HAL_IFACE_MODE_ENUM iface_mode)
{
    if ((iface_mode != DC_HAL_IFACE_MODE_NORMAL) &&
        (iface_mode != DC_HAL_IFACE_MODE_SILENT) &&
        (iface_mode != DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    if ((timings == NULL) ||
        !IS_FDCAN_NOMINAL_PRESCALER(timings->bit_rate_prescaler) ||
        !IS_FDCAN_NOMINAL_TSEG1(timings->bit_segment_1) ||
        !IS_FDCAN_NOMINAL_TSEG2(timings->bit_segment_2) ||
        !IS_FDCAN_NOMINAL_SJW(timings->sync_jump_width)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    memset(&dc_hal_stats, 0, sizeof(dc_hal_stats));
    dc_hal_abort_tx_on_error = (iface_mode == DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);

    switch (can_instance) { // 1, 2, ...
#ifdef FDCAN2
        case DC_HAL_CAN2: hfdcan.Instance = FDCAN2; break;
#endif
        default: // DC_HAL_CAN1
            hfdcan.Instance = FDCAN1;
    }
    __HAL_FDCAN_DISABLE_IT(&hfdcan, 0);

    hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan.Init.Mode = FDCAN_MODE_NORMAL;

    hfdcan.Init.AutoRetransmission = DISABLE;
    hfdcan.Init.TransmitPause = ENABLE; // insert pause between TX frames to reduce bus contention, probably a good thing to enable it
    hfdcan.Init.ProtocolException = DISABLE; // ST examples: treat exceptions as form errors instead of special state

    hfdcan.Init.NominalPrescaler = timings->bit_rate_prescaler;
    hfdcan.Init.NominalTimeSeg1 = timings->bit_segment_1;
    hfdcan.Init.NominalTimeSeg2 = timings->bit_segment_2;
    hfdcan.Init.NominalSyncJumpWidth = timings->sync_jump_width;

    hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC; // FDCAN_FRAME_FD_NO_BRS, FDCAN_FRAME_FD_BRS
    hfdcan.Init.DataPrescaler = 1; // irrelevant if FrameFormat != FDCAN_FRAME_FD_BRS
    hfdcan.Init.DataTimeSeg1 = 1;
    hfdcan.Init.DataTimeSeg2 = 1;
    hfdcan.Init.DataSyncJumpWidth = 1;

    hfdcan.Init.StdFiltersNbr = 0; // these are used in FDCAN_CalcultateRamBlockAddresses() called in HAL_FDCAN_Init()
    hfdcan.Init.ExtFiltersNbr = DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX;

    hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION; // FDCAN_TX_QUEUE_OPERATION

    HAL_StatusTypeDef hres = HAL_FDCAN_Init(&hfdcan);
    if (hres != HAL_OK) {
        return -DC_HAL_ERROR_CAN_INIT;
    }

    // configure reception filter
    //   follow licanard's default filter setup in spirit
    //   here it is really needed since the FDCAN RAM was already set up for max num filters
    //   so we fill it with some default
    //   HAL_FDCAN_ConfigFilter() sets the filter registers even when it is disabled
    // at least one filter must be enabled for receive to work
    FDCAN_FilterTypeDef sFilterConfig = {};
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK; // classic filter: FilterID1 = id, FilterID2 = mask
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // FDCAN_FILTER_DISABLE, FDCAN_FILTER_TO_RXFIFO1
    sFilterConfig.FilterID1 = 0;
    sFilterConfig.FilterID2 = 0;

    for (uint8_t n = 0; n < DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX; n++) {
        //sFilterConfig.FilterConfig = ((n & 0x01) == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
        sFilterConfig.FilterConfig = (n == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_DISABLE;
        sFilterConfig.FilterIndex = n;
        hres = HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);
        if (hres != HAL_OK) {
            return -DC_HAL_ERROR_CAN_CONFIG_FILTER;
        }
    }

    // configure global filter
    //  reject non matching frames with STD and EXT ID
    //  filter all remote frames with STD and EXT ID
    hres = HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    if (hres != HAL_OK) {
        return -DC_HAL_ERROR_CAN_CONFIG_GLOBAL_FILTER;
    }

    // FDCAN_RX_FIFO_BLOCKING, FDCAN_RX_FIFO_OVERWRITE ???
    //hres = HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    //hres = HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_OVERWRITE);

    //HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan, 3, 0);
    //HAL_FDCAN_EnableTxDelayCompensation(&hfdcan);

    // CCE bit in FDCAN_CCCR register is automatically cleared when INIT bit in FDCAN_CCCR is cleared.
    //CLEAR_BIT(hfdcan.Instance->CCCR, FDCAN_CCCR_CCE);

    return 0;
}


int16_t dc_hal_start(void)
{
    HAL_StatusTypeDef hres = HAL_FDCAN_Start(&hfdcan);
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

    if (frame->data_len > 8) { // don't do FD
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }

    _process_error_status();

    // thx to the TxFiFo in the G4 we can do the crude method and just put the message into the fifo if there is space
    // check for space in fifo
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan) == 0) {
        if (tx_tlast_ms > 0 && (tnow_ms - tx_tlast_ms) > 10) {
            dc_hal_stats.tffl_count++; // TFFL, Tx Fifo Free Level
        }
        return 0; // no space, postpone
    }
    // this check is done in HAL_FDCAN_AddMessageToTxFifoQ(), so better do it here too
    if ((hfdcan.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
        if (tx_tlast_ms > 0 && (tnow_ms - tx_tlast_ms) > 10) {
            dc_hal_stats.tfqf_count++; // TFQF, Tx Fifo Queue Full
        }
        return 0; // tx fifo queue full, postpone
    }

    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // FDCAN_ESI_ACTIVE, FDCAN_ESI_PASSIVE ???
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF; // FDCAN_BRS_ON
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN; // FDCAN_FD_CAN
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // FDCAN_STORE_TX_EVENTS
    pTxHeader.MessageMarker = 0; // this parameter must be a number between 0 and 0xFF

    pTxHeader.Identifier = (frame->id & CANARD_CAN_EXT_ID_MASK);
    pTxHeader.IdType = FDCAN_EXTENDED_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;

    pTxHeader.DataLength = (uint32_t)_dlc_from_data_len(frame->data_len) << 16;

    HAL_StatusTypeDef hres = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &pTxHeader, frame->data);
    if (hres != HAL_OK) {
        return -DC_HAL_ERROR_CAN_ADD_TX_MESSAGE;
    }

    tx_tlast_ms = tnow_ms;
    dc_hal_stats.transmitted_frame_count++;

    return 1;
}


//-------------------------------------------------------
// Receive
//-------------------------------------------------------
#ifndef DRONECAN_USE_RX_ISR
//-- Polling
#error CAN polling not supported, DRONECAN_USE_RX_ISR must be defined !
#else
//-- ISR

#define DC_FDCAN_RX_FIFO_ELEMENT_SIZE  (18U * 4U) // Rx FIFO 0/1 element size in bytes, 2 words + 64 bytes = 18*4


typedef struct
{
    uint32_t r0;
    uint32_t r1;
    union {
        uint8_t data[CANARD_CAN_FRAME_MAX_DATA_LEN];
        uint32_t data_32[CANARD_CAN_FRAME_MAX_DATA_LEN / 4]; // CANARD_CAN_FRAME_MAX_DATA_LEN should be divisible by 4
    };
} tDcRxFifoElement;


typedef enum // see table 400 in datasheet
{
    DC_RX_FIFO_R0_XTD_BIT   = 0x40000000U, // extended identifier
    DC_RX_FIFO_R0_RTR_BIT   = 0x20000000U, // remote transmission request
    DC_RX_FIFO_R1_FDF_BIT   = 0x00200000U, // FD format
    DC_RX_FIFO_R1_BRS_BIT   = 0x00100000U, // bit rate switch
    DC_RX_FIFO_R1_DLC_MASK  = 0x000F0000U, // data length code
} DC_RX_FIFO_ELEMENT_ENUM;


#define DRONECAN_RXFRAMEBUFSIZEMASK  (DRONECAN_RXFRAMEBUFSIZE - 1)


volatile tDcRxFifoElement dronecan_rxbuf[DRONECAN_RXFRAMEBUFSIZE];
volatile uint16_t dronecan_rxwritepos; // pos at which the last frame was stored
volatile uint16_t dronecan_rxreadpos; // pos at which the next frame is to be fetched


void _dc_hal_receive_isr(uint32_t* RxAddress)
{
    uint32_t r0 = *RxAddress;
    if ((r0 & DC_RX_FIFO_R0_XTD_BIT) == 0) { // DroneCAN uses only EXT frames, so this should be an error
        dc_hal_stats.isr_xtd_count++;
        return;
    }
    if ((r0 & DC_RX_FIFO_R0_RTR_BIT) != 0) { // DroneCAN does not use RTR frames, so this should be an error
        dc_hal_stats.isr_rtr_count++;
        return;
    }

    RxAddress++;
    uint32_t r1 = *RxAddress;
    if ((r1 & DC_RX_FIFO_R1_FDF_BIT) != 0) {
        dc_hal_stats.isr_fdf_count++;
        return;
    }
    if ((r1 & DC_RX_FIFO_R1_BRS_BIT) != 0) {
        dc_hal_stats.isr_brs_count++;
        return;
    }

    // for classic CAN, reject frames with DLC > 8
    if ((r1 & DC_RX_FIFO_R1_DLC_MASK) > FDCAN_DLC_BYTES_8) {
        dc_hal_stats.isr_dlc_count++;
        return;
    }

    uint16_t next = (dronecan_rxwritepos + 1) & DRONECAN_RXFRAMEBUFSIZEMASK;
    if (dronecan_rxreadpos != next) { // fifo not full
        dronecan_rxwritepos = next;

        dronecan_rxbuf[next].r0 = r0;
        dronecan_rxbuf[next].r1 = r1;
        RxAddress++;
        // copy all data bytes based on actual DLC (G4 message RAM supports 64 bytes)
        uint8_t dlc = (r1 & DC_RX_FIFO_R1_DLC_MASK) >> 16;
        uint8_t data_len = _data_len_from_dlc(dlc);
        uint8_t word_len = (data_len + 3) / 4; // round up to full words
        for (uint8_t i = 0; i < word_len; i++) {
            dronecan_rxbuf[next].data_32[i] = *RxAddress;
            RxAddress++;
        }

    } else {
        dc_hal_stats.rx_overflow_count++; // rx frame buffer overflow
    }
}


void _dc_hal_isr_handler(void)
{
    //HAL_FDCAN_IRQHandler(&hfdcan);
    // copy the part relevant to us
    // flags:
    //   FDCAN_IR_RF0L           // Rx FIFO 0 message lost
    //   FDCAN_IR_RF0F           // Rx FIFO 0 full
    //   FDCAN_IR_RF0N           // New message written to Rx FIFO 0
    // more descriptive defines are in the HAL, like FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE
    // #define FDCAN_RX_FIFO0_MASK (FDCAN_IR_RF0L | FDCAN_IR_RF0F | FDCAN_IR_RF0N)
    // there are also analogous defines
    //   FDCAN_IE_RF0LE          // Rx FIFO 0 message lost
    //   FDCAN_IE_RF0FE          // Rx FIFO 0 full
    //   FDCAN_IE_RF0NE          // New message written to Rx FIFO 0
    // for which there are also more descriptive defines in the HAL, like FDCAN_IT_RX_FIFO0_NEW_MESSAGE

    uint32_t RxFifo0ITs = hfdcan.Instance->IR & (FDCAN_IR_RF0N | FDCAN_IR_RF0F | FDCAN_IR_RF0L); // __HAL_FDCAN_GET_FLAG()
    RxFifo0ITs &= hfdcan.Instance->IE; // __HAL_FDCAN_GET_IT_SOURCE()
    uint32_t RxFifo1ITs = hfdcan.Instance->IR & (FDCAN_IR_RF1N | FDCAN_IR_RF1F | FDCAN_IR_RF1L);
    RxFifo1ITs &= hfdcan.Instance->IE;

    if (RxFifo0ITs != 0) {
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan, RxFifo0ITs); // clear Rx FIFO0 flags

        if ((RxFifo0ITs & (FDCAN_IR_RF0N | FDCAN_IR_RF0F)) != 0) { // do it also if Rx FIFO 0 full
            // HAL_FDCAN_GetRxMessage()
            while ((hfdcan.Instance->RXF0S & FDCAN_RXF0S_F0FL) != 0) { // Rx FIFO 0 not empty
                // calculate Rx FIFO 0 element address
                uint32_t GetIndex = ((hfdcan.Instance->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos);
                uint32_t* RxAddress = (uint32_t*)(hfdcan.msgRam.RxFIFO0SA + (GetIndex * DC_FDCAN_RX_FIFO_ELEMENT_SIZE));
                _dc_hal_receive_isr(RxAddress);
                // acknowledge the Rx FIFO 0 that the oldest element is read so that it increments the GetIndex
                hfdcan.Instance->RXF0A = GetIndex;
            }
        }

        if ((RxFifo0ITs & FDCAN_IR_RF0F) != 0) {
            dc_hal_stats.isr_rf0f_count++; // RF0F, Rx Fifo 0 Full
        }
        if ((RxFifo0ITs & FDCAN_IR_RF0L) != 0) {
            dc_hal_stats.isr_rf0l_count++; // RF0L, Rx Fifo 0 Message Lost
        }
    }

    if (RxFifo1ITs != 0) {
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan, RxFifo1ITs); // clear Rx FIFO1 flags

        if ((RxFifo1ITs & (FDCAN_IR_RF1N | FDCAN_IR_RF1F)) != 0) { // do it also if Rx FIFO 1 full
            while ((hfdcan.Instance->RXF1S & FDCAN_RXF1S_F1FL) != 0) { // Rx FIFO 1 not empty
                // calculate Rx FIFO 1 element address
                uint32_t GetIndex = ((hfdcan.Instance->RXF1S & FDCAN_RXF1S_F1GI) >> FDCAN_RXF1S_F1GI_Pos);
                uint32_t* RxAddress = (uint32_t*)(hfdcan.msgRam.RxFIFO1SA + (GetIndex * DC_FDCAN_RX_FIFO_ELEMENT_SIZE));
                _dc_hal_receive_isr(RxAddress);
                // acknowledge the Rx FIFO 1 that the oldest element is read so that it increments the GetIndex
                hfdcan.Instance->RXF1A = GetIndex;
            }
        }

        if ((RxFifo1ITs & FDCAN_IR_RF1F) != 0) {
            dc_hal_stats.isr_rf1f_count++; // RF1F, Rx Fifo 1 Full
        }
        if ((RxFifo1ITs & FDCAN_IR_RF1L) != 0) {
            dc_hal_stats.isr_rf1l_count++; // RF1L, Rx Fifo 1 Message Lost
        }
    }

    // EOL: Error Logging Overflow                -> Bit and Line Error
    // WDI: Watchdog Interrupt                    -> Protocol Error
    // PEA: Protocol Error in Arbitration Phase   -> Protocol Error
    // PED: Protocol Error in Data Phase          -> Protocol Error
    // ARA: Access to Reserved Address            -> Protocol Error
    uint32_t Errors = hfdcan.Instance->IR & (FDCAN_IR_ELO | FDCAN_IR_WDI | FDCAN_IR_PEA | FDCAN_IR_PED | FDCAN_IR_ARA);
    Errors &= hfdcan.Instance->IE;
    // EP: Error Passive                          -> Bit and Line Error
    // EW: Warning Status                         -> Protocol Error
    // BO: Bus_Off Status                         -> Protocol Error
    uint32_t ErrorStatusITs = hfdcan.Instance->IR & (FDCAN_IR_EP | FDCAN_IR_EW | FDCAN_IR_BO);
    ErrorStatusITs &= hfdcan.Instance->IE;

    if (Errors != 0) {
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan, Errors); // clear the Error flags
        dc_hal_stats.isr_errors_count++;
    }
    if (ErrorStatusITs != 0) {
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan, ErrorStatusITs); // clear the Error flags
        dc_hal_stats.isr_errorstatus_count++;
    }
}


// is already C context, not C++ !

#ifdef FDCAN2
void FDCAN2_IT0_IRQHandler(void)
{
    if (hfdcan.Instance == FDCAN2) _dc_hal_isr_handler();
}
#endif

void FDCAN1_IT0_IRQHandler(void)
{
    if (hfdcan.Instance == FDCAN1) _dc_hal_isr_handler();
}


//-- API

int16_t dc_hal_enable_isr(void)
{
HAL_StatusTypeDef hres;

    dronecan_rxwritepos = 0;
    dronecan_rxreadpos = 0;
    memset(&dc_hal_stats, 0, sizeof(dc_hal_stats));

// doc in stm32g4xx_hal_fdcan.c says: By default, all interrupts are assigned to line 0
// so we should not have to do this
//    hres = HAL_FDCAN_ConfigInterruptLines(
//        &hfdcan,
//        FDCAN_IT_GROUP_RX_FIFO0 | FDCAN_IT_GROUP_RX_FIFO1,
//        FDCAN_INTERRUPT_LINE0);
//    if (hres != HAL_OK) { return -DC_HAL_ERROR_ISR_CONFIG; }

    hres = HAL_FDCAN_ActivateNotification(
        &hfdcan,
//        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_BUS_OFF,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL |
        FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_FULL |
        FDCAN_IT_BUS_OFF,
//        FDCAN_IT_LIST_RX_FIFO0 | FDCAN_IT_LIST_RX_FIFO1 |
//        FDCAN_IT_LIST_BIT_LINE_ERROR | FDCAN_IT_LIST_PROTOCOL_ERROR,
        0);
    if (hres != HAL_OK) { return -DC_HAL_ERROR_ISR_CONFIG; }

    hfdcan.Instance->IR = 0xFFFFFFFF; // clear all flags by writing 1 to them

#ifdef FDCAN2
    if (hfdcan.Instance == FDCAN2) {
        NVIC_SetPriority(FDCAN2_IT0_IRQn, DRONECAN_IRQ_PRIORITY);
        NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    } else
#endif
    if (hfdcan.Instance == FDCAN1) {
        NVIC_SetPriority(FDCAN1_IT0_IRQn, DRONECAN_IRQ_PRIORITY);
        NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }

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

    frame->id = (dronecan_rxbuf[rxreadpos].r0 & CANARD_CAN_EXT_ID_MASK);
    frame->id |= CANARD_CAN_FRAME_EFF;

    // convert DLC to actual byte count
    uint32_t dlc = (dronecan_rxbuf[rxreadpos].r1 & DC_RX_FIFO_R1_DLC_MASK) >> 16;
    frame->data_len = _data_len_from_dlc(dlc);
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

    FDCAN_FilterTypeDef sFilterConfig = {};
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK; // classic filter: FilterID1 = id, FilterID2 = mask
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // FDCAN_FILTER_DISABLE, FDCAN_FILTER_TO_RXFIFO1
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterID1 = 0;
    sFilterConfig.FilterID2 = 0;

    for (uint8_t n = 0; n < num_filter_configs; n++) {
        if (filter_configs[n].rx_fifo == DC_HAL_RX_FIFO0) {
            sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        } else
        if (filter_configs[n].rx_fifo == DC_HAL_RX_FIFO1) {
            sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
        } else {
            sFilterConfig.FilterConfig = ((n & 0x01) == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
        }
        sFilterConfig.FilterIndex = n;
        sFilterConfig.FilterID1 = (filter_configs[n].id & CANARD_CAN_EXT_ID_MASK);
        sFilterConfig.FilterID2 = (filter_configs[n].mask & CANARD_CAN_EXT_ID_MASK);
        HAL_StatusTypeDef hres = HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);
        if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_CONFIG_FILTER; }
    }

    // fill remaining filters with default
    sFilterConfig.FilterID1 = 0;
    sFilterConfig.FilterID2 = 0;
    for (uint8_t n = num_filter_configs; n < DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX; n++) {
        //sFilterConfig.FilterConfig = ((n & 0x01) == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
        //sFilterConfig.FilterConfig = (n == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_DISABLE;
        sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
        sFilterConfig.FilterIndex = n;
        HAL_StatusTypeDef hres = HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);
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


const char* dc_hal_psr_lec_to_str(uint32_t psr)
{
    uint32_t lec = (psr & FDCAN_PSR_LEC) >> FDCAN_PSR_LEC_Pos; // Last Error Code
    switch (lec) {
        case 0: return "ok";
        case 1: return "STUFF";
        case 2: return "FORM";
        case 3: return "ACK";
        case 4: return "BIT1";
        case 5: return "BIT0";
        case 6: return "CRC";
        case 7: return "NC"; // No Change
        default: return "??";
    }
}


const char* dc_hal_psr_act_to_str(uint32_t psr)
{
    uint32_t act = (psr & FDCAN_PSR_ACT) >> FDCAN_PSR_ACT_Pos; // Activity
    switch (act) {
        case 0: return "Sync";  // Synchronizing
        case 1: return "Idle";  // Idle
        case 2: return "Rx";    // Receiver
        case 3: return "Tx";    // Transmitter
        default: return "??";
    }
}


int16_t dc_hal_compute_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_bit_rate,
    tDcHalCanTimings* const timings)
{
    if (target_bit_rate != 1000000) {
        return -DC_HAL_ERROR_UNSUPPORTED_BIT_RATE;
    }

    // general rule:
    // tq = peripheral_clock_rate / bit_rate / prescaler
    // BS1 = SP * tq - 1, where SP is e.g. 3/4 = 75% or 7/8 = 87.5%
    // BS2 = tq - 1 - BS1 = (1 - SP) * tq
    // -> SP = (1 + BS1)/(1 + BS1 + BS2)

    // Note: ChatGPT was very clear on that one should use 80 MHz clock for FDCAN
    // we used 170 MHz before for classic CAN, but 80 MHz is said to be just better

    // timings by ArduPilot (confirmed by ChatGPT, by JLP, by myself using AP's code explicitly)
    // 10 tq
    // prescaler 8
    // BS1 = 8
    // BS2 = 1
    // SJW = 1
    // -> SP = 90.0%
    // Note: this is somewhat weird. AP cites a source that says that 8 tq would be optimal,
    // which can be achieved with prescaler 10, BS1 = 6, BS2 = 1, SJW = 1, -> SP = 7/8 = 87.5%.
    // It also would give SP 87.5% which ChatGPT says is industry standard. ??
#if 1
    if (peripheral_clock_rate == 80000000) { // 80 MHz
        timings->bit_rate_prescaler = 8;
        timings->bit_segment_1 = 8;
        timings->bit_segment_2 = 1;
        timings->sync_jump_width = 1;
    } else if (peripheral_clock_rate == 160000000) { // 160 MHz // NOT PREFFRED, but not terrible
        timings->bit_rate_prescaler = 16;
        timings->bit_segment_1 = 8;
        timings->bit_segment_2 = 1;
        timings->sync_jump_width = 1;
    } else if (peripheral_clock_rate == 170000000) { // 170 MHz // NOT PREFFRED
        timings->bit_rate_prescaler = 17;
        timings->bit_segment_1 = 8;
        timings->bit_segment_2 = 1;
        timings->sync_jump_width = 1;
    } else {
        return -DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY;
    }
#endif

    // timings generated by phryniszak for 75%
    // legacy: this is what we used before with 170 MHz FDCAN clock
#if 0
    if (peripheral_clock_rate == 170000000) { // 170 MHz
        timings->bit_rate_prescaler = 1;
        timings->bit_segment_1 = 127;
        timings->bit_segment_2 = 42; // -> SP = 0.75294 %
        timings->sync_jump_width = 42;
    } else if (peripheral_clock_rate == 160000000) { // 160 MHz
        timings->bit_rate_prescaler = 1;
        timings->bit_segment_1 = 119;
        timings->bit_segment_2 = 40; // -> SP = 0.75 %
        timings->sync_jump_width = 40;
    } else if (peripheral_clock_rate == 80000000) { // 80 MHz
        timings->bit_rate_prescaler = 1;
        timings->bit_segment_1 = 59;
        timings->bit_segment_2 = 20; // -> SP = 0.75 %
        timings->sync_jump_width = 20;
    } else {
        return -DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY;
    }
#endif
    // timings generated by phryniszak for 87.5%
#if 0
    if (peripheral_clock_rate == 170000000) { // 170 MHz
        timings->bit_rate_prescaler = 1;
        timings->bit_segment_1 = 147;
        timings->bit_segment_2 = 22;
        timings->sync_jump_width = 21;
    } else {
        return -DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY;
    }
#endif

    // let's do a check
    // datasheet:
    //   tq = prescaler * 1/f_clk
    //   bit_time = (1 + BS1 + BS2) * tq
    // =>
    //   bit_time = (1 + BS1 + BS2) * prescaler * 1/f_clk
    // we want bit_time = 1/1000000
    // =>
    //   (1 + BS1 + BS2) * prescaler = f_clk / 1000000
    const uint32_t bit_time = (1 + timings->bit_segment_1 + timings->bit_segment_2) * timings->bit_rate_prescaler;
    const uint32_t f_clk_MHz = peripheral_clock_rate / 1000000;
    if (bit_time != f_clk_MHz) {
        return -DC_HAL_ERROR_TIMING;
    }

    return 0;
}


/*
https://phryniszak.github.io/stm32g-fdcan/
https://github.com/phryniszak/stm32g-fdcan?tab=readme-ov-file
nominal:  TSeg1 = 127,  Tseg2 = 42, SJW = 42, Prescale = 1     75%
data:     TSeg1 = 7,    Tseg2 = 2,  SJW = 2, Prescale = 17     80%

https://kvaser.com/support/calculators/can-fd-bit-timing-calculator/
170 MHz, 500 ppm, 100ns
1MB/s 1MB/s
for ca 75%
nominal:  TSeg1 = 85 + 42 = 127,  Tseg2 = 42, SJW = 43, Prescale = 1
data:     TSeg1 = 126,            Tseg2 = 43, SJW = 43, Prescale = 1

STM32G4 examples
https://github.com/STMicroelectronics/STM32CubeG4/tree/master/Projects/STM32G474E-EVAL/Examples/FDCAN

https://github.com/pierremolinaro/acanfd-stm32

https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/CANFDIface.cpp

https://github.com/am32-firmware/AM32/pull/36/files

https://github.com/ARMmbed/mbed-os/pull/13565/files
*/
/*
good source on CAN errors
https://www.csselectronics.com/pages/can-bus-errors-intro-tutorial
*/


#endif // HAL_PCD_MODULE_ENABLED
#endif // STM32G4
