//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP CAN (TWAI)
// DroneCAN driver for ESP32 using TWAI peripheral
// implements the dc_hal_* API for use with libcanard
//*******************************************************
#ifndef ESPLIB_CAN_H
#define ESPLIB_CAN_H
#ifdef __cplusplus
extern "C" {
#endif


#include "esp-peripherals.h"
#include "driver/twai.h"
#include "../../modules/stm32-dronecan-lib/libcanard/canard.h"

#if defined CONFIG_IDF_TARGET_ESP32C3 || defined CONFIG_IDF_TARGET_ESP32S3
#include "soc/usb_serial_jtag_reg.h"
#endif


//-------------------------------------------------------
// configuration
//-------------------------------------------------------

#define DRONECAN_RXFRAMEBUFSIZE   64     // ring buffer size for rx frames


//-------------------------------------------------------
// types (matching stm32-dronecan-driver.h API)
//-------------------------------------------------------

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
    DC_HAL_IFACE_MODE_NORMAL = 0,
    DC_HAL_IFACE_MODE_SILENT,
    DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR
} DC_HAL_IFACE_MODE_ENUM;


typedef struct
{
    uint32_t transmitted_frame_count;
    uint32_t received_frame_count;
    uint32_t rx_overflow_count;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t bus_off_count;
    uint32_t arb_lost_count;
    uint32_t error_sum_count;
    uint32_t tec_count;
    uint32_t rec_count;
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
    uint8_t rx_fifo;
} tDcHalAcceptanceFilterConfiguration;


typedef struct
{
    uint16_t bit_rate_prescaler;
    uint8_t bit_segment_1;
    uint8_t bit_segment_2;
    uint8_t sync_jump_width;
} tDcHalCanTimings;


typedef enum
{
    DC_HAL_CAN1 = 0,
    DC_HAL_CAN2,
} DC_HAL_CAN_ENUM;


//-------------------------------------------------------
// ring buffer
//-------------------------------------------------------

static volatile CanardCANFrame _dc_rxbuf[DRONECAN_RXFRAMEBUFSIZE];
static volatile uint16_t _dc_rxwritepos = 0;
static volatile uint16_t _dc_rxreadpos = 0;


//-------------------------------------------------------
// statistics
//-------------------------------------------------------

static tDcHalStatistics _dc_stats;


//-------------------------------------------------------
// software acceptance filters (up to 2 ID/mask pairs)
// mirrors what STM32 does in hardware
//-------------------------------------------------------

#define DC_RX_FILTER_MAX  2

static volatile uint32_t _dc_rx_filter_id[DC_RX_FILTER_MAX] = { 0, 0 };
static volatile uint32_t _dc_rx_filter_mask[DC_RX_FILTER_MAX] = { 0, 0 };
static volatile uint8_t  _dc_rx_filter_count = 0; // 0 = accept all

// call from application code to configure filters
void dc_hal_set_rx_filters(const tDcHalAcceptanceFilterConfiguration* const cfgs, uint8_t count)
{
    if (count > DC_RX_FILTER_MAX) count = DC_RX_FILTER_MAX;
    for (uint8_t i = 0; i < count; i++) {
        _dc_rx_filter_id[i] = cfgs[i].id;
        _dc_rx_filter_mask[i] = cfgs[i].mask;
    }
    _dc_rx_filter_count = count; // publish last (volatile)
}


static bool _dc_rx_task_running = false;

static void _dc_rx_task(void* parameter)
{
    twai_message_t rx_msg;

    while (true) {
        // yields until a frame arrives; 1s timeout enables bus-off detection
        esp_err_t err = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) {

            // only accept extended frames, skip RTR
            if (!(rx_msg.extd) || rx_msg.rtr) continue;
            if (rx_msg.data_length_code > 8) continue;

            // software acceptance filter — drop frames that don't match
            uint8_t nf = _dc_rx_filter_count;
            if (nf > 0) {
                uint32_t can_id = rx_msg.identifier;
                bool accepted = false;
                for (uint8_t i = 0; i < nf; i++) {
                    if ((can_id & _dc_rx_filter_mask[i]) == (_dc_rx_filter_id[i] & _dc_rx_filter_mask[i])) {
                        accepted = true;
                        break;
                    }
                }
                if (!accepted) continue;
            }

            uint16_t next_writepos = (_dc_rxwritepos + 1) % DRONECAN_RXFRAMEBUFSIZE;
            if (next_writepos == _dc_rxreadpos) {
                // ring buffer overflow
                _dc_stats.rx_overflow_count++;
                continue;
            }

            _dc_rxbuf[_dc_rxwritepos].id = (rx_msg.identifier & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
            _dc_rxbuf[_dc_rxwritepos].data_len = rx_msg.data_length_code;
            memcpy((void*)_dc_rxbuf[_dc_rxwritepos].data, rx_msg.data, rx_msg.data_length_code);

            __sync_synchronize(); // ensure data is visible before position update
            _dc_rxwritepos = next_writepos;

        } else {
            // on timeout, check for bus-off (TEC >= 256) and attempt recovery
            twai_status_info_t status;
            if (twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_BUS_OFF) {
                _dc_stats.bus_off_count++;
                twai_initiate_recovery();
                // wait for 128 × 11 recessive bit sequences to complete
                do {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    twai_get_status_info(&status);
                } while (status.state == TWAI_STATE_RECOVERING);
                twai_start();
            }
        }
    }
}


//-------------------------------------------------------
// CAN init
//-------------------------------------------------------
// mirrors the stdstm32-can.h pattern: install the TWAI driver during hw init

void can_init(void)
{
    memset(&_dc_stats, 0, sizeof(_dc_stats));
    _dc_rxwritepos = 0;
    _dc_rxreadpos = 0;

    // on ESP32-C3/S3, GPIO 18/19 are USB D-/D+ by default.
    // release them so the GPIO matrix can route TWAI to these pins.
    // this disables USB serial/JTAG.
#if defined CONFIG_IDF_TARGET_ESP32C3 || defined CONFIG_IDF_TARGET_ESP32S3
    CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
#endif

    // install TWAI driver here, matching the STM32 pattern where can_init()
    // sets up the CAN peripheral and calls dc_hal_init()
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_IO,
        (gpio_num_t)CAN_RX_IO,
        TWAI_MODE_NORMAL);
    g_config.rx_queue_len = DRONECAN_RXFRAMEBUFSIZE;
    g_config.tx_queue_len = 16;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) while (1) {} // fatal
}


//-------------------------------------------------------
// dc_hal_* API implementation
//-------------------------------------------------------

int16_t dc_hal_init(
    DC_HAL_CAN_ENUM can_instance,
    const tDcHalCanTimings* const timings,
    const DC_HAL_IFACE_MODE_ENUM iface_mode)
{
    // twai driver is already installed by can_init()
    (void)can_instance;
    (void)timings;
    (void)iface_mode;
    return 0;
}


int16_t dc_hal_start(void)
{
    esp_err_t err = twai_start();
    if (err != ESP_OK) return -DC_HAL_ERROR_CAN_START;
    return 0;
}


int16_t dc_hal_transmit(const CanardCANFrame* const frame, uint32_t tnow_ms)
{
    (void)tnow_ms;

    if (frame == NULL) return -DC_HAL_ERROR_INVALID_ARGUMENT;
    if (!(frame->id & CANARD_CAN_FRAME_EFF)) return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    if (frame->id & CANARD_CAN_FRAME_RTR) return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    if (frame->data_len > 8) return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;

    twai_message_t tx_msg;
    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.identifier = frame->id & CANARD_CAN_EXT_ID_MASK;
    tx_msg.extd = 1;
    tx_msg.data_length_code = frame->data_len;
    memcpy(tx_msg.data, (const void*)frame->data, frame->data_len);

    // non-blocking transmit with short timeout
    esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
    if (err != ESP_OK) {
        _dc_stats.tx_error_count++;
        return -DC_HAL_ERROR_CAN_ADD_TX_MESSAGE;
    }

    _dc_stats.transmitted_frame_count++;
    return 1;
}


int16_t dc_hal_receive(CanardCANFrame* const frame)
{
    if (_dc_rxwritepos == _dc_rxreadpos) {
        return 0; // ring buffer empty
    }

    frame->id = _dc_rxbuf[_dc_rxreadpos].id;
    frame->iface_id = 0;
    frame->data_len = _dc_rxbuf[_dc_rxreadpos].data_len;
    memcpy(frame->data, (const void*)_dc_rxbuf[_dc_rxreadpos].data, frame->data_len);

    _dc_rxreadpos = (_dc_rxreadpos + 1) % DRONECAN_RXFRAMEBUFSIZE;
    _dc_stats.received_frame_count++;
    return 1;
}


int16_t dc_hal_config_acceptance_filters(
    const tDcHalAcceptanceFilterConfiguration* const filter_configs,
    const uint8_t num_filter_configs)
{
    // ESP32 TWAI only supports a single hardware acceptance filter, so we
    // implement multi-filter support in software via dc_hal_set_rx_filters().
    dc_hal_set_rx_filters(filter_configs, num_filter_configs);
    return 0;
}


int16_t dc_hal_enable_isr(void)
{
    if (_dc_rx_task_running) return 0;

    // on dual-core ESP32, pin to Core 0 so mLRS radio loop on Core 1 is unaffected.
    // on single-core ESP32-C3, pinning is a no-op but priority 5 ensures
    // frames are serviced promptly by preempting the Arduino loop (priority 1).
    // each wake takes ~10 us per frame (context switch + copy to ring buffer).
    BaseType_t ret = xTaskCreatePinnedToCore(
        _dc_rx_task,
        "DcRx",
        2048,
        NULL,
        5,    // priority: above idle, below radio-critical
        NULL,
        0);   // Core 0
    if (ret != pdPASS) return -DC_HAL_ERROR_ISR_CONFIG;
    _dc_rx_task_running = true;
    return 0;
}


void dc_hal_rx_flush(void)
{
    _dc_rxwritepos = 0;
    _dc_rxreadpos = 0;
}


tDcHalStatistics dc_hal_get_stats(void)
{
    // augment with TWAI driver bus-level status
    twai_status_info_t twai_status;
    if (twai_get_status_info(&twai_status) == ESP_OK) {
        _dc_stats.bus_off_count = twai_status.bus_error_count;
        _dc_stats.arb_lost_count = twai_status.arb_lost_count;
        _dc_stats.tec_count = twai_status.tx_error_counter;
        _dc_stats.rec_count = twai_status.rx_error_counter;
    }

    _dc_stats.error_sum_count =
        _dc_stats.rx_overflow_count +
        _dc_stats.tx_error_count +
        _dc_stats.rx_error_count +
        _dc_stats.bus_off_count +
        _dc_stats.arb_lost_count;

    return _dc_stats;
}


const char* dc_hal_psr_lec_to_str(uint32_t psr)
{
    (void)psr;
    return ""; // not applicable for ESP32 TWAI
}


const char* dc_hal_psr_act_to_str(uint32_t psr)
{
    (void)psr;
    return ""; // not applicable for ESP32 TWAI
}


int16_t dc_hal_compute_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_bitrate,
    tDcHalCanTimings* const timings)
{
    (void)peripheral_clock_rate;
    (void)target_bitrate;

    // ESP32 TWAI uses TWAI_TIMING_CONFIG_1MBITS() macro directly
    // these values are for reference only
    timings->bit_rate_prescaler = 4;
    timings->bit_segment_1 = 15;
    timings->bit_segment_2 = 4;
    timings->sync_jump_width = 3;
    return 0;
}


#ifdef __cplusplus
}
#endif
#endif // ESPLIB_CAN_H
