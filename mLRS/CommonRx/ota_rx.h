//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OTA, Rx side
// ESP32, ESP8266: no loader, the app receives the new image itself
// the rx pulls the image, see ota_protocol.h
//*******************************************************
#ifndef OTA_RX_H
#define OTA_RX_H
#pragma once

#include "../Common/ota/ota_protocol.h"


#ifndef USE_RX_OTA

class tRxOta
{
  public:
    void Init(void) {}
    void Enter(void) {}
    bool EnterRequested(void) { return false; }
    void Run(uint16_t session_id) {}
};

#else

#include "../Common/ota/ota_link.h"


#define OTA_RESPONSE_TMO_MS       300 // covers our request, the way through the host, and its data at SF7
#define OTA_IDLE_TIMEOUT_MS       10000 // leave if no host shows up, it needs some secs to get going
#define OTA_STALL_TIMEOUT_MS      5000 // leave if no answer came for so long
#define OTA_RETRIES               5 // leave if a request got no answer so often

#define OTA_CRC_INIT              0xFFFFFFFF

// tells the host tool which receiver the image is for
// volatile, so that the read in request() is real, else the linker drops it
extern "C" const volatile tOtaAppInfo ota_app_info = { OTA_APP_INFO_MAGIC, VERSION, DEVICE_NAME };
#define OTA_DEVICE_NAME           (const char*)ota_app_info.device_name


#ifdef ESP32
//-------------------------------------------------------
// ESP32, where the app goes
//-------------------------------------------------------
// The image goes into the inactive app partition, the bootloader switches over.

#include <esp_ota_ops.h>
#include <esp_rom_crc.h>
#if defined CONFIG_IDF_TARGET_ESP32C3 // inflate is in the rom
  #include <esp32c3/rom/miniz.h>
#elif defined CONFIG_IDF_TARGET_ESP32S3
  #include <esp32s3/rom/miniz.h>
#else
  #include <esp32/rom/miniz.h>
#endif

#define OTA_STORAGE_FLAGS         OTA_FLAG_DEFLATE


// CRC-32/MPEG-2, the rom functions take and give it inverted, see esp_rom_crc.h
uint32_t ota_crc(uint32_t crc, const uint8_t* data, uint32_t len)
{
    return ~esp_rom_crc32_be(~crc, data, len);
}


typedef struct
{
    const esp_partition_t* partition;
    esp_ota_handle_t handle;
    uint32_t space; // size of the partition
    uint32_t image_length; // what it becomes in flash
    uint32_t written;
    // deflate, inflator = nullptr if not used
    tinfl_decompressor* inflator;
    uint8_t* dict; // is also the output buffer
    uint32_t dict_pos;
    bool inflate_done;
} tOtaStorage;


uint8_t ota_write(tOtaStorage* st, const uint8_t* data, uint32_t len)
{
    if (!len) return OTA_STATUS_OK;
    if (st->written + len > st->image_length) return OTA_STATUS_ERR_DATA;
    if (esp_ota_write(st->handle, data, len) != ESP_OK) return OTA_STATUS_ERR_FLASH;
    st->written += len;
    return OTA_STATUS_OK;
}


// inflates the data and writes the output, more = not the last data
uint8_t ota_inflate_and_write(tOtaStorage* st, const uint8_t* data, uint32_t len, bool more)
{
    if (st->inflate_done) return OTA_STATUS_ERR_DATA;

    while (1) {
        size_t in_len = len;
        size_t out_len = TINFL_LZ_DICT_SIZE - st->dict_pos;
        tinfl_status res = tinfl_decompress(st->inflator, data, &in_len, st->dict, st->dict + st->dict_pos, &out_len,
                                            TINFL_FLAG_PARSE_ZLIB_HEADER | ((more) ? TINFL_FLAG_HAS_MORE_INPUT : 0));
        if (res < TINFL_STATUS_DONE) return OTA_STATUS_ERR_DATA;

        uint8_t status = ota_write(st, st->dict + st->dict_pos, out_len);
        if (status != OTA_STATUS_OK) return status;
        st->dict_pos = (st->dict_pos + out_len) & (TINFL_LZ_DICT_SIZE - 1);
        data += in_len;
        len -= in_len;

        if (res == TINFL_STATUS_DONE) {
            st->inflate_done = true;
            return (len) ? OTA_STATUS_ERR_DATA : OTA_STATUS_OK;
        }
        if (res == TINFL_STATUS_NEEDS_MORE_INPUT && !len) return OTA_STATUS_OK;
        if (!in_len && !out_len) return OTA_STATUS_ERR_DATA; // must not happen
    }
}


uint8_t ota_storage_begin(tOtaStorage* st, uint32_t length, uint8_t flags, uint32_t image_length)
{
    if (image_length > st->space || (!(flags & OTA_FLAG_DEFLATE) && image_length != length)) return OTA_STATUS_ERR_LENGTH;

    if (flags & OTA_FLAG_DEFLATE) {
        st->inflator = (tinfl_decompressor*)malloc(sizeof(tinfl_decompressor));
        st->dict = (uint8_t*)malloc(TINFL_LZ_DICT_SIZE);
        if (!st->inflator || !st->dict) return OTA_STATUS_ERR_UNSUPPORTED; // no memory
        tinfl_init(st->inflator);
    }
    // erases along with the writes, erasing all here would take seconds
    if (esp_ota_begin(st->partition, OTA_WITH_SEQUENTIAL_WRITES, &st->handle) != ESP_OK) return OTA_STATUS_ERR_FLASH;
    st->image_length = image_length;
    return OTA_STATUS_OK;
}


uint8_t ota_storage_write(tOtaStorage* st, const uint8_t* data, uint8_t len, bool more)
{
    led_green_toggle();
    return (st->inflator) ? ota_inflate_and_write(st, data, len, more) : ota_write(st, data, len);
}


uint8_t ota_storage_end(tOtaStorage* st, uint32_t crc)
{
    if (st->written != st->image_length || (st->inflator && !st->inflate_done)) {
        esp_ota_abort(st->handle);
        return OTA_STATUS_ERR_DATA;
    }
    // checks the image, releases the handle in any case
    if (esp_ota_end(st->handle) != ESP_OK || esp_ota_set_boot_partition(st->partition) != ESP_OK) return OTA_STATUS_ERR_IMAGE;
    return OTA_STATUS_OK;
}


void ota_storage_init(tOtaStorage* st)
{
    st->partition = esp_ota_get_next_update_partition(NULL);
    st->space = (st->partition) ? st->partition->size : 0;
}


#elif defined ESP8266
//-------------------------------------------------------
// ESP8266, where the app goes
//-------------------------------------------------------
// No second app partition. The image goes as gzip stream into the free flash behind the app,
// the bootloader (eboot) unpacks it over the app at the next reset.
// ATTENTION: no way back once eboot has started, it doesn't check the stream before!

#include <Updater.h>
#include <coredecls.h>

#define OTA_STORAGE_FLAGS         OTA_FLAG_GZIP


// CRC-32/MPEG-2, is what the core's crc32() does
uint32_t ota_crc(uint32_t crc, const uint8_t* data, uint32_t len)
{
    return crc32(data, len, crc);
}


typedef struct
{
    uint32_t space; // flash for the app and the update together
} tOtaStorage;


uint8_t ota_storage_begin(tOtaStorage* st, uint32_t length, uint8_t flags, uint32_t image_length)
{
    uint32_t length_in_flash = (length + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);

    // the unpacked app must not run into the stream
    if ((!(flags & OTA_FLAG_GZIP) && image_length != length) || image_length + length_in_flash > st->space) return OTA_STATUS_ERR_LENGTH;

    // fails if the chip was not power cycled after flashing by wire
    return (Update.begin(length, U_FLASH)) ? OTA_STATUS_OK : OTA_STATUS_ERR_FLASH;
}


uint8_t ota_storage_write(tOtaStorage* st, const uint8_t* data, uint8_t len, bool more)
{
    led_red_toggle();
    return (Update.write((uint8_t*)data, len) == len) ? OTA_STATUS_OK : OTA_STATUS_ERR_FLASH;
}


// eboot doesn't check the stream, but the crc of all was checked before
uint8_t ota_storage_end(tOtaStorage* st, uint32_t crc)
{
    return (Update.end()) ? OTA_STATUS_OK : OTA_STATUS_ERR_IMAGE;
}


void ota_storage_init(tOtaStorage* st)
{
    st->space = ((ESP.getSketchSize() + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1)) + ESP.getFreeSketchSpace();
}


#endif


//-------------------------------------------------------
// The update
//-------------------------------------------------------

class tRxOta
{
  public:
    void Init(void) { enter = false; }
    void Enter(void) { enter = true; }
    bool EnterRequested(void) { return enter; }
    void Run(uint16_t _session_id); // does not return
    void Update(uint32_t sx_freq_reg, uint16_t _session_id); // returns when done or failed

  private:
    typedef enum {
        STATE_INFO = 0, // ask what the host has for us
        STATE_DATA, // ask for the data at offset
        STATE_RESULT, // tell how it went
        STATE_DONE,
    } STATE_ENUM;

    uint8_t request(void);
    bool handle_response(uint8_t len);

    void radio_start(uint32_t sx_freq_reg);
    void send(uint8_t len);
    uint8_t receive(uint16_t tmo_ms);

    bool enter;
    uint16_t session_id;

    uint8_t state;
    uint8_t result;
    tOtaStorage storage;
    uint32_t length; // what is transferred
    uint32_t crc;
    uint32_t crc_expected;
    uint32_t offset; // of what we want next

    uint8_t buf[OTA_PACKET_LEN_MAX + 8];
};


// puts the request for the state into buf, returns its length
uint8_t tRxOta::request(void)
{
    uint8_t* payload = buf + OTA_PACKET_HEADER_LEN;

    switch (state) {
    case STATE_INFO: {
        tOtaInfoRequest* info = (tOtaInfoRequest*)payload;
        buf[0] = OTA_CMD_INFO;
        info->version = OTA_PROTOCOL_VERSION;
        info->space = storage.space;
        info->flags = OTA_STORAGE_FLAGS;
        strbufstrcpy(info->device_name_20, OTA_DEVICE_NAME, 20); // reading it keeps it in the image
        return OTA_PACKET_HEADER_LEN + sizeof(tOtaInfoRequest); }
    case STATE_DATA: {
        tOtaDataRequest* data = (tOtaDataRequest*)payload;
        buf[0] = OTA_CMD_DATA;
        data->offset = offset;
        data->max_len = OTA_DATA_LEN_MAX;
        return OTA_PACKET_HEADER_LEN + sizeof(tOtaDataRequest); }
    }
    buf[0] = OTA_CMD_RESULT;
    payload[0] = result;
    return OTA_PACKET_HEADER_LEN + 1;
}


// handles the response in buf, returns false if it doesn't answer the request of the state
bool tRxOta::handle_response(uint8_t len)
{
    uint8_t* payload = buf + OTA_PACKET_HEADER_LEN;
    uint8_t payload_len = len - OTA_PACKET_HEADER_LEN;

    switch (state) {
    case STATE_INFO: {
        tOtaInfoResponse* info = (tOtaInfoResponse*)payload;
        if (buf[0] != (OTA_CMD_INFO | OTA_CMD_RESPONSE) || payload_len != sizeof(tOtaInfoResponse)) return false;
        if (info->status != OTA_STATUS_OK) { state = STATE_DONE; return true; } // host doesn't want to update us
        length = info->length;
        crc_expected = info->crc;
        crc = OTA_CRC_INIT;
        offset = 0;
        if (!length || !info->image_length) {
            result = OTA_STATUS_ERR_LENGTH;
        } else
        if (info->flags & ~OTA_STORAGE_FLAGS) {
            result = OTA_STATUS_ERR_UNSUPPORTED;
        } else {
            result = ota_storage_begin(&storage, length, info->flags, info->image_length);
        }
        state = (result == OTA_STATUS_OK) ? STATE_DATA : STATE_RESULT;
        break; }

    case STATE_DATA: {
        tOtaDataResponse* data = (tOtaDataResponse*)payload;
        if (buf[0] != (OTA_CMD_DATA | OTA_CMD_RESPONSE) || payload_len < 4 + 1 || payload_len > sizeof(tOtaDataResponse)) return false;
        if (data->offset != offset) return false; // a late answer to an earlier request
        uint8_t data_len = payload_len - 4;
        if (offset + data_len > length) return false;

        offset += data_len;
        bool more = (offset < length);
        crc = ota_crc(crc, data->data, data_len);
        result = ota_storage_write(&storage, data->data, data_len, more);
        if (result != OTA_STATUS_OK) { state = STATE_RESULT; break; }
        if (more) break;

        result = (crc == crc_expected) ? ota_storage_end(&storage, crc) : OTA_STATUS_ERR_DATA;
        state = STATE_RESULT;
        break; }

    case STATE_RESULT:
        if (buf[0] != (OTA_CMD_RESULT | OTA_CMD_RESPONSE)) return false;
        state = STATE_DONE;
        break;
    }
    return true;
}


//-- the link, see ota_link.h
// isrs are off, DIO is polled and spi done only when it is set

void tRxOta::radio_start(uint32_t sx_freq_reg)
{
    ota_link_start(sx_freq_reg);
    sx.SetRfPower_dbm(rfpower_list[0].dbm); // tx and rx sit next to each other
}


void tRxOta::send(uint8_t len)
{
    ota_set_session_id(buf, session_id);
    ota_link_set_packet_len(len);
    sx.SendFrame(buf, len, 200);

    uint32_t tstart_ms = millis32();
    while (!ota_link_dio_is_set()) { // tx done or timeout
        if (millis32() - tstart_ms > 250) break;
    }
    ota_link_get_and_clear_irq_status();
}


// returns the length of what was received for us, 0 if nothing
uint8_t tRxOta::receive(uint16_t tmo_ms)
{
    ota_link_set_packet_len(255);
    sx.SetToRx();

    uint32_t tstart_ms = millis32();
    while (!ota_link_dio_is_set()) {
        if (millis32() - tstart_ms > tmo_ms) { sx.SetToIdle(); return 0; }
    }
    uint8_t len = ota_link_read(ota_link_get_and_clear_irq_status(), buf);
    sx.SetToIdle();
    return (len && ota_session_id(buf) == session_id) ? len : 0;
}


// runs the update, returns when it is done, failed, or no host showed up
void tRxOta::Update(uint32_t sx_freq_reg, uint16_t _session_id)
{
    session_id = _session_id;

    ota_link_sx_only();
    led_red_on();

    storage = {};
    ota_storage_init(&storage);
    radio_start(sx_freq_reg);

    state = STATE_INFO;
    result = OTA_STATUS_OK;
    bool host_seen = false; // until then we wait for the idle timeout, not for the retries
    uint8_t retries = 0;
    uint32_t tlast_ms = millis32(); // of the last answer

    while (state != STATE_DONE) {
#ifdef ESP8266
        ESP.wdtFeed(); // else its wdt reboots after some seconds
#endif

        if (millis32() - tlast_ms > ((host_seen) ? OTA_STALL_TIMEOUT_MS : OTA_IDLE_TIMEOUT_MS)) break;
        if (host_seen && retries > OTA_RETRIES) break;

        send(request());
        uint8_t len = receive(OTA_RESPONSE_TMO_MS);
        if (!len || !handle_response(len)) { retries++; continue; }
        host_seen = true;
        retries = 0;
        tlast_ms = millis32();
    }
}


// runs the update, then reboots into the new app, or the running one if it failed, does not return
void tRxOta::Run(uint16_t _session_id)
{
    Update(ota_link_freq_reg(Config.Sx.FrequencyBand), _session_id);

    // the chip is left in reset, the startup releases it
#if defined SX_RESET
    gpio_low(SX_RESET);
#elif defined DEVICE_HAS_SX127x
    sx.SetStandby();
#else
    sx.SetStandby(OTA_SX(STDBY_CONFIG_STDBY_RC));
#endif
    delay_us(1000);

    ESP.restart();
    while (1) {}
}


#endif // USE_RX_OTA

#endif // OTA_RX_H
