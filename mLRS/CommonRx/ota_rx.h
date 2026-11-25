//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OTA, Rx side
// ESP32, ESP8266: no loader, the app receives the new image itself
// STM32: the app hands over to the OTA loader, which receives the new image, see ota_loader.cpp
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
#ifndef OTA_LOADER
extern "C" const volatile tOtaAppInfo ota_app_info = { OTA_APP_INFO_MAGIC, VERSION, DEVICE_NAME };
#define OTA_DEVICE_NAME           (const char*)ota_app_info.device_name
#else
#define OTA_DEVICE_NAME           DEVICE_NAME // the loader is not in the image
#endif


#if !(defined ESP32 || defined ESP8266)
//-------------------------------------------------------
// STM32, the loader's state
//-------------------------------------------------------
// The loader sits in low flash, the app behind it. The flash page in between is the loader's state, a list
// of records which only get appended, the last one counts. The app adds LINK to ask for an update, the loader
// adds DONE when it has served it. The state is away from the EE pages, and the powerup counter page behind them.

#if defined STM32G4 || defined STM32WL
  #define OTA_APP_BASE            (FLASH_BASE + 0x4000) // must match the .ld
#else
  #error OTA loader: mcu not supported!
#endif

#define OTA_STATE_ADR             (OTA_APP_BASE - EE_PAGE_SIZE)
#define OTA_APP_END               EE_START_ADDRESS

typedef enum : uint32_t {
    OTA_STATE_NONE = 0,
    OTA_STATE_LINK = 0x4F544101, // app: please update, a = sx_freq_reg, b = session_id, band, mode
    OTA_STATE_DONE, // loader: the update was served, or no host came
} OTA_STATE_ENUM;

typedef struct
{
    uint32_t a;
    uint32_t b;
    uint32_t spare;
    uint32_t type; // OTA_STATE_ENUM, is written last
} tOtaStateRecord;

typedef struct
{
    uint32_t last; // type of the last record
    tOtaStateRecord link; // the last LINK
    uint32_t free_adr; // 0 if the page is full
} tOtaState;


void ota_state_read(tOtaState* const st)
{
    *st = {};
    for (uint32_t adr = OTA_STATE_ADR; adr < OTA_STATE_ADR + EE_PAGE_SIZE; adr += sizeof(tOtaStateRecord)) {
        const tOtaStateRecord* r = (const tOtaStateRecord*)adr;
        if ((r->a & r->b & r->spare & r->type) == 0xFFFFFFFF) { st->free_adr = adr; return; } // erased, the end
        switch (r->type) {
        case OTA_STATE_LINK: st->link = *r; break;
        case OTA_STATE_DONE: break;
        default: continue; // half written
        }
        st->last = r->type;
    }
}


bool ota_state_program(uint32_t adr, uint32_t type, uint32_t a, uint32_t b)
{
#ifdef EE_USE_DOUBLEWORD
    if (!ee_hal_programdoubleword(adr, ((uint64_t)b << 32) | a)) return false;
    return ee_hal_programdoubleword(adr + 8, ((uint64_t)type << 32) | 0xFFFFFFFF);
#else
    return ee_hal_programword(adr, a) && ee_hal_programword(adr + 4, b) && ee_hal_programword(adr + 12, type);
#endif
}


void ota_state_write(uint32_t type, uint32_t a, uint32_t b)
{
    tOtaState st;
    ota_state_read(&st);

    ee_hal_unlock();
    if (!st.free_adr) { // the page is full, start over, but keep the link, an app which doesn't run needs it
        ee_hal_erasepage(OTA_STATE_ADR, (OTA_STATE_ADR - FLASH_BASE) / EE_PAGE_SIZE);
        st.free_adr = OTA_STATE_ADR;
        if (st.link.type == OTA_STATE_LINK && type != OTA_STATE_LINK) {
            ota_state_program(st.free_adr, OTA_STATE_LINK, st.link.a, st.link.b);
            st.free_adr += sizeof(tOtaStateRecord);
        }
    }
    ota_state_program(st.free_adr, type, a, b);
    ee_hal_lock();
}


#endif

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


#elif defined OTA_LOADER
//-------------------------------------------------------
// STM32 loader, where the app goes
//-------------------------------------------------------
// The image is sent as it is and goes to its place behind the loader, a page is erased when the writing gets to
// it. The first double word, with the stack pointer, is written last, so the app is invalid until all is in.

#define OTA_STORAGE_FLAGS         0


// CRC-32/MPEG-2, is what the crc unit does in its default setup
uint32_t ota_crc(uint32_t crc, const uint8_t* data, uint32_t len)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
    LL_CRC_SetInitialData(CRC, crc);
    LL_CRC_ResetCRCCalculationUnit(CRC);
    while (len--) LL_CRC_FeedData8(CRC, *data++);
    return LL_CRC_ReadData32(CRC);
}


typedef struct
{
    uint32_t space;
    uint32_t length;
    uint32_t adr; // where the next double word goes
    uint8_t dw[8];
    uint8_t dw_len;
    uint64_t first_dw;
} tOtaStorage;


bool ota_program_dw(tOtaStorage* st)
{
    uint32_t adr = st->adr;
    uint64_t dw;
    memcpy(&dw, st->dw, 8);
    st->adr += 8;
    st->dw_len = 0;

    if (!(adr % EE_PAGE_SIZE) && !ee_hal_erasepage(adr, (adr - FLASH_BASE) / EE_PAGE_SIZE)) return false;
    if (adr == OTA_APP_BASE) { st->first_dw = dw; return true; }
    return ee_hal_programdoubleword(adr, dw) && *(volatile uint64_t*)adr == dw;
}


uint8_t ota_storage_begin(tOtaStorage* st, uint32_t length, uint8_t flags, uint32_t image_length)
{
    if (flags || image_length != length || length < 8 || length > st->space) return OTA_STATUS_ERR_LENGTH;

    ee_hal_unlock();
    st->length = length;
    st->adr = OTA_APP_BASE;
    st->dw_len = 0;
    return OTA_STATUS_OK;
}


uint8_t ota_storage_write(tOtaStorage* st, const uint8_t* data, uint8_t len, bool more)
{
    led_green_toggle();
    while (len--) {
        st->dw[st->dw_len++] = *data++;
        if (st->dw_len == 8 && !ota_program_dw(st)) return OTA_STATUS_ERR_FLASH;
    }
    return OTA_STATUS_OK;
}


// the crc of what was transferred was checked before, it's checked again on what is in flash
// only then the first double word goes in, which makes the app valid
uint8_t ota_storage_end(tOtaStorage* st, uint32_t crc)
{
    if (st->dw_len) {
        memset(st->dw + st->dw_len, 0xFF, 8 - st->dw_len);
        if (!ota_program_dw(st)) { ee_hal_lock(); return OTA_STATUS_ERR_FLASH; }
    }

    uint32_t flash_crc = ota_crc(OTA_CRC_INIT, (const uint8_t*)&st->first_dw, 8);
    flash_crc = ota_crc(flash_crc, (const uint8_t*)(OTA_APP_BASE + 8), st->length - 8);
    if (flash_crc != crc) { ee_hal_lock(); return OTA_STATUS_ERR_IMAGE; }

    bool ok = ee_hal_programdoubleword(OTA_APP_BASE, st->first_dw) && *(volatile uint64_t*)OTA_APP_BASE == st->first_dw;
    ee_hal_lock();
    return (ok) ? OTA_STATUS_OK : OTA_STATUS_ERR_FLASH;
}


void ota_storage_init(tOtaStorage* st)
{
    st->space = OTA_APP_END - OTA_APP_BASE;
}


#endif
#if defined ESP32 || defined ESP8266 || defined OTA_LOADER
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


#if defined ESP32 || defined ESP8266
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
#endif


#else
//-------------------------------------------------------
// STM32 app, hands over to the loader
//-------------------------------------------------------

char mlrs_ota_app; // tells the .ld that the app sits behind the loader


class tRxOta
{
  public:
    void Init(void) { enter = false; (void)ota_app_info.magic; } // reading it keeps it in the image
    void Enter(void) { enter = true; }
    bool EnterRequested(void) { return enter; }
    void Run(uint16_t session_id); // does not return

  private:
    bool enter;
};


// asks the loader for the update and reboots into it, does not return
void tRxOta::Run(uint16_t session_id)
{
    __disable_irq(); // the isrs do spi
    sx.SetToIdle();
    sx2.SetToIdle();
    ota_state_write(OTA_STATE_LINK, ota_link_freq_reg(Config.Sx.FrequencyBand),
                    session_id | ((uint32_t)Config.Sx.FrequencyBand << 16) | ((uint32_t)Config.Mode << 24));
    NVIC_SystemReset();
}


#endif
#endif // USE_RX_OTA

#endif // OTA_RX_H
