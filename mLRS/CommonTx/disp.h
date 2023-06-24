//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Display Interface
//********************************************************
#ifndef DISP_H
#define DISP_H
#pragma once


#include "../Common/hal/hal.h"


#ifndef USE_DISPLAY

class tTxDisp
{
  public:
    void Init(void) {};
    void Tick_ms(void) {};
    uint8_t Task(void) { return 0; };
    void DrawBoot(void) {};
    void DrawFlashEsp(void) {};
};
#else


#include <stdlib.h>
#include <ctype.h>
#include "../Common/thirdparty/gfxfont.h"
#include "../Common/thirdparty/gfxfontFreeMono12pt7b.h"
#include "../Common/thirdparty/gfxfontFreeMono9pt7b.h"
#include "../Common/libs/gdisp.h"


extern tGDisplay gdisp;


#define DISP_START_TMO_MS       SYSTICK_DELAY_MS(500)
#define DISP_START_PAGE_TMO_MS  SYSTICK_DELAY_MS(1500)
#define KEYS_DEBOUNCE_TMO_MS    SYSTICK_DELAY_MS(40)


typedef enum {
    PAGE_STARTUP = 0,
    PAGE_NOTIFY_BIND,
    PAGE_NOTIFY_STORE,

    // left-right navigation menu
    PAGE_MAIN,
    PAGE_COMMON,
    PAGE_TX,
    PAGE_RX,
    PAGE_ACTIONS,

    PAGE_NAV_MIN = PAGE_MAIN, // left endpoint
    PAGE_NAV_MAX = PAGE_ACTIONS, //PAGE_RX, // right endpoint

    PAGE_UNDEFINED, // this is also used to time startup page sequence
} PAGE_ENUM;


typedef enum {
    SUBPAGE_DEFAULT = 0,

    // sub pages for main page
    SUBPAGE_MAIN_SUB0 = SUBPAGE_DEFAULT,
    SUBPAGE_MAIN_SUB1,
    SUBPAGE_MAIN_SUB2,

    SUBPAGE_MAIN_NUM,
} SUBPAGE_ENUM;


class tTxDisp
{
  public:
    void Init(void);
    void Tick_ms(void);
    void UpdateMain(void);
    void SetBind(void);
    void Draw(void);
    uint8_t Task(void);
    void DrawNotify(const char* s);
    void DrawBoot(void);
    void DrawFlashEsp(void);

    typedef struct {
        uint8_t list[SETUP_PARAMETER_NUM];
        uint8_t num;
        uint8_t allowed_num[SETUP_PARAMETER_NUM];
        void clear(void) { num = 0; }
        void add(uint8_t param_idx) { list[num] = param_idx; allowed_num[num] = param_get_allowed_opt_num(param_idx); num++; }
    } tParamList;

    bool key_has_been_pressed(uint8_t key_idx);

    void draw_page_startup(void);
    void draw_page_notify(const char* s);
    void draw_page_main(void);
    void draw_page_common(void);
    void draw_page_tx(void);
    void draw_page_rx(void);
    void draw_page_actions(void);

    void draw_page_main_sub0(void);
    void draw_page_main_sub1(void);
    void draw_page_main_sub2(void);

    void draw_header(const char* s);
    void draw_options(tParamList* list);

    bool initialized;
    uint8_t task_pending;
    bool connected_last; // to detect connection changes

    uint16_t keys_tick;
    uint8_t keys_state;
    uint8_t keys_ct0;
    uint8_t keys_ct1;
    uint8_t keys_has_been_pressed;
    uint16_t keys_pressed_tmo;
    uint8_t keys_pending;

    uint8_t page;
    uint16_t page_startup_tmo;
    bool page_modified;

    uint8_t subpage; // for pages which may have different screens
    uint8_t subpage_max;

    uint8_t idx_first;          // index of first param displayed on page
    uint8_t idx_max;            // index of last param in list
    uint8_t idx_focused;        // index of highlighted param
    bool idx_focused_in_edit;   // if param is in edit
    uint8_t idx_focused_pos;    // pos in str6 (bind phrase) parameter
    uint8_t idx_focused_task_pending;

    tParamList common_list;
    tParamList tx_list;
    tParamList rx_list;

    void load_rx_list(void);
    tParamList* current_list(void);

    void page_init(void);
    bool edit_setting(void);
    void run_action(void);
};


void tTxDisp::Init(void)
{
    fiveway_init();

    i2c_init();
    i2c_setdeviceadr(SSD1306_ADR);
    initialized = (i2c_device_ready() == HAL_OK);

    if (initialized) {
        gdisp_init(GDISPLAY_TYPE_SSD1306);
#ifdef DEVICE_HAS_I2C_DISPLAY_ROT180
        gdisp_setrotation(GDISPLAY_ROTATION_180);
#endif
    }

    task_pending = CLI_TASK_NONE;
    connected_last = false;

    keys_tick = 0;
    keys_state = 0;
    keys_ct0 = 0;
    keys_ct1 = 0;
    keys_has_been_pressed = 0;
    keys_pressed_tmo = 0;

    keys_state = fiveway_read();

    page = PAGE_UNDEFINED;
    page_startup_tmo = DISP_START_TMO_MS;
    page_modified = false;

    subpage = SUBPAGE_DEFAULT;
    subpage_max = 0;

    common_list.clear();
    tx_list.clear();
    rx_list.clear();
    for (uint8_t param_idx = 0; param_idx < SETUP_PARAMETER_NUM; param_idx++) {
        if (setup_param_is_tx(param_idx)) {
            tx_list.add(param_idx);
        } else
        if (setup_param_is_rx(param_idx)) {
        //    rx_list.add(param_idx);
        } else {
            common_list.add(param_idx);
        }
    }

    idx_first = 0;
    idx_focused = 0;
    idx_max = 0;
    idx_focused_in_edit = false;
    idx_focused_pos = 0;
    idx_focused_task_pending = CLI_TASK_NONE;
}


uint8_t tTxDisp::Task(void)
{
    uint8_t task = task_pending;
    task_pending = 0;
    return task;
}


void tTxDisp::load_rx_list(void)
{
    rx_list.clear();
    for (uint8_t param_idx = 0; param_idx < SETUP_PARAMETER_NUM; param_idx++) {
        if (setup_param_is_rx(param_idx)) rx_list.add(param_idx);
    }
}


tTxDisp::tParamList* tTxDisp::current_list(void)
{
    switch (page) {
        case PAGE_COMMON: return &common_list;
        case PAGE_TX: return &tx_list;
        case PAGE_RX: return &rx_list;
    }
    return nullptr;
}


void tTxDisp::Tick_ms(void)
{
uint16_t keys, i, keys_new;

    if (!initialized) return;

    // keys debounce
    DECc(keys_tick, KEYS_DEBOUNCE_TMO_MS/4);
    if (!keys_tick) {
        keys = fiveway_read(); // up 0x01, down 0x02, left 0x04, right 0x04, center 0x08

        keys_has_been_pressed = 0;

        i = keys_state ^ keys;                // key changed?
        keys_ct0 =~ (keys_ct0 & i);           // reset or count ct0
        keys_ct1 = keys_ct0 ^ (keys_ct1 & i); // reset or count ct1
        i &= keys_ct0 & keys_ct1;             // count until roll over?
        keys_state ^= i;                      // then toggle debounced state
        keys_new = keys_state & i;            // determine which keys have newly be set
        keys_has_been_pressed |= keys_new;    // 0->1: key press detect, is not erased automatically
    }

    // keys fast scrolling
    if (!keys_state) {
        keys_pressed_tmo = 0;
    }
    uint8_t fast_scroll_mask = (1 << KEY_DOWN) | (1 << KEY_UP);
    if (idx_focused_in_edit) fast_scroll_mask = (1 << KEY_LEFT) | (1 << KEY_RIGHT);
    if (keys_has_been_pressed & fast_scroll_mask) {
        keys_pressed_tmo = SYSTICK_DELAY_MS(750);
    }
    if (keys_pressed_tmo) {
        keys_pressed_tmo--;
        if (!keys_pressed_tmo && (keys_state & fast_scroll_mask)) {
            keys_pressed_tmo = SYSTICK_DELAY_MS(175);
            keys_has_been_pressed = keys_state & fast_scroll_mask;
        }
    }

    // startup page
    if (page_startup_tmo) {
        page_startup_tmo--;
        if (!page_startup_tmo) {
            if (page == PAGE_UNDEFINED) {
                page = PAGE_STARTUP;
                page_startup_tmo = DISP_START_PAGE_TMO_MS;
            } else {
                page = PAGE_MAIN;
                subpage = SUBPAGE_DEFAULT;
                subpage_max = SUBPAGE_MAIN_NUM - 1;
            }
            page_modified = true;
        }
        return;
    }

    // bind
    if (page == PAGE_NOTIFY_BIND) {
        return;
    }

    // store
    if (page == PAGE_NOTIFY_STORE) {
        return;
    }

    // handle connection & receiver
    if (connected_last != connected()) { // connection state has changed
        page_modified = true; // redraws, and avoids calling gdisp_update() multiple time
        if (connected()) load_rx_list();
    }
    connected_last = connected();

    if (!connected() && page == PAGE_RX) { // reset navigation
        idx_first = 0;
        idx_focused = 0;
        idx_focused_in_edit = false;
        idx_focused_pos = 0;
        keys_has_been_pressed &= ((1 << KEY_RIGHT) | (1 << KEY_LEFT)); // only allow these
    }

    // navigation & edit
if(!idx_focused_in_edit){
    // navigation
    // only allow one key event
    if (key_has_been_pressed(KEY_RIGHT)) {
        if (page >= PAGE_NAV_MIN && page < PAGE_NAV_MAX) {
            page++;
            page_init();
            page_modified = true;
        }
    } else
    if (key_has_been_pressed(KEY_LEFT)) {
        if (page > PAGE_NAV_MIN && page <= PAGE_NAV_MAX) {
            page--;
            page_init();
            page_modified = true;
        }

    } else
    if (key_has_been_pressed(KEY_DOWN)) {
        if (idx_focused < idx_max) {
            idx_focused++;
            if (idx_focused - idx_first > (5 - 1)) { idx_first = idx_focused - (5 - 1); } // 5 lines visible
            page_modified = true;
        }
        if (subpage < subpage_max) {
            subpage++;
            page_modified = true;
        }
    } else
    if (key_has_been_pressed(KEY_UP)) {
        if (idx_focused > 0) {
            idx_focused--;
            if (idx_focused < idx_first) { idx_first = idx_focused; }
            page_modified = true;
        }
        if (subpage > 0) {
            subpage--;
            page_modified = true;
        }

    } else
    if (key_has_been_pressed(KEY_CENTER)) {
        if (page >= PAGE_COMMON && page <= PAGE_RX) {
            if (current_list()->allowed_num[idx_focused] > 1) { // param is editable
                idx_focused_in_edit = true;
                idx_focused_pos = 0;
                page_modified = true;
            }
        } else
        if (page == PAGE_ACTIONS) {
            run_action();
        }
    }

}else{
    // edit parameter
/*
    if (key_has_been_pressed(KEY_CENTER)) {
        idx_focused_in_edit = false;
        page_modified = true;
        if (idx_focused_task_pending != CLI_TASK_NONE) task_pending = idx_focused_task_pending;
        idx_focused_task_pending = CLI_TASK_NONE;
    } else {
        edit_setting();
    } */
    if (edit_setting()) { // edit, and finish if true
        idx_focused_in_edit = false;
        page_modified = true;
        if (idx_focused_task_pending != CLI_TASK_NONE) task_pending = idx_focused_task_pending;
        idx_focused_task_pending = CLI_TASK_NONE;
    }

}
}


void tTxDisp::page_init(void)
{
    idx_first = 0;
    idx_focused = 0;
    idx_focused_pos = 0;

    idx_max = 0;
    switch (page) {
        case PAGE_COMMON: idx_max = common_list.num - 1; break;
        case PAGE_TX: idx_max = tx_list.num - 1; break;
        case PAGE_RX: idx_max = rx_list.num - 1; break;
        case PAGE_ACTIONS: 
            idx_max = 2; 
#ifdef USE_ESP_WIFI_BRIDGE
            idx_max++;
#endif
            break;
    }

    subpage = SUBPAGE_DEFAULT;
    subpage_max = 0;
    switch (page) {
        case PAGE_MAIN: subpage_max = SUBPAGE_MAIN_NUM - 1; break;
    }
}


void tTxDisp::run_action(void)
{
    switch (idx_focused) {
    case 0: // STORE
        page = PAGE_NOTIFY_STORE;
        page_modified = true;
        task_pending = CLI_TASK_PARAM_STORE;
        break;
    case 1: // BIND
        task_pending = CLI_TASK_BIND;
        break;
    case 2: // BOOT
        task_pending = CLI_TASK_BOOT;
        break;
    case 3: // FLASH ESP
        task_pending = CLI_TASK_FLASH_ESP;
        break;
    }
}


// this needs to be called to update pages which show live data
void tTxDisp::UpdateMain(void)
{
    if (page == PAGE_MAIN) page_modified = true;
}


void tTxDisp::SetBind(void)
{
    if (page == PAGE_NOTIFY_BIND) return;
    page = PAGE_NOTIFY_BIND;
    page_modified = true;
}


void tTxDisp::DrawNotify(const char* s)
{
    if (!initialized) return;
    draw_page_notify(s);
    gdisp_update();
    page_modified = false;
}


void tTxDisp::DrawBoot(void)
{
    DrawNotify("BOOT");
    delay_ms(250);
}


void tTxDisp::DrawFlashEsp(void)
{
    DrawNotify("FLASH ESP");
}


void tTxDisp::Draw(void)
{
    if (!initialized) return;

//    if (1) { // good for stress testing
    if (page_modified) {
//uint32_t t1 = micros(); //HAL_GetTick();

        switch (page) {
            case PAGE_STARTUP: draw_page_startup(); break;
            case PAGE_MAIN: draw_page_main(); break;
            case PAGE_COMMON: draw_page_common(); break;
            case PAGE_TX: draw_page_tx(); break;
            case PAGE_RX: draw_page_rx(); break;
            case PAGE_ACTIONS: draw_page_actions(); break;
            case PAGE_NOTIFY_BIND: draw_page_notify("BINDING"); break;
            case PAGE_NOTIFY_STORE: draw_page_notify("STORE"); break;
        }

//uint32_t t2 = micros(); //HAL_GetTick();
//dbg.puts("\ndraw ");dbg.puts(u16toBCD_s(t1));dbg.puts(" , ");dbg.puts(u16toBCD_s(t2-t1));

//t1 = micros(); //HAL_GetTick();

        gdisp_update();

        page_modified = false;

//while (!gdisp_update_completed()) {}
//t2 = micros(); //HAL_GetTick();
//dbg.puts("\nupda ");dbg.puts(u16toBCD_s(t1));dbg.puts(" , ");dbg.puts(u16toBCD_s(t2-t1));
    }
}


//-------------------------------------------------------
// Low-level routines
//-------------------------------------------------------

bool tTxDisp::key_has_been_pressed(uint8_t key_idx)
{
    bool pressed = (keys_has_been_pressed & (1 << key_idx));
    keys_has_been_pressed &=~ (1 << key_idx); // clear it
    return pressed;
}


void _diversity_str(char* s, uint8_t div)
{
    switch (div) {
        case 0: strcpy(s, "en."); return;
        case 1: strcpy(s, "ant1"); return;
        case 2: strcpy(s, "ant2"); return;
    }
    strcpy(s, "?");
}


void tTxDisp::draw_header(const char* s)
{
    gdisp_clear();
    gdisp_setcurXY(0, 6);
    gdisp_puts(s);
    gdisp_drawline_H(0, 10, gdisp.width-1, 1);
}


void tTxDisp::draw_options(tParamList* list)
{
char s[32];

    for (uint8_t idx = 0; idx < list->num; idx++) {
        if (idx < idx_first) continue;
        if (idx - idx_first >= 5) break;

        uint8_t param_idx = list->list[idx];

        gdisp_setcurXY(0, (idx - idx_first) * 10 + 20);
        if (idx == idx_focused) gdisp_setinverted();

        if (setup_param_is_tx(param_idx) || setup_param_is_rx(param_idx)) {
            strcpy(s, SetupParameter[param_idx].name + 3); // +3 to remove 'Tx ', 'Rx '
        } else {
            strcpy(s, SetupParameter[param_idx].name);
        }

        s[13] = '\0'; // ensure it's not more than 13 chars
        gdisp_puts(s);
        gdisp_unsetinverted();

        gdisp_setcurX(gdisp.width-1 - 7*6);
        strcpy(s, "ups ?");

        if (SetupParameter[param_idx].type == SETUP_PARAM_TYPE_STR6) {
            param_get_val_formattedstr(s, param_idx, PARAM_FORMAT_DISPLAY);
            // inverted makes it very hard to see, so we draw a box
            /* for (uint8_t i = 0; i < 6; i++) {
                if ((idx == idx_focused) && idx_focused_in_edit && (i == idx_focused_pos)) gdisp_setinverted();
                gdisp_putc(s[i]);
                gdisp_unsetinverted();
            } */
            uint8_t x = gdisp.curX - 2;
            gdisp_puts(s);
            if ((idx == idx_focused) && idx_focused_in_edit) {
                gdisp_drawrect_WH(x + idx_focused_pos*6, gdisp.curY - 9,  11, 13, 1);
            }
        } else {
            if (idx == idx_focused && idx_focused_in_edit) gdisp_setinverted();
            if (list->allowed_num[idx] == 0) { // unavailable
                strcpy(s, "-");
            } else {
                param_get_val_formattedstr(s, param_idx, PARAM_FORMAT_DISPLAY);
                // fake some settings
                if (!strncmp(s,"antenna",7)) { s[3] = s[7]; s[4] = '\0'; }
            }
            s[7] = '\0'; // ensure it's not more than 7 chars
            gdisp_puts(s);
        }
        gdisp_unsetinverted();
    }
}


//-------------------------------------------------------
// Page Draw routines
//-------------------------------------------------------

void tTxDisp::draw_page_startup(void)
{
    gdisp_clear();
    gdisp_setcurXY(0, 6);
    gdisp_setfont(&FreeMono12pt7b);
    gdisp_setcurY(33-10); gdisp_puts_XCentered("mLRS");
    gdisp_unsetfont();
    gdisp_setcurY(48); gdisp_puts_XCentered(DEVICE_NAME);
    gdisp_setcurY(60); gdisp_puts_XCentered(VERSIONONLYSTR);
}


void tTxDisp::draw_page_notify(const char* s)
{
    gdisp_clear();
    gdisp_setcurXY(0, 6);
    gdisp_setfont(&FreeMono12pt7b);
    uint8_t len = strlen(s);
    if (len < 9) {
        gdisp_setcurY(37-10); gdisp_puts_XCentered(s);
    } else {
        gdisp_setcurY(37-10); gdisp_puts(s); // TODO: gets the job done, but could be smarter
    }
    gdisp_unsetfont();
}


void tTxDisp::draw_page_main_sub0(void)
{
char s[32];
int8_t power;

    draw_header("Main");

    gdisp_setcurXY(50, 6);
    param_get_val_formattedstr(s, PARAM_INDEX_MODE); // 1 = index of Mode
    gdisp_puts(s);
    gdisp_setcurX(85);
    power = sx.RfPower_dbm();
    if (power >= -9) { stoBCDstr(power, s); gdisp_puts(s); } else { gdisp_puts("-\x7F"); }
    gdisp_setcurX(100);
    if (connected_and_rx_setup_available()) {
        power = SetupMetaData.rx_actual_power_dbm;
        if (power >= -9) { stoBCDstr(power, s); gdisp_puts(s); } else { gdisp_puts("-\x7F"); }
    }
    gdisp_setcurX(115);
    gdisp_puts("dB");

    gdisp_setcurXY(0, 0 * 10 + 20);
    gdisp_puts("Rssi");

    gdisp_setcurXY(5, 1 * 10 + 20 + 5);
    gdisp_setfont(&FreeMono9pt7b);
    s8toBCDstr(stats.GetLastRssi(), s);
    gdisp_puts(s);
    gdisp_setcurX(60);
    s8toBCDstr(stats.received_rssi, s);
    if (connected()) gdisp_puts(s);
    gdisp_unsetfont();

    gdisp_setcurX(115);
    gdisp_puts("dB");

    gdisp_setcurXY(0, 3 * 10 + 20 - 4);
    gdisp_puts("LQ");

    gdisp_setcurXY(5 + 11, 4 * 10 + 20 + 1);
    gdisp_setfont(&FreeMono9pt7b);
    stoBCDstr(txstats.GetLQ(), s);
    gdisp_puts(s);
    gdisp_setcurX(60 + 11);
    if (connected()) {
        stoBCDstr(stats.received_LQ, s);
        gdisp_puts(s);
    }
    gdisp_unsetfont();

    gdisp_setcurX(115+6);
    gdisp_puts("%");
}


void tTxDisp::draw_page_main_sub1(void)
{
char s[32];

    draw_header("Main/2");

    gdisp_setcurXY(0, 0 * 10 + 20);
    gdisp_puts("Mode");
    gdisp_setcurX(40);
    param_get_val_formattedstr(s, PARAM_INDEX_MODE); // 1 = index of Mode
    gdisp_puts(s);
    gdisp_setcurX(80 + 5);
    stoBCDstr(sx.ReceiverSensitivity_dbm(), s);
    gdisp_puts(s);
    gdisp_puts(" dB");

    gdisp_setcurXY(0, 1 * 10 + 20);
    gdisp_puts("Power");
    gdisp_setcurX(40);
    stoBCDstr(sx.RfPower_dbm(), s);
    gdisp_puts(s);
    gdisp_setcurX(80);
    stoBCDstr(SetupMetaData.rx_actual_power_dbm, s);
    if (connected_and_rx_setup_available()) gdisp_puts(s);

    gdisp_setcurX(115);
    gdisp_puts("dB");

    gdisp_setcurXY(0, 2 * 10 + 20);
    gdisp_puts("Div.");
    gdisp_setcurX(40);
    uint8_t tx_actual_diversity = 3; // 3 = invalid
    if (USE_ANTENNA1 && USE_ANTENNA2) {
        tx_actual_diversity = 0;
    } else if (USE_ANTENNA1) {
        tx_actual_diversity = 1;
    } else if (USE_ANTENNA2) {
        tx_actual_diversity = 2;
    }
    _diversity_str(s, tx_actual_diversity);
    gdisp_puts(s);
    gdisp_setcurX(80);
    uint8_t rx_actual_diversity = (SetupMetaData.rx_available) ? SetupMetaData.rx_actual_diversity : 3; // 3 = invalid
    _diversity_str(s, rx_actual_diversity);
    if (connected_and_rx_setup_available()) gdisp_puts(s);

    gdisp_setcurXY(0, 3 * 10 + 20);
    gdisp_puts("Rssi");
    gdisp_setcurX(40);
    s8toBCDstr(stats.GetLastRssi(), s);
    gdisp_puts(s);
    gdisp_setcurX(80);
    s8toBCDstr(stats.received_rssi, s);
    if (connected()) gdisp_puts(s);

    gdisp_setcurX(115);
    gdisp_puts("dB");

    gdisp_setcurXY(0, 4 * 10 + 20);
    gdisp_puts("LQ");
    gdisp_setcurX(40);
    stoBCDstr(txstats.GetLQ(), s);
    gdisp_puts(s);
    gdisp_setcurX(80);
    if (connected()) {
        stoBCDstr(stats.received_LQ, s);
        gdisp_puts(s);
    }

    gdisp_setcurX(115+6);
    gdisp_puts("%");
}


void tTxDisp::draw_page_main_sub2(void)
{
char s[32];

    draw_header("Main/3");

    gdisp_setcurXY(0, 0 * 10 + 20);
    gdisp_puts(DEVICE_NAME);
    gdisp_setcurXY(0, 1 * 10 + 20);
    gdisp_puts(VERSIONONLYSTR);

    if (connected_and_rx_setup_available()) {
        gdisp_setcurXY(0, 3 * 10 + 20);
        gdisp_puts(SetupMetaData.rx_device_name);
        gdisp_setcurXY(0, 4 * 10 + 20);
        version_to_str(s, SetupMetaData.rx_firmware_version);
        gdisp_puts(s);
    }
}


void tTxDisp::draw_page_main(void)
{
    switch (subpage) {
    case SUBPAGE_MAIN_SUB1:
        draw_page_main_sub1();
        return;
    case SUBPAGE_MAIN_SUB2:
        draw_page_main_sub2();
        return;
    default:
        draw_page_main_sub0();
    }
}


void tTxDisp::draw_page_common(void)
{
    draw_header("Common");
    draw_options(&common_list);
}


void tTxDisp::draw_page_tx(void)
{
    draw_header("Tx");
    draw_options(&tx_list);
}


void tTxDisp::draw_page_rx(void)
{
    draw_header("Rx");

    if (!connected()) {
        gdisp_setcurXY(0, 20);
        gdisp_puts("not connected!");
        return;
    }

    draw_options(&rx_list);
}


void tTxDisp::draw_page_actions(void)
{
    draw_header("Actions");

    gdisp_setfont(&FreeMono9pt7b);

    uint8_t idx = 0;
    gdisp_setcurXY(5, idx * 16 + 25);
    if (idx == idx_focused) gdisp_setinverted();
    gdisp_puts("STORE");
    gdisp_unsetinverted();

/*    idx++;
    gdisp_setcurXY(5, idx * 16 + 25);
    if (idx == idx_focused) gdisp_setinverted();
    gdisp_puts("RELOAD");
    gdisp_unsetinverted(); */

    idx++;
    gdisp_setcurXY(5, idx * 16 + 25);
    if (idx == idx_focused) gdisp_setinverted();
    gdisp_puts("BIND");
    gdisp_unsetinverted();

    gdisp_unsetfont();

    idx++;
    gdisp_setcurXY(75, (idx - 2) * 11 + 20);
    if (idx == idx_focused) gdisp_setinverted();
    gdisp_puts("BOOT");
    gdisp_unsetinverted();

#ifdef USE_ESP_WIFI_BRIDGE
    idx++;
    gdisp_setcurXY(75, (idx - 2) * 11 + 20);
    if (idx == idx_focused) gdisp_setinverted();
    gdisp_puts("FLASH "); gdisp_movecurX(-2); gdisp_puts("ESP");
    gdisp_unsetinverted();
#endif
}


//-------------------------------------------------------
// Edit Parameter
//-------------------------------------------------------

bool tTxDisp::edit_setting(void)
{
    if (!keys_has_been_pressed) return false; // no key pressed, don't do anything

    tParamList* list = current_list();
    uint8_t param_idx = list->list[idx_focused];

    if (key_has_been_pressed(KEY_CENTER)) {
        if (SetupParameter[param_idx].type == SETUP_PARAM_TYPE_STR6) {
            idx_focused_pos++;
            page_modified = true;
            return (idx_focused_pos >= 6);
        }
        return true; // we are done with editing
    }

    tParamValue vv;
    bool rx_param_changed = false;

    if (key_has_been_pressed(KEY_RIGHT)) {

        if (SetupParameter[param_idx].type == SETUP_PARAM_TYPE_INT8) {
            int8_t v = *(int8_t*)(SetupParameter[param_idx].ptr);
            if (v < SetupParameter[param_idx].max.INT8_value) {
                vv.i8 = v + 1;
                rx_param_changed = setup_set_param(param_idx, vv);
                page_modified = true;
            }
        } else
        if (SetupParameter[param_idx].type == SETUP_PARAM_TYPE_LIST) {
            uint8_t v = *(uint8_t*)(SetupParameter[param_idx].ptr);
            uint8_t vmax = param_get_opt_num(param_idx);
            while (v < vmax) {
                v++;
                if (param_get_allowed_mask(param_idx) & (1 << v)) break; // allowed, so be happy
            }
            if (v < vmax) {
                vv.u8 = v;
                rx_param_changed = setup_set_param(param_idx, vv);
                page_modified = true;
            }
        } else
        if (SetupParameter[param_idx].type == SETUP_PARAM_TYPE_STR6) {
            char c = ((char*)SetupParameter[param_idx].ptr)[idx_focused_pos];
            const char* vptr = strchr(bindphrase_chars, c);
            if (!vptr) while(1){} // must not happen
            vptr++;
            if (vptr >= bindphrase_chars + BINDPHRASE_CHARS_LEN) vptr = bindphrase_chars;
            ((char*)SetupParameter[param_idx].ptr)[idx_focused_pos] = *vptr;
            rx_param_changed = true;
            page_modified = true;
        }

    } else
    if (key_has_been_pressed(KEY_LEFT)) {

        if (SetupParameter[param_idx].type == SETUP_PARAM_TYPE_INT8) {
            int8_t v = *(int8_t*)(SetupParameter[param_idx].ptr);
            if (v > SetupParameter[param_idx].min.INT8_value) {
                vv.i8 = v - 1;
                rx_param_changed = setup_set_param(param_idx, vv);
                page_modified = true;
            }
        } else
        if (SetupParameter[param_idx].type == SETUP_PARAM_TYPE_LIST) {
            uint8_t v = *(uint8_t*)(SetupParameter[param_idx].ptr);
            while (1) {
                if (v == 0) { v = UINT8_MAX; break; }
                v--;
                if (param_get_allowed_mask(param_idx) & (1 << v)) break; // allowed, so be happy
            }
            if (v != UINT8_MAX) {
                vv.u8 = v;
                rx_param_changed = setup_set_param(param_idx, vv);
                page_modified = true;
            }
        } else
        if (SetupParameter[param_idx].type == SETUP_PARAM_TYPE_STR6) {
            char c = ((char*)SetupParameter[param_idx].ptr)[idx_focused_pos];
            const char* vptr = strchr(bindphrase_chars, c);
            if (!vptr) while(1){} // must not happen
            vptr--;
            if (vptr < bindphrase_chars) vptr = bindphrase_chars + BINDPHRASE_CHARS_LEN - 1;
            ((char*)SetupParameter[param_idx].ptr)[idx_focused_pos] = *vptr;
            rx_param_changed = true;
            page_modified = true;
        }

    }

    if (rx_param_changed) idx_focused_task_pending = CLI_TASK_RX_PARAM_SET;
    return false; // keep on editing
}


#endif // USE_DISPLAY

#endif // DISP_H



