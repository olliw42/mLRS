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


#include "..\Common\hal\hal.h"


#ifndef USE_DISPLAY

class tTxDisp
{
  public:
    void Init(void) {};
    void Tick(void) {};
};
#else


#include <stdlib.h>
#include <ctype.h>
#include "..\Common\thirdparty\gfxfont.h"
#include "..\Common\thirdparty\gfxfontFreeMono12pt7b.h"
#include "..\Common\thirdparty\gfxfontFreeMono9pt7b.h"
#include "..\CommonTx\gdisp.h"

extern tGDisplay gdisp;

#define DISP_START_TMO_MS       SYSTICK_DELAY_MS(500)
#define DISP_START_PAGE_TMO_MS  SYSTICK_DELAY_MS(1500)
#define KEYS_DEBOUNCE_TMO_MS    SYSTICK_DELAY_MS(60)


typedef enum {
    PAGE_STARTUP = 0,
    PAGE_BIND,

    // left-right navigation menu
    PAGE_MAIN,
    PAGE_COMMON,
    PAGE_TX,
    PAGE_RX,
    PAGE_NAV_MIN = PAGE_MAIN,
    PAGE_NAV_MAX = PAGE_RX,

    PAGE_UNDEFINED, // this is also used to time startup page sequence
} PAGE_ENUM;


typedef enum {
    SUBPAGE_DEFAULT = 0,

    // sub pages for main page
    SUBPAGE_MAIN_SUB0 = SUBPAGE_DEFAULT,
    SUBPAGE_MAIN_SUB1,
} SUBPAGE_ENUM;


class tTxDisp
{
  public:
    void Init(void);
    void Tick(void);
    void UpdateMain(void);
    void SetBind(void);
    void Draw(void);

    bool key_has_been_pressed(uint8_t key_idx);

    void draw_page_startup(void);
    void draw_page_bind(void);
    void draw_page_main(void);
    void draw_page_common(void);
    void draw_page_tx(void);
    void draw_page_rx(void);

    void draw_page_main_sub0(void);
    void draw_page_main_sub1(void);

    void draw_header(const char* s);
    void draw_options(uint8_t* list, uint8_t num);

    bool initialized;

    uint16_t keys_tick;
    uint8_t keys_state;
    uint8_t keys_ct0;
    uint8_t keys_ct1;
    uint8_t keys_has_been_pressed;
    uint16_t keys_pressed_tmo;

    uint8_t page;
    uint16_t page_startup_tmo;
    bool page_modified;

    uint8_t subpage; // for pages which may have different screens
    uint8_t subpage_max;

    uint8_t idx_first;
    uint8_t idx_max;
    uint8_t idx_focused;

    uint8_t common_list[SETUP_PARAMETER_NUM];
    uint8_t common_list_num;
    uint8_t tx_list[SETUP_PARAMETER_NUM];
    uint8_t tx_list_num;
    uint8_t rx_list[SETUP_PARAMETER_NUM];
    uint8_t rx_list_num;
};


void tTxDisp::Init(void)
{
    fiveway_init();

    i2c_init();
    i2c_setdeviceadr(SSD1306_ADR);
    initialized = (i2c_device_ready() == HAL_OK);

    if (initialized) {
        gdisp_init(GDISPLAY_TYPE_SSD1306);
        gdisp_setrotation(GDISPLAY_ROTATION_180);
    }

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

    common_list_num = tx_list_num = rx_list_num = 0;
    memset(common_list, 0, SETUP_PARAMETER_NUM);
    memset(tx_list, 0, SETUP_PARAMETER_NUM);
    memset(rx_list, 0, SETUP_PARAMETER_NUM);
    for (uint8_t param_idx = 0; param_idx < SETUP_PARAMETER_NUM; param_idx++) {
        if (setup_param_is_tx(param_idx)) {
            tx_list[tx_list_num++] = param_idx;
        } else
        if (setup_param_is_rx(param_idx)) {
            rx_list[rx_list_num++] = param_idx;
        } else {
            common_list[common_list_num++] = param_idx;
        }
    }

    idx_first = 0;
    idx_focused = 0;
    idx_max = 0;
}


void tTxDisp::Tick(void)
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

    // keys scrolling
    if (!keys_state) {
        keys_pressed_tmo = 0;
    }
    if (keys_has_been_pressed & (1 << KEY_DOWN) || keys_has_been_pressed & (1 << KEY_UP)) {
        keys_pressed_tmo = SYSTICK_DELAY_MS(750);
    }
    if (keys_pressed_tmo) {
        keys_pressed_tmo--;
        if (!keys_pressed_tmo && (keys_state & (1 << KEY_DOWN) || keys_state & (1 << KEY_UP))) {
            keys_pressed_tmo = SYSTICK_DELAY_MS(175);
            keys_has_been_pressed = keys_state;
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
                subpage_max = 1;
            }
            page_modified = true;
        }
        return;
    }

    // bind
    if (page == PAGE_BIND) return;

    // navigation
    if (key_has_been_pressed(KEY_RIGHT)) {
        if (page >= PAGE_NAV_MIN && page < PAGE_NAV_MAX) {
            page++;
            page_modified = true;
            idx_first = 0;
            idx_focused = 0;
            switch (page) {
                case PAGE_MAIN: idx_max = 0; break;
                case PAGE_COMMON: idx_max = common_list_num - 1; break;
                case PAGE_TX: idx_max = tx_list_num - 1; break;
                case PAGE_RX: idx_max = rx_list_num - 1; break;
            }
            subpage = SUBPAGE_DEFAULT;
        }
    }
    if (key_has_been_pressed(KEY_LEFT)) {
        if (page > PAGE_NAV_MIN && page <= PAGE_NAV_MAX) {
            page--;
            page_modified = true;
            idx_first = 0;
            idx_focused = 0;
            switch (page) {
                case PAGE_MAIN: idx_max = 0; break;
                case PAGE_COMMON: idx_max = common_list_num - 1; break;
                case PAGE_TX: idx_max = tx_list_num - 1; break;
                case PAGE_RX: idx_max = rx_list_num - 1; break;
            }
            subpage = SUBPAGE_DEFAULT;
            subpage_max = 0;
            switch (page) {
                case PAGE_MAIN: subpage_max = 1; break;
            }
        }
    }

    if (key_has_been_pressed(KEY_DOWN)) {
        if (idx_focused < idx_max) {
            idx_focused++;
            if (idx_focused - idx_first > (5 - 1)) { idx_first = idx_focused - (5 - 1); }
            page_modified = true;
        }
        if (subpage < subpage_max) {
            subpage++;
            page_modified = true;
        }
    }
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
    }
}


// this needs to be called to update pages which show live data
void tTxDisp::UpdateMain(void)
{
    if (page == PAGE_MAIN) page_modified = true;
}


void tTxDisp::SetBind(void)
{
    if (page == PAGE_BIND) return;
    page = PAGE_BIND;
    page_modified = true;
}


void tTxDisp::Draw(void)
{
    if (!initialized) return;

//    if (1) { // good for stress testing
    if (page_modified) {
//uint32_t t1 = micros(); //HAL_GetTick();

        switch (page) {
            case PAGE_STARTUP: draw_page_startup(); break;
            case PAGE_BIND: draw_page_bind(); break;
            case PAGE_MAIN: draw_page_main(); break;
            case PAGE_COMMON: draw_page_common(); break;
            case PAGE_TX: draw_page_tx(); break;
            case PAGE_RX: draw_page_rx(); break;
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


void tTxDisp::draw_page_bind(void)
{
    gdisp_clear();
    gdisp_setcurXY(0, 6);
    gdisp_setfont(&FreeMono12pt7b);
    gdisp_setcurY(37-10); gdisp_puts_XCentered("BINDING");
    gdisp_unsetfont();
}


void tTxDisp::draw_header(const char* s)
{
    gdisp_clear();
    gdisp_setcurXY(0, 6);
    gdisp_puts(s);
    gdisp_drawline_H(0, 10, gdisp.width-1, 1);
}


void _disp_div_str(char* s, uint8_t div)
{
    switch (div) {
        case 0: strcpy(s, "en."); return;
        case 1: strcpy(s, "ant1"); return;
        case 2: strcpy(s, "ant2"); return;
    }
    strcpy(s, "?");
}


void tTxDisp::draw_page_main_sub0(void)
{
char s[32];

    draw_header("Main");

    gdisp_setcurXY(0, 0 * 10 + 20);
    gdisp_puts("Mode");
    gdisp_setcurX(40);
    param_get_setting_str(s, 1);
    gdisp_puts(s);
    gdisp_setcurX(80 + 5);
    stoBCDstr(sx.ReceiverSensitivity_dbm(), s);
    strcat(s, " dB");
    gdisp_puts(s);

    gdisp_setcurXY(0, 1 * 10 + 20);
    gdisp_puts("Power");
    gdisp_setcurX(40);
    stoBCDstr(sx.RfPower_dbm(), s);
    gdisp_puts(s);
    gdisp_setcurX(80);
    stoBCDstr(SetupMetaData.rx_actual_power_dbm, s);
    if (connected() && SetupMetaData.rx_available) gdisp_puts(s);

    gdisp_setcurX(115);
    strcpy(s, "dB");
    gdisp_puts(s);

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
    _disp_div_str(s, tx_actual_diversity);
    gdisp_puts(s);
    gdisp_setcurX(80);
    uint8_t rx_actual_diversity = (SetupMetaData.rx_available) ? SetupMetaData.rx_actual_diversity : 3; // 3 = invalid
    _disp_div_str(s, rx_actual_diversity);
    if (connected() && SetupMetaData.rx_available) gdisp_puts(s);

    gdisp_setcurXY(0, 3 * 10 + 20);
    gdisp_puts("Rssi");
    gdisp_setcurX(40);
    s8toBCDstr(stats.GetLastRxRssi(), s);
    gdisp_puts(s);
    gdisp_setcurX(80);
    s8toBCDstr(stats.received_rssi, s);
    if (connected()) gdisp_puts(s);

    gdisp_setcurX(115);
    strcpy(s, "dB");
    gdisp_puts(s);

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
    strcpy(s, "%");
    gdisp_puts(s);
}


void tTxDisp::draw_page_main_sub1(void)
{
char s[32];

    draw_header("Main/1");

    gdisp_setcurXY(0, 0 * 10 + 20);
    gdisp_puts("Rssi");

    gdisp_setcurXY(5, 1 * 10 + 20 + 5);
    gdisp_setfont(&FreeMono9pt7b);
    s8toBCDstr(stats.GetLastRxRssi(), s);
    gdisp_puts(s);
    gdisp_setcurX(60);
    s8toBCDstr(stats.received_rssi, s);
    if (connected()) gdisp_puts(s);
    gdisp_unsetfont();

    gdisp_setcurX(115);
    strcpy(s, "dB");
    gdisp_puts(s);

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
    strcpy(s, "%");
    gdisp_puts(s);
}


void tTxDisp::draw_page_main(void)
{
    switch (subpage) {
    case SUBPAGE_MAIN_SUB1: draw_page_main_sub1(); return;
    default:
        draw_page_main_sub0();
    }
}


void tTxDisp::draw_options(uint8_t* list, uint8_t num)
{
char s[32];

    for (uint8_t idx = 0; idx < num; idx++) {
        if (idx < idx_first) continue;
        if (idx - idx_first >= 5) break;

        uint8_t param_idx = list[idx];

        gdisp_setcurXY(0, (idx - idx_first) * 10 + 20);

        if (idx == idx_focused) gdisp_setinverted();
        if (setup_param_is_tx(param_idx) || setup_param_is_rx(param_idx)) {
            strcpy(s, SetupParameter[param_idx].name + 3);
        } else {
            strcpy(s, SetupParameter[param_idx].name);
        }
        s[13] = '\0';
        gdisp_puts(s);

        gdisp_unsetinverted();

        gdisp_setcurX(gdisp.width-1 - 7*6);
        strcpy(s, "ups");
        param_get_setting_str(s, param_idx);
        if (!strncmp(s,"antenna",7)) { s[6] = s[7]; }
        s[7] = '\0';
        gdisp_puts(s);
    }
}


void tTxDisp::draw_page_common(void)
{
    draw_header("Common");
    draw_options(common_list, common_list_num);
}


void tTxDisp::draw_page_tx(void)
{
    draw_header("Tx");
    draw_options(tx_list, tx_list_num);
}


void tTxDisp::draw_page_rx(void)
{
    draw_header("Rx");
    draw_options(rx_list, rx_list_num);
}


#endif // USE_DISPLAY

#endif // DISP_H



