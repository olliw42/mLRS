//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// mLRS RX OTA loader, STM32
//*******************************************************
// Sits in low flash in front of the app, and starts it. Waits for an update over the air when the app asks for
// one, or when there is no app which runs, see ota_rx.h. Is built from the same sources as the app, with this
// file in place of mlrs-rx.cpp, see tools/run_make_firmwares.py and ota_loader.mk.
// In the app's build this file puts the loader, which was built before, into the image.
//*******************************************************

#ifndef MLRS_OTA_LOADER_BUILD

#include "../Common/hal/device_conf.h"

#ifdef DEVICE_HAS_OTA_LOADER
// the .ld places it at the flash start, the app's flash and debug then have the loader too
asm(".section .ota_loader,\"a\"\n.incbin \"ota_loader.bin\"\n.previous");
#endif

#else

#define OTA_LOADER

#define SX_DIO_EXTI_IRQ_PRIORITY    13

#include "../Common/common_conf.h"
#include "../Common/common_types.h"
#include "../Common/sx-drivers/sx12xx.h"
#include "../Common/hal/glue.h"
#include "../modules/stm32ll-lib/src/stdstm32.h"
#include "../modules/stm32ll-lib/src/stdstm32-peripherals.h"
#ifdef STM32WL
#include "../modules/stm32ll-lib/src/stdstm32-subghz.h"
#endif
#include "../Common/hal/hal.h"
#include "../modules/stm32ll-lib/src/stdstm32-delay.h"
#include "../modules/stm32ll-lib/src/stdstm32-eeprom.h"
#include "../modules/stm32ll-lib/src/stdstm32-spi.h"
#include "../Common/hal/timer.h"
#include "../Common/sx-drivers/sx12xx_driver.h"
#ifdef STM32G4
#include "stm32g4xx_ll_crc.h"
#elif defined STM32WL
#include "stm32wlxx_ll_crc.h"
#endif


char mlrs_ota_loader; // tells the .ld that this is the loader

tGlobalConfig Config; // only the band and mode, for the sx driver and the link
SX_DRIVER sx;
SX2_DRIVER sx2;


#include "ota_rx.h"

tRxOta ota;


IRQHANDLER(
void SX_DIO_EXTI_IRQHandler(void)
{
    sx_dio_exti_isr_clearflag(); // the update polls the dio
})


//-------------------------------------------------------
// The app
//-------------------------------------------------------

// checks the vector table, the first double word is erased until an update is complete
bool app_valid(void)
{
    uint32_t sp = *(volatile uint32_t*)OTA_APP_BASE;
    uint32_t pc = *(volatile uint32_t*)(OTA_APP_BASE + 4);
    return ((sp & 0xFFF00000) == SRAM1_BASE) && (pc > OTA_APP_BASE) && (pc < OTA_APP_END);
}


// does not return
void start_app(void)
{
    // the app expects things as after a reset
    HAL_RCC_DeInit();
    HAL_DeInit();
    __disable_irq();
    SysTick->CTRL = 0;
    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    uint32_t sp = *(volatile uint32_t*)OTA_APP_BASE;
    uint32_t pc = *(volatile uint32_t*)(OTA_APP_BASE + 4);
    SCB->VTOR = OTA_APP_BASE;
    __set_MSP(sp);
    __enable_irq();
    ((void (*)(void))pc)();
    while (1) {}
}


//-------------------------------------------------------
// Main
//-------------------------------------------------------

void main_loop(void) // never returns
{
    delay_init();
    leds_init();

    bool radio_up = false;

    while (1) {
        tOtaState st;
        ota_state_read(&st);

        // start the app, unless it asks for an update, or there is no app which runs, then wait for an update
        if (app_valid() && st.last != OTA_STATE_LINK) start_app();

        if (st.link.type != OTA_STATE_LINK) { // we don't know the link, only flashing by wire helps
            while (1) { led_red_toggle(); delay_ms(100); }
        }

        if (!radio_up) {
            Config.Sx.FrequencyBand = (SX_FHSS_FREQUENCY_BAND_ENUM)((st.link.b >> 16) & 0xFF);
            Config.Mode = st.link.b >> 24;
            Config.Sx.is_lora = true;
            Config.Sx.Power_dbm = rfpower_list[0].dbm;
            sx.Init();
            sx.StartUp(&Config.Sx);
            radio_up = true;
        }

        ota.Update(st.link.a, st.link.b & 0xFFFF);

        // the request is served, with an update or without, e.g. no host showed up
        if (st.last == OTA_STATE_LINK) ota_state_write(OTA_STATE_DONE, 0, 0);
    }
}

#endif // MLRS_OTA_LOADER_BUILD
