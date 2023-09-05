//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// ConfiId
//********************************************************

#include "config_id.h"
#include "../Common/setup_types.h"


extern volatile uint32_t millis32(void);
extern tSetup Setup;
extern tGlobalConfig Config;


void tConfigId::Init(void)
{
    change_tlast_ms = 0; // 0 = nothing to do
}


void tConfigId::Change(uint8_t config_id)
{
    if (config_id >= SETUP_CONFIG_LEN) {
        config_id = SETUP_CONFIG_LEN - 1;
    }

    if (config_id == Config.ConfigId) {
        change_tlast_ms = 0; // clear any pending change
        return;
    }

    change_tlast_ms = millis32();
    new_config_id = config_id;
}


bool tConfigId::Do(void)
{
    if (!change_tlast_ms) return false;

    uint32_t tnow_ms = millis32();

    if ((tnow_ms - change_tlast_ms) > 750) {
        change_tlast_ms = 0;
        if (new_config_id != Config.ConfigId) {
             Setup._ConfigId = new_config_id;
             return true;
        }
    }

    return false;
}
