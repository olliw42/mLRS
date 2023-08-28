//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Diversity
//********************************************************


#include "hal/device_conf.h"
#include "common_types.h"
#include "link_types.h"
#include "diversity.h"


//-------------------------------------------------------
// Receive Diversity Class
//-------------------------------------------------------
/*
theory of operation
work out which antenna we choose

Tx:
            |   NONE   |  INVALID  | VALID
--------------------------------------------------------
 NONE       |          |   1 or 2  |  1
 INVALID    |  1 or 2  |   1 or 2  |  1
 VALID      |    2     |     2     |  1 or 2

Rx:
            |   NONE   |  INVALID  | CRC1_VALID | VALID
--------------------------------------------------------
 NONE       |          |   1 or 2  |     1      |  1
 INVALID    |  1 or 2  |   1 or 2  |     1      |  1
 CRC1_VALID |    2     |     2     |   1 or 2   |  1
 VALID      |    2     |     2     |     2      |  1 or 2
*/


uint8_t tRDiversity::Antenna(uint8_t link_rx1_status, uint8_t link_rx2_status, int8_t rssi1, int8_t rssi2)
{
uint8_t antenna;

    if (link_rx1_status == link_rx2_status) {
        // we can choose either antenna, so select the one with the better rssi
        antenna = (rssi1 > rssi2) ? ANTENNA_1 : ANTENNA_2;
    } else
    if (link_rx1_status == RX_STATUS_VALID) {
        antenna = ANTENNA_1;
    } else
    if (link_rx2_status == RX_STATUS_VALID) {
        antenna = ANTENNA_2;
#ifdef DEVICE_IS_RECEIVER
    } else
    if (link_rx1_status == RX_STATUS_CRC1_VALID) {
        antenna = ANTENNA_1;
    } else
    if (link_rx2_status == RX_STATUS_CRC1_VALID) {
        antenna = ANTENNA_2;
#endif
    } else {
        // we can choose either antenna, so select the one with the better rssi
        antenna = (rssi1 > rssi2) ? ANTENNA_1 : ANTENNA_2;
    }

    return antenna;
}

