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


//-------------------------------------------------------
// Transmit Diversity Class
//-------------------------------------------------------
/*
theory of operation

Tx:
 1   \   2  |    NONE     |   INVALID     |  VALID
--------------------------------------------------------
            | alternate   | go quickly    | go quickly
 NONE       | between     | to 2          | to 2
            | 1 & 2       | -75           | -75
--------------------------------------------------------
            | go quickly  | choose which  | go quickly
 INVALID    | to 1 or 2   | is less often | to 2
            | +75         | invalid       | -75
--------------------------------------------------------
            | go quickly  | go quickly    |
 VALID      | to 1        | to 1          | go by rssi
            | +75         | +75           |
--------------------------------------------------------

Rx:
 1   \   2  |    NONE     |   INVALID     |  CRC1 VALID   |  VALID
------------------------------------------------------------------------
            | alternate   | go quickly    | go quickly    | go quickly
 NONE       | between     | to 2          | to 2          | to 2
            | 1 & 2       | -75           | -75           | -75
------------------------------------------------------------------------
            | go quickly  | choose which  | go quickly    | go quickly
 INVALID    | to 1 or 2   | is less often | to 2          | to 2
            | +75         | invalid       | -75           | -75
------------------------------------------------------------------------
            | go quickly  | go quickly    |               | go quickly
 CRC1 VALID | to 1        | to 1          | go by rssi    | to 2
            | +75         | +75           |               | -75
------------------------------------------------------------------------
            | go quickly  | go quickly    | go quickly    |
 VALID      | to 1        | to 1          | to 1          | go by rssi
            | +75         | +75           | +75           |
------------------------------------------------------------------------
*/

#define TDIVERSITY_TIME_CONSTANT  250 // determines the "time constant" to switch
#define TDIVERSITY_INVALID_STEP   75  // determines how quickly it moves otherwise


#define TDIVERSITY_RANGE          (TDIVERSITY_TIME_CONSTANT / 2)


void tTDiversity::Init(uint16_t _frame_rate_ms)
{
    frame_rate_ms = _frame_rate_ms;

    estimator_value = TDIVERSITY_RANGE; // start with antenna1
    estimator_step_last = frame_rate_ms;
    invalid1_cnt = 0;
    invalid2_cnt = 0;

    _seed = 12345; // just use something smaller than m-1

    proposed_antenna = ANTENNA_1;
}


void tTDiversity::DoEstimate(uint8_t link_rx1_status, uint8_t link_rx2_status, int8_t rssi1, int8_t rssi2)
{
int16_t estimator_step;

    if (link_rx1_status <= RX_STATUS_INVALID) {
        if (invalid1_cnt < 5) invalid1_cnt++;
    } else {
        if (invalid1_cnt) invalid1_cnt--;
    }

    if (link_rx2_status <= RX_STATUS_INVALID) {
        if (invalid2_cnt < 5) invalid2_cnt++;
    } else {
        if (invalid2_cnt) invalid2_cnt--;
    }

    // now run the estimator
#ifdef DEVICE_IS_RECEIVER
    if ((link_rx1_status == RX_STATUS_VALID && link_rx2_status == RX_STATUS_VALID) ||
        (link_rx1_status == RX_STATUS_CRC1_VALID && link_rx2_status == RX_STATUS_CRC1_VALID)) {
#else
    if (link_rx1_status == RX_STATUS_VALID && link_rx2_status == RX_STATUS_VALID) {
#endif

        // go into direction of the better antenna
        // choosing frame_rate_ms as step makes it that 2*STEP_RANGE is the time to
        // switch to the other antenna
        if (rssi1 > rssi2) {
            estimator_step = frame_rate_ms; // go towards antenna 1
        } else
        if (rssi1 < rssi2) {
            estimator_step = -frame_rate_ms; // go towards antenna 2
        } else {
            estimator_step = 0; // stay with current antenna
        }

    } else
    if (link_rx1_status > link_rx2_status) {

        // antenna 1 is better, go quickly to antenna 1
        estimator_step = +TDIVERSITY_INVALID_STEP;

    } else
    if (link_rx1_status < link_rx2_status) {

        // antenna 2 is better, go quickly to antenna 2
        estimator_step = -TDIVERSITY_INVALID_STEP;

    } else
    if (link_rx1_status == RX_STATUS_INVALID && link_rx2_status == RX_STATUS_INVALID) {

        // go to the one which is less often invalid
        if (invalid1_cnt < invalid2_cnt) {
            estimator_step = TDIVERSITY_RANGE; // choose antenna 1
        } else
        if (invalid1_cnt > invalid2_cnt) {
            estimator_step = -TDIVERSITY_RANGE; // choose antenna 2
        } else {
            //estimator_step = 0; // stay with current antenna
            estimator_step = (prng()) ? TDIVERSITY_RANGE : -TDIVERSITY_RANGE; // choose randomly
        }

    } else {
        // should be (link_rx1_status == RX_STATUS_NONE && link_rx1_status == RX_STATUS_NONE)

        // alternate
        // Q? should we choose randomly? Or even do random switches, in a Monte Carlo sense?

        if (estimator_step_last >= 0) {
            // we are moving in direction of antenna 1
            if (estimator_value < TDIVERSITY_RANGE) {
                estimator_step = frame_rate_ms; // go further towards antenna 1
            } else {
                estimator_step = -frame_rate_ms; // swap direction and go back to antenna 2
            }
        } else {
            // we are moving in direction of antenna 1
            if (estimator_value > -TDIVERSITY_RANGE) {
                estimator_step = -frame_rate_ms; // go further towards antenna 2
            } else {
                estimator_step = frame_rate_ms; // swap direction and go back to antenna 1
            }
        }

    }

    estimator_value += estimator_step;

    if (estimator_value >= TDIVERSITY_RANGE) {
        estimator_value = TDIVERSITY_RANGE; // constrain it
        proposed_antenna = ANTENNA_1;
    }

    if (estimator_value <= -TDIVERSITY_RANGE) {
        estimator_value = -TDIVERSITY_RANGE; // constrain it
        proposed_antenna = ANTENNA_2;
    }

    estimator_step_last = estimator_step;
}


uint16_t tTDiversity::prng(void) // same as in fhss.h, but generates value 0 or 1
{
    const uint32_t a = 214013;
    const uint32_t c = 2531011;
    const uint32_t m = 2147483648;

    _seed = (a * _seed + c) % m;

    return (2 * _seed) / m; // lo + ((hi - lo + 1) * ran ) / m, see Numerical Recipies in C
}















/*
old attempt
            // work out which antenna we want to choose for next transmission
            //            |   NONE   |  INVALID  | VALID
            // --------------------------------------------------------
            // NONE       |    -     |    +1     |  +3
            // INVALID    |   -1     | better +-x|  +3
            // VALID      |   -3     |     -3    |  better +-x

            if (link_rx1_status == link_rx2_status && link_rx1_status == RX_STATUS_VALID) {
                if (stats.last_rx_rssi1 > stats.last_rx_rssi2 + 10)
                    tx_antenna_estimator += 5;
                else
                if (stats.last_rx_rssi1 > stats.last_rx_rssi2 + 5)
                    tx_antenna_estimator += 3;
                else
                if (stats.last_rx_rssi2 > stats.last_rx_rssi2 + 10)
                    tx_antenna_estimator -= 5;
                else
                if (stats.last_rx_rssi2 > stats.last_rx_rssi2 + 5)
                    tx_antenna_estimator -= 3;
            } else
            if (link_rx1_status == link_rx2_status && link_rx1_status == RX_STATUS_INVALID) {
                if (stats.last_rx_rssi1 > stats.last_rx_rssi2 + 10)
                    tx_antenna_estimator += 2;
                else
                if (stats.last_rx_rssi1 > stats.last_rx_rssi2 + 5)
                    tx_antenna_estimator += 1;
                else
                if (stats.last_rx_rssi2 > stats.last_rx_rssi2 + 10)
                    tx_antenna_estimator -= 2;
                else
                if (stats.last_rx_rssi2 > stats.last_rx_rssi2 + 5)
                    tx_antenna_estimator -= 1;
            } else
            if (link_rx1_status == RX_STATUS_VALID) { // antenna 1 is the clear winner
                tx_antenna_estimator += 3;
            } else
            if (link_rx2_status == RX_STATUS_VALID) { // antenna 2 is the clear winner
                tx_antenna_estimator -= 3;
            } else
            if (link_rx1_status == RX_STATUS_INVALID) { // antenna 1 wins
                tx_antenna_estimator += 1;
            } else
            if (link_rx2_status == RX_STATUS_INVALID) { // antenna 2 wins
                tx_antenna_estimator -= 1;
            }

            if (tx_antenna_estimator >  10) { tx_antenna = ANTENNA_1; tx_antenna_estimator = 10; }
            if (tx_antenna_estimator < -10) { tx_antenna = ANTENNA_2; tx_antenna_estimator = -10; }
*/

