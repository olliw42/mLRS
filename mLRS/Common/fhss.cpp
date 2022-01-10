//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// FHSS
//*******************************************************


#include <stdint.h>
#include "fhss.h"


// https://en.wikipedia.org/wiki/Linear_congruential_generator: Microsoft Visual/Quick C/C++
// also used by ELRS
// generates values in range [0, 0x7FFF]
uint16_t FhssBase::prng(void)
{
  const uint32_t a = 214013;
  const uint32_t c = 2531011;
  const uint32_t m = 2147483648;
  _seed = (a * _seed + c) % m;
  return _seed >> 16;
}


void FhssBase::generate(uint32_t seed)
{
  _seed = seed;

  bool used_flag[FREQ_LIST_LEN];
  for (uint8_t ch = 0; ch < FREQ_LIST_LEN; ch++) used_flag[ch] = false;

  for (uint8_t k = 0; k < cnt; k++) {

     uint8_t fi = prng() % (FREQ_LIST_LEN - k); // get a new frequency index

     uint8_t i = 0;
     uint8_t ch;
     for (ch = 0; ch < FREQ_LIST_LEN; ch++) {
       if (used_flag[ch]) continue;
       if (fi == i) break; // ch is our next index
       i++;
     }

     if (ch >= FREQ_LIST_LEN) { // argh, must not happen !
       ch = 0;
     }

     fhss_list[k] = freq_list[ch];
     used_flag[ch] = true;
  }

  curr_i = 0;
  // mark all channels as equally bad
  for (uint8_t k = 0; k < cnt; k++) {
    fhss_last_rssi[k] = INT8_MIN;
  }
}


