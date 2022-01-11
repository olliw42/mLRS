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
// spektrum:
// appears to use val = val * 0x0019660D + 0x3C6EF35F; (val >> 8) % 73
// is numerical recipes method in the wikipedia source
// picks 24 channels out of 74, ensures it's distributed into three slots
// redpine:
// same prng as spektrum, picks 50 channels, ensures not close and not 0,1

//QUESTION:
// how many different fhss sequences can be generated with the 32bit seed value ??

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

  uint8_t k = 0;
  while (k < cnt) {

     uint8_t rn = prng() % (FREQ_LIST_LEN - k); // get a random number in the remaining range

     uint8_t i = 0;
     uint8_t ch;
     for (ch = 0; ch < FREQ_LIST_LEN; ch++) {
       if (used_flag[ch]) continue;
       if (i == rn) break; // ch is our next index
       i++;
     }

     if (ch >= FREQ_LIST_LEN) { // argh, must not happen !
       ch = 0;
     }

     // do not pick a bind channel
     bool is_bind_channel = false;
     for (uint8_t bi = 0; bi < FHSS_BIND_CHANNEL_LIST_LEN; bi++) {
       if (ch == fhss_bind_channel_list[bi]) is_bind_channel = true;
     }
     if (is_bind_channel) continue;

     // ensure it is not too close to the previous
     bool is_too_close = false;
     if (k > 0) {
       int8_t last_ch = ch_list[k - 1];
       if (last_ch == 0) { // special treatment for this case
         if (ch < 2) is_too_close = true;
       } else {
         if ((ch >= last_ch - 1) && (ch <= last_ch + 1)) is_too_close = true;
       }
     }
     if (is_too_close) continue;

     // we got a new ch, so register it
     ch_list[k] = ch;
     fhss_list[k] = fhss_freq_list[ch];
     used_flag[ch] = true;

     k++;
  }

  curr_i = 0;
  // mark all channels as equally bad
  for (uint8_t k = 0; k < cnt; k++) {
    fhss_last_rssi[k] = INT8_MIN;
  }
}


