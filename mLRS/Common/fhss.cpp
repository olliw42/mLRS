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
  for (uint8_t n =  0; n < FREQ_LIST_LEN; n++) used_flag[n] = false;

//  uartc_puts("fhss generate\n");

  for (uint8_t k = 0; k < cnt; k++) {

     uint8_t fi = prng() % (FREQ_LIST_LEN - k); // get a new frequency index

     uint8_t i = 0;
     uint8_t n;
     for (n = 0; n < FREQ_LIST_LEN; n++) {
       if (used_flag[n]) continue;
       if (fi == i) break; // n is our next index
       i++;
     }

     if (n >= FREQ_LIST_LEN) { // argh, must not happen !
       n = 0;
     }

     fhss_list[k] = freq_list[n];
     used_flag[n] = true;
/*
     uartc_puts("k = "); uartc_puts(u8toBCD_s(k)); uartc_puts(", "); delay_ms(25);
     uartc_puts("fi = "); uartc_puts(u8toBCD_s(fi)); uartc_puts(", "); delay_ms(25);
     uartc_puts("n = "); uartc_puts(u8toBCD_s(n)); uartc_puts(", "); delay_ms(25);
     uartc_puts("f = "); uartc_puts(u32toBCD_s(fhss_list[k])); uartc_puts("\n"); delay_ms(25);
*/
  }

  curr_i = 0;
  // mark all channels as equally bad
  for (uint8_t k = 0; k < cnt; k++) {
    fhss_last_rssi[k] = INT8_MIN;
  }
}


