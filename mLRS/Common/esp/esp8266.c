//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down STM32 standard library
//*******************************************************

#include "esp8266.h"
#include <Arduino.h>

void __disable_irq(void){
    noInterrupts();
}

void __enable_irq(void){
    interrupts();
}


//-------------------------------------------------------
//  STD string handling routines
//-------------------------------------------------------

//-- helper

void u8toBCD(uint8_t n, char* d2,char* d1,char* d0)
{
  for (*d2 = '0'; n >= 100; n -= 100) { (*d2)++; }
  for (*d1 = '0'; n >= 10; n -= 10) { (*d1)++; }
  *d0 = '0' + n;
}


void u16toBCD(uint16_t n, char* d4,char* d3,char* d2,char* d1,char* d0)
{
  for (*d4 = '0'; n >= 10000; n -= 10000) { (*d4)++; }
  for (*d3 = '0'; n >= 1000; n -= 1000) { (*d3)++; }
  for (*d2 = '0'; n >= 100; n -= 100) { (*d2)++; }
  for (*d1 = '0'; n >= 10; n -= 10) { (*d1)++; }
  *d0 = '0' + n;
}


void u32toBCD(uint32_t n, char* d9,char* d8,char* d7,char* d6,char* d5,char* d4,char* d3,char* d2,char* d1,char* d0)
{
  for (*d9 = '0'; n >= 1000000000; n -= 1000000000) { (*d9)++; }
  for (*d8 = '0'; n >= 100000000; n -= 100000000) { (*d8)++; }
  for (*d7 = '0'; n >= 10000000; n -= 10000000) { (*d7)++; }
  for (*d6 = '0'; n >= 1000000; n -= 1000000) { (*d6)++; }
  for (*d5 = '0'; n >= 100000; n -= 100000) { (*d5)++; }

  for (*d4 = '0'; n >= 10000; n -= 10000) { (*d4)++; }
  for (*d3 = '0'; n >= 1000; n -= 1000) { (*d3)++; }
  for (*d2 = '0'; n >= 100; n -= 100) { (*d2)++; }
  for (*d1 = '0'; n >= 10; n -= 10) { (*d1)++; }
  *d0 = '0' + n;
}


//-- conversion of integers to BCD string

void u8toBCDstr(uint8_t n, char* s)
{
  u8toBCD(n, s, s + 1, s + 2);
  s[3] = '\0';
}


void s8toBCDstr(int8_t n, char* s)
{
  uint8_t u;
  if (n < 0) { s[0] = '-'; u = (n == -128) ? 128 : -n; } else { s[0] = ' '; u = n; }
  u8toBCD(u, s + 1, s + 2, s + 3);
  s[4] = '\0';
}


void u16toBCDstr(uint16_t n, char* s)
{
  u16toBCD(n, s, s + 1, s + 2, s + 3, s + 4);
  s[5] = '\0';
}


void s16toBCDstr(int16_t n, char* s)
{
  uint16_t u;
  if (n < 0) { s[0] = '-'; u = (n == -32768) ? 32768 : -n; } else { s[0] = ' '; u = n; }
  u16toBCD(u, s + 1, s + 2, s + 3, s + 4, s + 5);
  s[6] = '\0';
}


void u32toBCDstr(uint32_t n, char* s)
{
  u32toBCD(n, s, s + 1, s + 2, s + 3, s + 4, s + 5, s + 6, s + 7, s + 8, s + 9);
  s[10] = '\0';
}


void s32toBCDstr(int32_t n, char* s)
{
  uint32_t u;
  if (n < 0) { s[0] = '-'; u = (n == -2147483648) ? 2147483648 : -n; } else { s[0] = ' '; u = n; }
  u32toBCD(u, s + 1, s + 2, s + 3, s + 4, s + 5, s + 6, s + 7, s + 8, s + 9, s + 10);
  s[11] = '\0';
}


// without leading zeros
void utoBCDstr(uint32_t n, char* s)
{
  uint32_t d = 10;
  uint16_t len = 1;
  while (n >= d) { len++; d *= 10; }
  for (int16_t i = len - 1; i >= 0; i--) { s[i] = '0' + (n % 10); n /= 10; }
  s[len] = '\0';
}


// without leading zeros
void stoBCDstr(int32_t n, char* s)
{
  if (n >= 0) { utoBCDstr(n, s); } else { utoBCDstr(-n, s+1); s[0] = '-'; }
}


//-- conversion of u8 to HEX char

char u8toHEXchar(uint8_t n)
{
  if (n <= 9) { return '0' + n; } else { return n - 10 + 'A'; }
}


//-- conversion of integers to HEX string

void u8toHEXstr(uint8_t n, char* s)
{
uint8_t i;

  i = n >> 4;
  if (i <= 9) { s[0] = '0' + i; } else { s[0] = i - 10 + 'A'; }
  i = n & 0x0F;
  if (i <= 9) { s[1] = '0' + i; } else { s[1] = i - 10 + 'A'; }
  s[2] = '\0';
}


void u16toHEXstr(uint16_t n, char* s)
{
char ss[3];

  u8toHEXstr(n, ss);
  u8toHEXstr(n >> 8, s);
  s[2] = ss[0]; s[3] = ss[1]; s[4] = ss[2];
}


void u32toHEXstr(uint32_t n, char* s)
{
char ss[3];

  u8toHEXstr(n, ss);
  s[6] = ss[0]; s[7] = ss[1];
  u8toHEXstr(n >> 8, ss);
  s[4] = ss[0]; s[5] = ss[1];
  u8toHEXstr(n >> 16, ss);
  s[2] = ss[0]; s[3] = ss[1];
  u8toHEXstr(n >> 24, ss);
  s[0] = ss[0]; s[1] = ss[1];
  s[8] = '\0';
}


void u64toHEXstr(uint64_t n, char* s)
{
char ss[3];

  u8toHEXstr(n, ss );
  s[14] = ss[0]; s[15] = ss[1];
  u8toHEXstr(n >> 8, ss);
  s[12] = ss[0]; s[13] = ss[1];
  u8toHEXstr(n >> 16, ss);
  s[10] = ss[0]; s[11] = ss[1];
  u8toHEXstr(n >> 24, ss);
  s[8] = ss[0]; s[9] = ss[1];

  u8toHEXstr(n >> 32, ss);
  s[6] = ss[0]; s[7] = ss[1];
  u8toHEXstr(n >> 40, ss);
  s[4] = ss[0]; s[5] = ss[1];
  u8toHEXstr(n >> 48, ss);
  s[2] = ss[0]; s[3] = ss[1];
  u8toHEXstr(n >> 56, ss);
  s[0] = ss[0]; s[1] = ss[1];

  s[16] = '\0';
}


//-- conversion of HEX string/char to integers

uint8_t HEXstrtou8(char* s)
{
uint8_t val = 0;

  while (*s) {
    val <<= 4;
    if ((*s >= '0') && (*s <= '9')) {
      val += (*s - '0');
    } else
    if ((*s >= 'a') && (*s <= 'f')) {
      val+= (*s  - 'a' + 10);
    } else
    if ((*s >= 'A') && (*s <= 'F')) {
      val+= (*s  - 'A' + 10);
    }
    s++;
  }
  return val;
}


uint16_t HEXstrtou16(char* s)
{
uint16_t val = 0;

  while (*s) {
    val <<= 4;
    if ((*s >= '0') && (*s <= '9')) {
      val += (*s - '0');
    }else
    if ((*s >= 'a') && (*s <= 'f')) {
      val += (*s  - 'a' + 10);
    }else
    if ((*s >= 'A') && (*s <= 'F')) {
      val += (*s  - 'A' + 10);
    }
    s++;
 }
 return val;
}


uint16_t HEXchartou16(uint16_t c)
{
  if ((c >= 'A') && (c <= 'F'))
    return c - 'A' + 10;
  else
  if ((c >= 'a') && (c <= 'f'))
    return c - 'a' + 10;
  else
    return c - '0';
}


//-- test functions

uint16_t isHEXchar(char c)
{
  if ((c >= '0') && (c <= '9')) return 1;
  if ((c >= 'a') && (c <= 'f')) return 1;
  if ((c >= 'A') && (c <= 'F')) return 1;
  return 0;
}


uint16_t isHEXstr(char* s)
{
  while (*s) {
    if (!isHEXchar(*s)) return 0;
    s++;
  }
  return 1;
}


//-- conversion of BCD string to integers

uint16_t BCDstrtou16(char* s)
{
uint16_t i = 0, n = 0;

  while (s[n] != '\0') i = i * 10 + s[n++] - '0';
  return i;
}


uint32_t BCDstrtou32(char* s)
{
uint32_t i = 0, n = 0;

  while (s[n] != '\0') i = i * 10 + s[n++] - '0';
  return i;
}


//-- implicit conversion of integers to BCD string

char ststm32_str[32]; // we hold here a small static string


char* u8toBCD_s(uint8_t n)
{
  u8toBCDstr(n, ststm32_str);
  return ststm32_str;
}


char* s8toBCD_s(int8_t n)
{
  s8toBCDstr(n, ststm32_str);
  return ststm32_str;
}


char* u16toBCD_s(uint16_t n)
{
  u16toBCDstr(n, ststm32_str);
  return ststm32_str;
}


char* s16toBCD_s(int16_t n)
{
  s16toBCDstr(n, ststm32_str);
  return ststm32_str;
}


char* u32toBCD_s(uint32_t n)
{
  u32toBCDstr(n, ststm32_str);
  return ststm32_str;
}


char* s32toBCD_s(int32_t n)
{
  s32toBCDstr(n, ststm32_str);
  return ststm32_str;
}


char* utoBCD_s(uint32_t n)
{
  utoBCDstr(n, ststm32_str);
  return ststm32_str;
}


char* u8toHEX_s(uint8_t n)
{
  u8toHEXstr(n, ststm32_str);
  return ststm32_str;
}


char* u16toHEX_s(uint16_t n)
{
  u16toHEXstr(n, ststm32_str);
  return ststm32_str;
}


char* u32toHEX_s(uint32_t n)
{
  u32toHEXstr(n, ststm32_str);
  return ststm32_str;
}


char* u64toHEX_s(uint64_t n)
{
  u64toHEXstr(n, ststm32_str);
  return ststm32_str;
}


