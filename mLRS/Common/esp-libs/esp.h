//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down STM32 standard library
//*******************************************************
#ifndef STDESP8266_H
#define STDESP8266_H
#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>


//-------------------------------------------------------
//  STD MACROS
//-------------------------------------------------------

#define ABS(a)              ((a)<(0)?(-a):(a))

#define MIN(a,b)            ((a)<(b)?(a):(b))
#define MAX(a,b)            ((a)>(b)?(a):(b))

#define INCc(a,period)      (a)++; if((a)>=(period)){ (a)=0;}
#define DECc(a,period)      ((a)<=0?((a)=(period)-1):((a)--))

#define INCl(a)             if((a)<255){(a)++;}
#define DECl(a)             if((a)>0){(a)--;}

#define INClm(a,max)        if((a)<(max)){(a)++;}

#define INCln(x,n)          {for(i=0;i<n;i++) INCl(x);}
#define DECln(x,n)          {for(i=0;i<n;i++) DECl(x);}

#define INCOPTc(a,min,max)  if((a)<(max)){ (a)++; }else{ (a)=(min); }
#define DECOPTc(a,min,max)  if((a)>(min)){ (a)--; }else{ (a)=(max); }

#define LIMMIN(a,min)       if((a)<(min)){(a)=(min);}
#define LIMMAX(a,max)       if((a)>(max)){(a)=(max);}
#define LIMIT(a,min,max)    if((a)<(min)){(a)=(min);}else{if((a)>(max)){(a)=(max);}}

#define CLIPU8(x)           ((x)<UINT8_MAX)?(x):UINT8_MAX
#define CLIPS8(x)           ((x)>INT8_MAX)?UINT8_MAX:((x)<INT8_MIN)?INT8_MIN:(x)
#define CLIPU16(x)          ((x)<UINT16_MAX)?(x):UINT16_MAX


#ifndef PACKED
#  define PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) //that's for __GNUC__
#endif


#ifndef ALWAYS_INLINE
#define ALWAYS_INLINE  __attribute__ ((always_inline))
#endif


#ifndef INLINE_FORCED
#define INLINE_FORCED  static inline __attribute__ ((always_inline))
#endif


#ifndef ALIGNED_ATTR
#define ALIGNED_ATTR  __attribute__((aligned(4)))
#endif

#ifndef ALIGNED8_ATTR
#define ALIGNED8_ATTR  __attribute__((aligned(8)))
#endif


#ifdef __cplusplus
#  define IRQHANDLER(__Declaration__)  extern "C" {__Declaration__}
#else
#  define IRQHANDLER(__Declaration__)  __Declaration__
#endif


#define ATOMIC(x)  __disable_irq();{x;}__enable_irq();


#ifndef STATIC_ASSERT
#define STATIC_ASSERT_(cond,msg)  _Static_assert((cond),msg);
#define STATIC_ASSERT(cond,msg)  _Static_assert((cond), msg ": (" #cond ") failed");
#endif


#define ATTRIBUTE_OPTIMIZEO3  __attribute__((optimize("O3")))
/*
instead of void __attribute__((optimize("O3"))) foo(void):

#pragma GCC push_options
#pragma GCC optimize ("O3")
...
#pragma GCC pop_options
*/


void __disable_irq(void);
void __enable_irq(void);

//-------------------------------------------------------
//  STD string handling routines
//-------------------------------------------------------

//-- conversion of integers to BCD string

void u8toBCDstr(uint8_t n, char* s);
void s8toBCDstr(int8_t n, char* s);
void u16toBCDstr(uint16_t n, char* s);
void s16toBCDstr(int16_t n, char* s);
void u32toBCDstr(uint32_t n, char* s);
void s32toBCDstr(int32_t n, char* s);

void utoBCDstr(uint32_t n, char* s); // without leading zeros
void stoBCDstr(int32_t n, char* s); // without leading zeros

//-- conversion of u8 to HEX char

char u8toHEXchar(uint8_t n);

//-- conversion of integers to HEX string

void u8toHEXstr(uint8_t n, char* s);
void u16toHEXstr(uint16_t n, char* s);
void u32toHEXstr(uint32_t n, char* s);
void u64toHEXstr(uint64_t n, char* s);

//-- conversion of HEX string/char to integers

uint8_t HEXstrtou8(char* s);
uint16_t HEXstrtou16(char* s);
uint16_t HEXchartou16(uint16_t c);

//-- test functions

uint16_t isHEXchar(char c);
uint16_t isHEXstr(char* s);

//-- conversion of BCD string to integers

uint16_t BCDstrtou16(char* s);
uint32_t BCDstrtou32(char* s);

//-- implicit conversion of integers to BCD string

char* u8toBCD_s(uint8_t n);
char* s8toBCD_s(int8_t n);
char* u16toBCD_s(uint16_t n);
char* s16toBCD_s(int16_t n);
char* u32toBCD_s(uint32_t n);
char* s32toBCD_s(int32_t n);
char* utoBCD_s(uint32_t n);
char* u8toHEX_s(uint8_t n);
char* u16toHEX_s(uint16_t n);
char* u32toHEX_s(uint32_t n);
char* u64toHEX_s(uint64_t n);


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDESP8266_H