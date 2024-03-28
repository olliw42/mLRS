//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP - Emulated EEPROM standard library
//*******************************************************
// Interface:
//
// #define EE_START_PAGE // mandatory
// #define EE_PAGE_SIZE  // optional
//
//*******************************************************
#ifndef ESPLIB_EEPROM_H
#define ESPLIB_EEPROM_H

#include <EEPROM.h>

typedef enum {
    EE_PAGE0 = 0,
    EE_PAGE1,
} EE_PAGE_ENUM;

typedef enum {
    EE_STATUS_FLASH_FAIL = 0, // indicates failure in hal functions
    EE_STATUS_PAGE_UNDEF,
    EE_STATUS_PAGE_EMPTY,
    EE_STATUS_PAGE_FULL,
    EE_STATUS_OK
} EE_STATUS_ENUM;

// ESP8266 has 4kb available for EEPROM. We need 3 pages, 2 for setup and 
// 1 for the powerup counter. So best to keep page size to 3kb to let it 
// fit in. 
#define EE_PAGE_SIZE  0x0400 // Page size = 1 KByte

// EEPROM start address in Flash
#define EE_START_ADDRESS          ((uint32_t)(0x0000))

// Pages 0 and 1 base and end addresses
#define EE_PAGE0_BASE_ADDRESS     ((uint32_t)(EE_START_ADDRESS + 0x0000))
#define EE_PAGE0_END_ADDRESS      ((uint32_t)(EE_START_ADDRESS + (EE_PAGE_SIZE - 1)))

#define EE_PAGE1_BASE_ADDRESS     ((uint32_t)(EE_START_ADDRESS + EE_PAGE_SIZE))
#define EE_PAGE1_END_ADDRESS      ((uint32_t)(EE_START_ADDRESS + (2 * EE_PAGE_SIZE - 1)))

#define EE_START_PAGE0            ((uint32_t)(EE_START_PAGE))
#define EE_START_PAGE1            ((uint32_t)(EE_START_PAGE + 1))

// Page status definitions stored in EEPROM
#define EE_ERASE                  ((uint32_t)0xFFFFFFFF)     // PAGE is erased, may not be completely erased in case of an error
#define EE_VALID_PAGE             ((uint32_t)0x11111111)     // PAGE containing valid data

#define EE_USE_WORD

//-------------------------------------------------------
// FLASH extensions
//-------------------------------------------------------

typedef enum {
    FLASH_STATUS_BUSY = 1, // called FLASH_STATUS_xxx to avoid overlap with HAL
    FLASH_STATUS_ERROR_PG,
    FLASH_STATUS_ERROR_WRP,
    FLASH_STATUS_COMPLETE,
    FLASH_STATUS_TIMEOUT
} FLASH_STATUS_ENUM;


void ee_hal_unlock(void)
{
}

void ee_hal_lock(void)
{
}

uint16_t ee_hal_programword(uint32_t Address, uint32_t Data)
{
    EEPROM.put(Address, Data);
    return 1;
}

uint16_t ee_hal_erasepage(uint32_t Page_Address, uint32_t Page_No)
{
    ee_hal_programword(Page_Address, EE_ERASE);
    uint16_t result = EEPROM.commit();
    delay_ms(20);
    return result;
}

//-------------------------------------------------------
// Helper
//-------------------------------------------------------

// this function is only for internal use
// if data != NULL: write data to specified page
// if data == NULL: copy data from other page to specified page
EE_STATUS_ENUM _ee_write_to(uint16_t ToPage, void* data, uint16_t datalen)
{
EE_STATUS_ENUM status;
uint16_t n;
uint32_t ToPageBaseAddress, ToPageEndAddress, FromPageBaseAddress, PageNo, adr;

    if (ToPage == EE_PAGE1) {
        // Write data to page1, or copy page0 to page1
        PageNo = EE_START_PAGE1;
        ToPageBaseAddress = EE_PAGE1_BASE_ADDRESS;
        ToPageEndAddress = EE_PAGE1_END_ADDRESS;

        FromPageBaseAddress = EE_PAGE0_BASE_ADDRESS;
    } else {
        // Write data to page0, or copy page1 to page0
        PageNo = EE_START_PAGE0;
        ToPageBaseAddress = EE_PAGE0_BASE_ADDRESS;
        ToPageEndAddress = EE_PAGE0_END_ADDRESS;

        FromPageBaseAddress = EE_PAGE1_BASE_ADDRESS;
    }

    // Erase ToPage & set page status to EE_ERASE status
    if (!ee_hal_erasepage(ToPageBaseAddress, PageNo)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }

    // Write data to ToPage
    uint32_t val;

    if (data == NULL) datalen = EE_PAGE_SIZE -16 -16; // -16 to be on the safe side

    datalen = (datalen + 3)/4; // adjust datalen to be even and 32 bit, i.e. word size

    for (n = 0; n < datalen; n++) {
        if (data == NULL) {
            adr = FromPageBaseAddress + 16 + 4*n;
            EEPROM.get(adr, val); // get data from FromPage
        } else {
            val = ((uint32_t*)data)[n]; // get data from specified data buffer
        }
        if (val != (uint32_t)0xFFFFFFFF) {
            adr = ToPageBaseAddress + 16 + 4*n;
            if (adr >= ToPageEndAddress) { status = EE_STATUS_PAGE_FULL; goto QUICK_EXIT; }
            // with deffered commit we can't test the flash until commit()
            ee_hal_programword(adr, val);
        }
    }
    if (!EEPROM.commit()) {
         status = EE_STATUS_FLASH_FAIL; 
         goto QUICK_EXIT;
    }
    delay_ms(20);
    // Set ToPage status to EE_VALID_PAGE status
    ee_hal_programword(ToPageBaseAddress, EE_VALID_PAGE);
    if (!EEPROM.commit()) {
         status = EE_STATUS_FLASH_FAIL; 
         goto QUICK_EXIT;
    }
    delay_ms(20);
    status = EE_STATUS_OK;
QUICK_EXIT:
    ee_hal_lock();
    EEPROM.commit();
    return status;
}

EE_STATUS_ENUM ee_readdata(void* data, uint16_t datalen)
{
    uint16_t n;
    uint32_t adr;

    // Read data from page0
    for (n = 0; n < datalen; n++) {
        adr = EE_PAGE0_BASE_ADDRESS + 16 + n;
        if (adr >= EE_PAGE0_END_ADDRESS) return EE_STATUS_PAGE_FULL;
        ((uint8_t*)data)[n] = EEPROM.read(adr);
    }
    return EE_STATUS_OK;
}


EE_STATUS_ENUM ee_writedata(void* data, uint16_t datalen)
{
EE_STATUS_ENUM status;

    // Write data to page0
    noInterrupts();
    status = _ee_write_to(EE_PAGE0, data, datalen);
    interrupts();
    if (status != EE_STATUS_OK) return status;

    // Write data to page1
    noInterrupts();
    status = _ee_write_to(EE_PAGE1, data, datalen);
    interrupts();
    if (status != EE_STATUS_OK) return status;

    return status;
}


//-------------------------------------------------------
// API
//-------------------------------------------------------

EE_STATUS_ENUM ee_format(void)
{
  EE_STATUS_ENUM status;

    //ee_hal_unlock();

    // Erase Page0 & set page status to ERASED status
    if (!ee_hal_erasepage(EE_PAGE0_BASE_ADDRESS,EE_START_PAGE0)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }

    // Erase Page1 & set page status to ERASED status
    if (!ee_hal_erasepage(EE_PAGE1_BASE_ADDRESS,EE_START_PAGE1)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }

    status = EE_STATUS_OK;
QUICK_EXIT:
    //ee_hal_lock();
    return status;
}

EE_STATUS_ENUM ee_init(void)
{

    EEPROM.begin(EE_PAGE_SIZE*2);
    EE_STATUS_ENUM status;
    uint32_t Page0Status, Page1Status;

    EEPROM.get(EE_PAGE0_BASE_ADDRESS, Page0Status);
    EEPROM.get(EE_PAGE1_BASE_ADDRESS, Page1Status);

    // Check for invalid header states and repair if necessary
    if ((Page0Status == EE_VALID_PAGE) && (Page1Status == EE_VALID_PAGE)) {
        // everything is ok
        status = EE_STATUS_OK;
    } else
    if ((Page0Status == EE_VALID_PAGE) && (Page1Status != EE_VALID_PAGE)) {
        // page0 is ok, copy to page1
        status = _ee_write_to(EE_PAGE1, NULL, 0);
        if (status != EE_STATUS_OK) return status;
    } else
    if ((Page0Status != EE_VALID_PAGE) && (Page1Status == EE_VALID_PAGE)) {
        // page1 is ok, copy to page0
        status = _ee_write_to(EE_PAGE0, NULL, 0);
        if (status != EE_STATUS_OK) return status;
    } else {
        // both pages invalid, format and return EE_PAGE_EMPTY
        status = ee_format();
        if (status != EE_STATUS_OK) return status;
        if ((Page0Status == EE_ERASE) && (Page1Status == EE_ERASE)) {
            status = EE_STATUS_PAGE_EMPTY;
        } else {
            status = EE_STATUS_PAGE_UNDEF;
        }
    }

    return status;
}

//-------------------------------------------------------
#endif // ESPLIB_EEPROM_H