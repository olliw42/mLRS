//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down standard emulated EEPROM library
//*******************************************************
// Interface:
//
// #define EE_START_PAGE // mandatory
// #define EE_PAGE_SIZE  // optional
//
//*******************************************************
#ifndef STDSTM32_EEPROM_H
#define STDSTM32_EEPROM_H
#ifdef __cplusplus
extern "C" {
#endif


#ifndef EE_PAGE_SIZE
  #if defined STM32F1
    // <= 128 kB -> 1 kB page size
    // >= 256 kB -> 2 kB page size
    #if defined STM32F103xB
    #define EE_PAGE_SIZE  0x0400 // Page size = 1 KByte
    #endif
  #elif defined STM32F3
    #define EE_PAGE_SIZE  0x0800 // Page size = 2 KByte
  #elif defined STM32G4
    #define EE_PAGE_SIZE  0x0800 // Page size = 2 KByte
  #elif defined STM32L4
    #define EE_PAGE_SIZE  0x0800 // Page size = 2 KByte
  #endif
#endif
#ifndef EE_PAGE_SIZE
  #error NO EE_PAGE_SIZE specified!
#endif
#ifndef EE_START_PAGE
  #error NO EE_START_PAGE specified!
#endif

#if defined STM32G4 || defined STM32L4
  #ifndef EE_USE_DOUBLEWORD
    #define EE_USE_DOUBLEWORD // G4 must use DOUBLEWORD
  #endif
#endif

#if !defined EE_USE_WORD && !defined EE_USE_DOUBLEWORD
  #define EE_USE_WORD
#endif


//-------------------------------------------------------
// Defines & Types & Enums
//-------------------------------------------------------

// EEPROM start address in Flash
#define EE_START_ADDRESS          ((uint32_t)(0x08000000 + (EE_START_PAGE * EE_PAGE_SIZE)))

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


// page0/page1      |  EE_ERASE         |  EE_VALID_PAGE    |
// ----------------------------------------------------------
// EE_ERASE         |   format          |   page1 ok,       |
//                  |   return EE_Emtpy |   copy to page0   |
// ----------------------------------------------------------
// EE_VALID_PAGE    |   page0 ok,       |   page0 ok        |
//                  |   copy to page1   |   do nothing      |
// ----------------------------------------------------------


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


#if defined STM32F1
// FLASH_Program_HalfWord() is not available in stm32f1xx_hal_flash.h GRRRR
// it makes it not so easy to copy it over and modify it here

HAL_StatusTypeDef HAL_FLASH_Program_GD32F1(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
HAL_StatusTypeDef status = HAL_ERROR;
uint8_t index = 0;
uint8_t nbiterations = 0;

    status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
    if (status != HAL_OK) return status;
    if (TypeProgram == FLASH_TYPEPROGRAM_HALFWORD) {
        nbiterations = 1;
    } else if (TypeProgram == FLASH_TYPEPROGRAM_WORD) {
        nbiterations = 2;
    } else {
        nbiterations = 4;
    }
    SET_BIT(FLASH->CR, FLASH_CR_PG);
    for (index = 0; index < nbiterations; index++) {
        *(__IO uint16_t*)(Address + (2 * index)) = (uint16_t)(Data >> (16 * index)); // FLASH_Program_HalfWord( (Address + (2 * index)), (uint16_t)(Data >> (16 * index)) );
        status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
        if (status != HAL_OK) { break; }
    }
    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
    return status;
}
#endif


FLASH_STATUS_ENUM FLASH_ErasePage(uint32_t Page_Address, uint32_t Page_No)
{
#if defined STM32F1
uint32_t PageError;
HAL_StatusTypeDef status;
FLASH_EraseInitTypeDef pEraseInit = {0};

    pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    pEraseInit.Banks = FLASH_BANK_1;
    pEraseInit.PageAddress = Page_Address;
    pEraseInit.NbPages = 1;

    status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
    return (status == HAL_OK) ? FLASH_STATUS_COMPLETE : FLASH_STATUS_TIMEOUT;

#elif defined STM32F3
uint32_t PageError;
HAL_StatusTypeDef status;
FLASH_EraseInitTypeDef pEraseInit = {0};

    pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    pEraseInit.PageAddress = Page_Address;
    pEraseInit.NbPages = 1;

    status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
    return (status == HAL_OK) ? FLASH_STATUS_COMPLETE : FLASH_STATUS_TIMEOUT;

#elif defined STM32F7
    uint32_t sector = 0;

    if (Page_Address == 0x08004000) {
        sector = FLASH_SECTOR_1;
    } else
    if (Page_Address == 0x08008000) {
        sector = FLASH_SECTOR_2;
    }else {
        while (1) {};
    }

    FLASH_Erase_Sector(sector, FLASH_VOLTAGE_RANGE_3);
    return (FLASH_WaitForLastOperation(1000) == HAL_OK) ? FLASH_STATUS_COMPLETE : FLASH_STATUS_TIMEOUT;

#elif defined STM32G4 || defined STM32L4
uint32_t PageError;
HAL_StatusTypeDef status;
FLASH_EraseInitTypeDef pEraseInit = {0};

    pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    pEraseInit.Banks = FLASH_BANK_1;
    pEraseInit.Page = Page_No;
    pEraseInit.NbPages = 1;

    status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
    return (status == HAL_OK) ? FLASH_STATUS_COMPLETE : FLASH_STATUS_TIMEOUT;
#endif
}


FLASH_STATUS_ENUM FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
#if defined STM32F1
    return (HAL_FLASH_Program_GD32F1(FLASH_TYPEPROGRAM_WORD, Address, Data) == HAL_OK) ? FLASH_STATUS_COMPLETE : FLASH_STATUS_TIMEOUT;
#elif defined STM32F3
    return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data) == HAL_OK) ? FLASH_STATUS_COMPLETE : FLASH_STATUS_TIMEOUT;
#elif defined STM32F7
    return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data) == HAL_OK) ? FLASH_STATUS_COMPLETE : FLASH_STATUS_TIMEOUT;
#elif defined STM32G4
    return FLASH_STATUS_ERROR_PG;
#elif defined STM32L4
    return FLASH_STATUS_ERROR_PG;
#endif
}


FLASH_STATUS_ENUM FLASH_ProgramDoubleWord(uint32_t Address, uint64_t Data)
{
#if defined STM32F1
    return FLASH_STATUS_ERROR_PG;
#elif defined STM32F3
    return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Data) == HAL_OK) ? FLASH_STATUS_COMPLETE : FLASH_STATUS_TIMEOUT;
#elif defined STM32F7
    return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Data) == HAL_OK) ? FLASH_STATUS_COMPLETE : FLASH_STATUS_TIMEOUT;
#elif defined STM32G4
    return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Data) == HAL_OK) ? FLASH_STATUS_COMPLETE : FLASH_STATUS_TIMEOUT;
#elif defined STM32L4
    return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Data) == HAL_OK) ? FLASH_STATUS_COMPLETE : FLASH_STATUS_TIMEOUT;
#endif
}


//-------------------------------------------------------
// EEPROM HAL
//-------------------------------------------------------

void ee_hal_unlock(void)
{
    HAL_FLASH_Unlock();
}


void ee_hal_lock(void)
{
    HAL_FLASH_Lock();
}


uint16_t ee_hal_erasepage(uint32_t Page_Address, uint32_t Page_No)
{
    return (FLASH_ErasePage(Page_Address, Page_No) == FLASH_STATUS_COMPLETE) ? 1 : 0;
}


uint16_t ee_hal_programword(uint32_t Address, uint32_t Data)
{
    return (FLASH_ProgramWord(Address, Data) == FLASH_STATUS_COMPLETE) ? 1 : 0;
}


uint16_t ee_hal_programdoubleword(uint32_t Address, uint64_t Data)
{
    return (FLASH_ProgramDoubleWord(Address, Data) == FLASH_STATUS_COMPLETE) ? 1 : 0;
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

    ee_hal_unlock();

    // Erase ToPage & set page status to EE_ERASE status
    if (!ee_hal_erasepage(ToPageBaseAddress, PageNo)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }

    // Write data to ToPage
#if !defined EE_USE_WORD && !defined EE_USE_DOUBLEWORD
uint16_t val;

    if (data == NULL) datalen = EE_PAGE_SIZE-16 -16; // -16 to be on the safe side

    datalen = (datalen + 1)/2; // adjust datalen to be even and 16 bit, i.e. half-word size

    for (n = 0; n < datalen; n++) {
        if (data == NULL) {
            adr = FromPageBaseAddress + 16 + 2*n;
            val = (*(__IO uint16_t*)(adr)); // get data from FromPage
        } else {
            val = ((uint16_t*)data)[n]; // get data from specified data buffer
        }
        if (val != 0xFFFF) {
            adr = ToPageBaseAddress + 16 + 2*n;
            if (adr >= ToPageEndAddress) { status = EE_STATUS_PAGE_FULL; goto QUICK_EXIT; }
            if (!ee_hal_programhalfword(adr, val)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }
        }
    }

    // Set ToPage status to EE_VALID_PAGE status
    if (!ee_hal_programword(ToPageBaseAddress, EE_VALID_PAGE)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }

#elif defined EE_USE_WORD
uint32_t val;

    if (data == NULL) datalen = EE_PAGE_SIZE-16 -16; // -16 to be on the safe side

    datalen = (datalen + 3)/4; // adjust datalen to be even and 32 bit, i.e. word size

    for (n = 0; n < datalen; n++) {
        if (data == NULL) {
            adr = FromPageBaseAddress + 16 + 4*n;
            val = (*(__IO uint32_t*)(adr)); // get data from FromPage
        } else {
            val = ((uint32_t*)data)[n]; // get data from specified data buffer
        }
        if (val != (uint32_t)0xFFFFFFFF) {
            adr = ToPageBaseAddress + 16 + 4*n;
            if (adr >= ToPageEndAddress) { status = EE_STATUS_PAGE_FULL; goto QUICK_EXIT; }
            if (!ee_hal_programword(adr, val)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }
        }
    }

    // Set ToPage status to EE_VALID_PAGE status
    if (!ee_hal_programword(ToPageBaseAddress, EE_VALID_PAGE)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }

#else
uint64_t val;

    if (data == NULL) datalen = EE_PAGE_SIZE-16 -16; // -16 to be on the safe side

    datalen = (datalen + 7)/8; // adjust datalen to be even 64 bit aligned

    for (n = 0; n < datalen; n++) {
        if (data == NULL) {
            adr = FromPageBaseAddress + 16 + 8*n;
            val = (*(__IO uint64_t*)(adr));
        } else {
            val = ((uint64_t*)data)[n];
        }
        if (val != (uint64_t)0xFFFFFFFFFFFFFFFF) {
            adr = ToPageBaseAddress + 16 + 8*n;
            if (adr >= ToPageEndAddress) { status = EE_STATUS_PAGE_FULL; goto QUICK_EXIT; }
            if (!ee_hal_programdoubleword(adr, val)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }
        }
    }

    // Set ToPage status to EE_VALID_PAGE status
    val = (uint64_t)0xFFFFFFFF00000000 | (uint64_t)EE_VALID_PAGE;
    if (!ee_hal_programdoubleword(ToPageBaseAddress, val)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }

#endif

    status = EE_STATUS_OK;
QUICK_EXIT:
    ee_hal_lock();
    return status;
}


//-------------------------------------------------------
// API
//-------------------------------------------------------

EE_STATUS_ENUM ee_format(void)
{
  EE_STATUS_ENUM status;

    ee_hal_unlock();

    // Erase Page0 & set page status to ERASED status
    if (!ee_hal_erasepage(EE_PAGE0_BASE_ADDRESS,EE_START_PAGE0)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }

    // Erase Page1 & set page status to ERASED status
    if (!ee_hal_erasepage(EE_PAGE1_BASE_ADDRESS,EE_START_PAGE1)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }

    status = EE_STATUS_OK;
QUICK_EXIT:
    ee_hal_lock();
    return status;
}


EE_STATUS_ENUM ee_readdata(void* data, uint16_t datalen)
{
uint16_t n;
uint32_t adr;
/*
    // Read data from page0
    datalen = (datalen + 1)/2; // adjust datalen to be even
    for (n = 0; n < datalen; n++) {
        adr = EE_PAGE0_BASE_ADDRESS + 16 + 2*n;
        if (adr >= EE_PAGE0_END_ADDRESS) return EE_STATUS_PAGE_FULL;
        ((uint16_t*)data)[n] = (*(__IO uint16_t*)(adr)); // read variable data
    }
    return EE_STATUS_OK;
*/
    // Read data from page0
    for (n = 0; n < datalen; n++) {
        adr = EE_PAGE0_BASE_ADDRESS + 16 + n;
        if (adr >= EE_PAGE0_END_ADDRESS) return EE_STATUS_PAGE_FULL;
        ((uint8_t*)data)[n] = (*(__IO uint8_t*)(adr));
    }
    return EE_STATUS_OK;
}


EE_STATUS_ENUM ee_writedata(void* data, uint16_t datalen)
{
EE_STATUS_ENUM status;

    // Write data to page0
    __disable_irq();
    status = _ee_write_to(EE_PAGE0, data, datalen);
    __enable_irq();
    if (status != EE_STATUS_OK) return status;

    // Write data to page1
    __disable_irq();
    status = _ee_write_to(EE_PAGE1, data, datalen);
    __enable_irq();
    if (status != EE_STATUS_OK) return status;

    return status;
}


EE_STATUS_ENUM ee_init(void)
{
EE_STATUS_ENUM status;
uint32_t Page0Status, Page1Status;

    Page0Status = (*(__IO uint32_t*)EE_PAGE0_BASE_ADDRESS);
    Page1Status = (*(__IO uint32_t*)EE_PAGE1_BASE_ADDRESS);

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


uint32_t ee_getpagebaseaddress(void)
{
    return EE_PAGE0_BASE_ADDRESS;
}


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_EEPROM_H
