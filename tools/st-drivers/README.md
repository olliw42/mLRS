# STM Drivers #

STM32Cube_MCU_Overall_Offer: https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer

For each MCU there are three loactions
- STM32Cube MCU Packages -> STM32Cube MCU Packages (https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer#stm32cube-mcu-packages)
- STM32Cube MCU Components -> STM32Cube CMSIS -> STM32Cube CMSIS Devices (https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer#stm32cube-cmsis)
- STM32Cube MCU Components -> STM32Cube HAL Drivers -> STM32Cube HAL Drivers (https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer#stm32cube-hal-drivers)

Each of these sources come with their own set of version numbers. In total 4 version numbers are involved:
- version of full MCU package
- version of HAL Driver
- version of CMSIS Device
- version of CMSIS Core

For instance, on the "STM32Cube CMSIS -> STM32F1 CMSIS Device files" page one may find listed:

CMSIS Device F1 | CMSIS Core |	Was delivered in the full MCU package
--------------- | ---------- |  -------------------------------------
Tag v4.3.3 | Tag v5.4.0_cm3 | Tag v1.8.4

For instance, on the "STM32Cube HAL Drivers -> STM32F1 HAL-LL Drivers page" one may find listed:

HAL Driver F1 | CMSIS Device F1 | CMSIS Core | Was delivered in the full MCU package
------------- | --------------- | ---------- | -------------------------------------
Tag v1.1.8 | Tag v4.3.3 | Tag v5.4.0_cm3 | Tag v1.8.4 (and following, if any, till next HAL tag)

The versions for one and the same item may not always be consistent across the different sources.

In this folder, the files of "STM32Cube MCU Components -> STM32Cube CMSIS -> STM32Cube CMSIS Devices" and "STM32Cube MCU Components -> STM32Cube HAL Drivers -> STM32Cube HAL Drivers" are included as git submodules. 

The CMSIS core is not included as git submodule, due to the size of the content in https://github.com/STMicroelectronics/cmsis_core, but by downloading the zip and deleting all unneeded content. Current version: @fe753d6 on Nov 21, 2022
