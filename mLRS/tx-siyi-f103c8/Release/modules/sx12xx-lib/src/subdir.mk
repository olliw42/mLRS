################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
C:/Users/Olli/Documents/GitHub/mlrs/mLRS/modules/sx12xx-lib/src/sx128x.cpp 

OBJS += \
./modules/sx12xx-lib/src/sx128x.o 

CPP_DEPS += \
./modules/sx12xx-lib/src/sx128x.d 


# Each subdirectory must supply rules for building sources it contributes
modules/sx12xx-lib/src/sx128x.o: C:/Users/Olli/Documents/GitHub/mlrs/mLRS/modules/sx12xx-lib/src/sx128x.cpp modules/sx12xx-lib/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

