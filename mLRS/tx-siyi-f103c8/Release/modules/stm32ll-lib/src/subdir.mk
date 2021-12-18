################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Olli/Documents/GitHub/mlrs/mLRS/modules/stm32ll-lib/src/stdstm32.c 

C_DEPS += \
./modules/stm32ll-lib/src/stdstm32.d 

OBJS += \
./modules/stm32ll-lib/src/stdstm32.o 


# Each subdirectory must supply rules for building sources it contributes
modules/stm32ll-lib/src/stdstm32.o: C:/Users/Olli/Documents/GitHub/mlrs/mLRS/modules/stm32ll-lib/src/stdstm32.c modules/stm32ll-lib/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

