#*******************************************************
# Copyright (c) MLRS project
# GPL3
# https://www.gnu.org/licenses/gpl-3.0.de.html
#*******************************************************
# mLRS RX OTA loader, STM32CubeIDE
#*******************************************************
# Included by the makefile.targets of the targets with DEVICE_HAS_OTA_LOADER, runs in <target>/Release. Builds
# the loader into ota_loader.bin, which ota_loader.cpp puts into the app. tools/run_make_firmwares.py does the
# same, keep the sources in sync. Names have the OTA_LOADER_ prefix, as they share the makefile with CubeIDE's.
# The files are flat in Release with an ota-loader- prefix, so the recipes need nothing but the toolchain.
#*******************************************************

OTA_LOADER_HAL ?= STM32G4xx
OTA_LOADER_MCU_FLAGS ?= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard

OTA_LOADER_SRCS = $(wildcard ../Core/Startup/*.s ../Core/Src/*.c ../Core/Src/*.cpp ../Drivers/$(OTA_LOADER_HAL)_HAL_Driver/Src/*.c)
OTA_LOADER_SRCS += \
  ../../modules/sx12xx-lib/src/sx126x.cpp \
  ../../modules/sx12xx-lib/src/sx127x.cpp \
  ../../modules/sx12xx-lib/src/sx128x.cpp \
  ../../modules/sx12xx-lib/src/lr11xx.cpp \
  ../../modules/sx12xx-lib/src/lr20xx.cpp \
  ../../modules/stm32ll-lib/src/stdstm32.c \
  ../../Common/libs/utils.cpp \
  ../../CommonRx/ota_loader.cpp

OTA_LOADER_FLAGS = $(OTA_LOADER_MCU_FLAGS) -mthumb --specs=nano.specs
OTA_LOADER_CFLAGS = $(OTA_LOADER_FLAGS) -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -D$(MCU_D) -DMLRS_OTA_LOADER_BUILD \
  -I../Core/Inc -I../Drivers/$(OTA_LOADER_HAL)_HAL_Driver/Inc -I../Drivers/$(OTA_LOADER_HAL)_HAL_Driver/Inc/Legacy \
  -I../Drivers/CMSIS/Device/ST/$(OTA_LOADER_HAL)/Include -I../Drivers/CMSIS/Include \
  -Os -ffunction-sections -fdata-sections -Wall -MMD -MP
OTA_LOADER_CXXFLAGS = $(OTA_LOADER_CFLAGS) -D$(TARGET_D) -std=gnu++14 -fno-exceptions -fno-rtti -fno-use-cxa-atexit

ota_loader_obj = ota-loader-$(notdir $(basename $(1))).o
OTA_LOADER_OBJS = $(foreach src,$(OTA_LOADER_SRCS),$(call ota_loader_obj,$(src)))

# the app's ota_loader.cpp has the loader's image
CommonRx/ota_loader.o: ota_loader.bin

ota_loader.bin: ota-loader.elf
	arm-none-eabi-objcopy -O binary $< $@
	arm-none-eabi-size $<

ota-loader.elf: $(OTA_LOADER_OBJS)
	arm-none-eabi-g++ -o $@ $^ -T$(wildcard ../*_FLASH.ld) $(OTA_LOADER_FLAGS) --specs=nosys.specs -static \
	  -Wl,-Map=ota-loader.map -Wl,--gc-sections -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group

define ota_loader_compile
$(call ota_loader_obj,$(1)): $(1)
	$(if $(filter %.s,$(1)),arm-none-eabi-gcc $(OTA_LOADER_FLAGS) -x assembler-with-cpp,$(if $(filter %.cpp,$(1)),arm-none-eabi-g++ $(OTA_LOADER_CXXFLAGS),arm-none-eabi-gcc $(OTA_LOADER_CFLAGS) -std=gnu11)) -c $$< -o $$@
endef
$(foreach src,$(OTA_LOADER_SRCS),$(eval $(call ota_loader_compile,$(src))))

-include $(OTA_LOADER_OBJS:.o=.d)
