#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 run_make_firmwares.py
 3rd version, doesn't use make but calls gnu directly
 gave up on cmake, hence naive by hand
 version 6.08.2023
********************************************************
'''
import os
import pathlib
import shutil
import re
import sys


#-- installation dependent
# effort at finding this automatically

#ST_DIR = os.path.join("C:/",'ST','STM32CubeIDE','STM32CubeIDE','plugins')
#GNU_DIR = 'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.10.3-2021.10.win32_1.0.0.202111181127'

def findSTM32CubeIDEGnuTools(search_root):
    st_dir = ''
    st_cubeide_dir = ''
    st_cubeide_ver_nr = 0
    for file in os.listdir(search_root):
        if 'stm32cubeide' in file.lower(): # makes it work on both win and linux
            if '_' in file:
                ver = file[13:].split('.')
                ver_nr = int(ver[0])*10000 + int(ver[1])*100 + int(ver[2])
                if ver_nr > st_cubeide_ver_nr:
                    st_cubeide_ver_nr = ver_nr
                    st_cubeide_dir = file
            else:
                st_cubeide_dir = file
                st_cubeide_ver_nr = 0
    if st_cubeide_dir != '':
        if os.name == 'posix': # install paths are os dependent
            st_dir = os.path.join(search_root,st_cubeide_dir,'plugins')
        else:
            st_dir = os.path.join(search_root,st_cubeide_dir,'stm32cubeide','plugins')

    gnu_dir = ''
    for dirpath in os.listdir(st_dir):
        if os.name == 'posix': # install paths are os dependent
            if 'mcu.externaltools.gnu-tools-for-stm32' in dirpath and 'linux' in dirpath:
                gnu_dir = dirpath 
        else:
            if 'mcu.externaltools.gnu-tools-for-stm32' in dirpath and 'win32' in dirpath:
                gnu_dir = dirpath 
    
    return st_dir, gnu_dir    

if os.name == 'posix': # install paths are os dependent
    ST_DIR,GNU_DIR = findSTM32CubeIDEGnuTools(os.path.join("/opt",'st'))
else:
    ST_DIR,GNU_DIR = findSTM32CubeIDEGnuTools(os.path.join("C:/",'ST'))

if not os.path.exists(os.path.join(ST_DIR,GNU_DIR)):
    printError('ERROR: gnu-tools not found!')
    exit(1)

print('STM32CubeIDE found in:', ST_DIR)
print('gnu-tools found in:', GNU_DIR)
print('------------------------------------------------------------')


#-- GCC preliminaries

GCC_DIR = os.path.join(ST_DIR,GNU_DIR,'tools','bin')

# we need to modify the PATH so that the correct toolchain/compiler is used
# why does sys.path.insert(0,xxx) not work?
# no, not needed anymore as we can call arm-none-eabi directly
#envpath = os.environ["PATH"]
#envpath = GCC_DIR + ';' + envpath
#os.environ["PATH"] = envpath


#-- mLRS directories

MLRS_PROJECT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

MLRS_DIR = os.path.join(MLRS_PROJECT_DIR,'mLRS')

MLRS_TOOLS_DIR = os.path.join(MLRS_PROJECT_DIR,'tools')
MLRS_BUILD_DIR = os.path.join(MLRS_PROJECT_DIR,'tools','build3')


#-- current version and branch

VERSIONONLYSTR = ''
BRANCHSTR = ''

def mlrs_set_version():
    global VERSIONONLYSTR
    F = open(os.path.join(MLRS_DIR,'Common','common_conf.h'), mode='r')
    content = F.read()
    F.close()
    
    if VERSIONONLYSTR != '':
        print('VERSIONONLYSTR =', VERSIONONLYSTR)
        return

    v = re.search(r'VERSIONONLYSTR\s+"(\S+)"', content)
    if v:
        VERSIONONLYSTR = v.groups()[0]
        print('VERSIONONLYSTR =', VERSIONONLYSTR)
    else:
        print('----------------------------------------')
        print('ERROR: VERSIONONLYSTR not found')
        os.system('pause')
        exit()


def mlrs_set_branch():
    global BRANCHSTR
    import subprocess
    git_branch = subprocess.getoutput("git branch --show-current")
    if not git_branch == 'main':
        BRANCHSTR = '-'+git_branch
    if BRANCHSTR != '':
        print('BRANCHSTR =', BRANCHSTR)


#-- helper

def remake_dir(path): # os dependent
    if os.name == 'posix':
        os.system('rm -r -f '+path)
    else:
        os.system('rmdir /s /q '+path)

def make_dir(path): # os dependent
    if os.name == 'posix':
        os.system('mkdir -p '+path)
    else:
        os.system('md '+path)


def create_dir(path):
    if not os.path.exists(path):
        make_dir(path)
        
def erase_dir(path):
    if os.path.exists(path):
        remake_dir(path)

def create_clean_dir(path):
    if os.path.exists(path):
        remake_dir(path)
    make_dir(path)


def printWarning(txt):
    print('\033[93m'+txt+'\033[0m') # light Yellow


def printError(txt):
    print('\033[91m'+txt+'\033[0m') # light Red


#--------------------------------------------------
# build system
#--------------------------------------------------

#-- source & include files, HAL, CubeMX, target independ
           
MLRS_SOURCES_HAL_STM32F1 = [
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_cortex.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_dma.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_flash.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_flash_ex.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_i2c.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_pwr.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_rcc.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_rcc_ex.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_adc.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_crc.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_dac.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_dma.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_exti.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_fsmc.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_gpio.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_i2c.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_pwr.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_rcc.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_rtc.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_sdmmc.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_spi.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_tim.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_usart.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_usb.c'),
	os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_utils.c'),
    ]

MLRS_SOURCES_HAL_STM32G4 = [
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_cortex.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_dma.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_flash.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_flash_ex.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_i2c.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_i2c_ex.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_pwr.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_pwr_ex.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_rcc.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_rcc_ex.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_adc.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_comp.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_cordic.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_crc.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_crs.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_dac.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_dma.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_exti.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_fmac.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_fmc.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_gpio.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_hrtim.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_i2c.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_lptim.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_lpuart.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_opamp.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_pwr.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_rcc.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_rng.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_rtc.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_spi.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_tim.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_ucpd.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_usart.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_usb.c'),
	os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_utils.c'),
    ]

MLRS_SOURCES_HAL_STM32WL = [
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_cortex.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_flash.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_flash_ex.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_i2c.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_i2c_ex.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_pwr.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_pwr_ex.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_rcc.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_rcc_ex.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_adc.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_comp.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_crc.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_dac.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_dma.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_exti.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_gpio.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_i2c.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_lptim.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_lpuart.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_pka.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_pwr.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_rcc.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_rng.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_rtc.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_spi.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_tim.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_usart.c'),
	os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_utils.c'),
    ]
    
MLRS_SOURCES_HAL_STM32L4 = [
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_cortex.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_flash.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_flash_ex.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_i2c.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_pwr.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_pwr_ex.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_rcc.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_rcc_ex.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_adc.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_comp.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_crc.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_crs.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_dac.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_dma.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_dma2d.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_exti.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_fmc.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_gpio.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_i2c.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_lptim.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_lpuart.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_opamp.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_pka.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_pwr.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_rcc.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_rng.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_rtc.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_sdmmc.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_spi.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_swpmi.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_tim.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_usart.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_usb.c'),
	os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_utils.c'),
    ]

MLRS_SOURCES_HAL_STM32F0 = [
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_cortex.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_flash.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_flash_ex.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_i2c.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_i2c_ex.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_pwr.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_pwr_ex.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_rcc.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_rcc_ex.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_dma.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_pcd.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_pcd_ex.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_adc.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_comp.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_crc.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_crs.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_dac.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_dma.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_exti.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_gpio.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_i2c.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_pwr.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_rcc.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_rtc.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_spi.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_tim.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_usart.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_usb.c'),
	os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_utils.c'),
    ]

MLRS_SOURCES_CORE = [ # the ?? are going to be replaced with mcu_family label, f1, g4, wl, l4
	os.path.join('Core','Src','main.cpp'),
	os.path.join('Core','Src','stm32??xx_hal_msp.c'),
	os.path.join('Core','Src','stm32??xx_it.c'),
	os.path.join('Core','Src','syscalls.c'),
	os.path.join('Core','Src','sysmem.c'),
	os.path.join('Core','Src','system_stm32??xx.c'),
    ]

MLRS_INCLUDES = [ # the ?? are going to be replaced with mcu_HAL label, STM32F1xx, STM32G4xx, STM32WLxx, STM32L4xx
    os.path.join('Core','Inc'),
    os.path.join('Drivers','??_HAL_Driver','Inc'),
    os.path.join('Drivers','??_HAL_Driver','Inc','Legacy'),
    os.path.join('Drivers','CMSIS','Device','ST','??','Include'),
    os.path.join('Drivers','CMSIS','Include'),
    ]


#-- source & include files, target independent/common 

MLRS_SOURCES_MODULES = [
	os.path.join('modules','sx12xx-lib','src','sx126x.cpp'),
	os.path.join('modules','sx12xx-lib','src','sx127x.cpp'),
	os.path.join('modules','sx12xx-lib','src','sx128x.cpp'),
	os.path.join('modules','stm32ll-lib','src','stdstm32.c'),
    ]

MLRS_SOURCES_COMMON = [
    os.path.join('Common','thirdparty','thirdparty.cpp'),
	os.path.join('Common','libs','filters.cpp'),
	os.path.join('Common','libs','gdisp.c'),
	os.path.join('Common','channel_order.cpp'),
	os.path.join('Common','common_types.cpp'),
	os.path.join('Common','fhss.cpp'),
	os.path.join('Common','while.cpp'),
	]

MLRS_SOURCES_RX = [
	os.path.join('CommonRx','out.cpp'),
	os.path.join('CommonRx','mlrs-rx.cpp'),
    ]

MLRS_SOURCES_TX = [
	os.path.join('CommonTx','in.cpp'),
	os.path.join('CommonTx','config_id.cpp'),
	os.path.join('CommonTx','mlrs-tx.cpp'),
    ]


MLRS_SOURCES_USB = [
	os.path.join('Drivers','STM32_USB_Device_Library','Class','CDC','Src','usbd_cdc.c'),
	os.path.join('Drivers','STM32_USB_Device_Library','Core','Src','usbd_core.c'),
	os.path.join('Drivers','STM32_USB_Device_Library','Core','Src','usbd_ctlreq.c'),
	os.path.join('Drivers','STM32_USB_Device_Library','Core','Src','usbd_ioreq.c'),
	os.path.join('..','modules','stm32-usb-device','usbd_cdc_if.c'),
	os.path.join('..','modules','stm32-usb-device','usbd_conf.c'),
	os.path.join('..','modules','stm32-usb-device','usbd_desc.c'),
    ]

MLRS_INCLUDES_USB = [
	os.path.join('Drivers','STM32_USB_Device_Library','Class','CDC','Inc'),
	os.path.join('Drivers','STM32_USB_Device_Library','Core','Inc'),
	os.path.join('..','modules','stm32-usb-device'),
    ]



#-- target class to handle targets
# target:       name of target
# target_D:     define for target in the code
# mcu_D:        define for mcu in  command line, e.g. -DSTM32WLE5xx
# mcu_HAL:      mcu part in the folder name to HAL drivers, e.g. .../Drivers/STM32WLxx_HAL_Driver/...      

class cTarget:
    def __init__(self, target, target_D, mcu_D, mcu_HAL, startup_script, linker_script, mcu_option_list, extra_D_list, build_dir, elf_name):
        self.target = target
        self.target_D = target_D
        self.mcu_D = mcu_D
        self.mcu_HAL = mcu_HAL
        self.startup_script = startup_script
        self.linker_script = linker_script
        self.mcu_option_list = mcu_option_list
        self.extra_D_list = extra_D_list
        self.build_dir = build_dir
        self.elf_name = elf_name

        self.mcu_family = ''
        if 'F1' in self.mcu_D and 'F1' in self.mcu_HAL:
            self.mcu_family = 'f1'
        elif 'G4' in self.mcu_D and 'G4' in self.mcu_HAL:
            self.mcu_family = 'g4'
        elif 'L4' in self.mcu_D and 'L4' in self.mcu_HAL:
            self.mcu_family = 'l4'
        elif 'WL' in self.mcu_D and 'WL' in self.mcu_HAL:
            self.mcu_family = 'wl'
        elif 'F0' in self.mcu_D and 'F0' in self.mcu_HAL:
            self.mcu_family = 'f0'
        else:
            printError('SHSHHSKHSKHSKHKSHKSHKH')
            print('mcu_D',self.mcu_D)
            print('mcu_HAL',self.mcu_HAL)
            exit(1)
            
        self.rx_or_tx = ''
        self.is_rx = False
        self.is_tx = False
        if 'rx' in target and 'RX' in target_D:
            self.rx_or_tx = 'rx'
            self.is_rx = True
        elif 'tx' in target and 'TX' in target_D:
            self.rx_or_tx = 'tx'
            self.is_tx = True
        else:
            printError('gkgkggjkgjkgjkgjgjgjgjg')
            exit(1)
            
        self.D_list = ['USE_HAL_DRIVER', 'USE_FULL_LL_DRIVER']
        if self.mcu_family == 'wl':
            self.D_list.append('CORE_CM4')
            
        self.MLRS_SOURCES_HAL = []
        if self.mcu_family == 'f1':
            self.MLRS_SOURCES_HAL = MLRS_SOURCES_HAL_STM32F1
        elif self.mcu_family == 'g4':
            self.MLRS_SOURCES_HAL = MLRS_SOURCES_HAL_STM32G4
        elif self.mcu_family == 'wl':
            self.MLRS_SOURCES_HAL = MLRS_SOURCES_HAL_STM32WL
        elif self.mcu_family == 'l4':
            self.MLRS_SOURCES_HAL = MLRS_SOURCES_HAL_STM32L4
        elif self.mcu_family == 'f0':
            self.MLRS_SOURCES_HAL = MLRS_SOURCES_HAL_STM32F0
        
        self.MLRS_SOURCES_CORE = []
        for file in MLRS_SOURCES_CORE:
            self.MLRS_SOURCES_CORE.append(file.replace('??',self.mcu_family))
            
        self.MLRS_INCLUDES = []
        for file in MLRS_INCLUDES:
            self.MLRS_INCLUDES.append(file.replace('??',self.mcu_HAL))

        self.MLRS_SOURCES_EXTRA = []
        if 'STDSTM32_USE_USB' in self.extra_D_list:
            for file in MLRS_SOURCES_USB:
                self.MLRS_SOURCES_EXTRA.append(file)
            for file in MLRS_INCLUDES_USB:
                self.MLRS_INCLUDES.append(file)
        else: # add stm32-usb-device sources to every target (we could have excluded them in the IDE, but are too lazy LOL)
            for file in MLRS_SOURCES_USB:
                if 'modules' in file:
                    self.MLRS_SOURCES_EXTRA.append(file)
                    

#-- compiler & linker

def mlrs_compile_file(target, file):
    file_path = os.path.dirname(file)
    file_name = os.path.splitext(file)[0]
    file_ext = os.path.splitext(file)[1]
    #print(file_path)
    #print(file_name)
    #print(file_ext)
    
    is_cpp = False
    is_asm = False
    if file_ext == '.cpp': is_cpp = True
    if file_ext == '.s': is_asm = True

#    if is_cpp:
#        print('g++',file)
#    else:    
#        print('gcc',file)

    # construct command line    
    cmd = ''
    if is_cpp:
        cmd = os.path.join(GCC_DIR,'arm-none-eabi-g++ ')
    else:
        cmd = os.path.join(GCC_DIR,'arm-none-eabi-gcc ')

    if not is_asm:    
        cmd += '"'+os.path.join(MLRS_DIR,file)+'" '
        if is_cpp:
            cmd += '-std=gnu++14 '
        else:
            cmd += '-std=gnu11 '
    
    cmd += '-c '
    
    for mcu_option in target.mcu_option_list:
        cmd += mcu_option+' '
    cmd += '-mthumb '
    cmd += '--specs=nano.specs '
    
    if not is_asm:
        for d in target.D_list:
            cmd += '-D'+d+' '
        cmd += '-D'+target.mcu_D+' '
        if is_cpp:
            cmd += '-D'+target.target_D+' '
        for extra_D in target.extra_D_list:
            cmd += '-D'+extra_D+' '

        for file in target.MLRS_INCLUDES:
            cmd += '-I'+os.path.join(MLRS_DIR,target.target,file)+' '
   
        cmd += '-Os '
        cmd += '-ffunction-sections -fdata-sections -Wall -fstack-usage '
        if is_cpp:
            cmd += '-fno-exceptions -fno-rtti -fno-use-cxa-atexit '
    else:
        cmd += '-x assembler-with-cpp '
    
    cmd += '-MMD -MP '
    cmd += '-MF"'+os.path.join(MLRS_BUILD_DIR,target.build_dir,file_name)+'.d" '
    cmd += '-MT"'+os.path.join(MLRS_BUILD_DIR,target.build_dir,file_name)+'.o" '

    cmd += '-o "'+os.path.join(MLRS_BUILD_DIR,target.build_dir,file_name)+'.o" '
    
    if is_asm:    
        cmd += '"'+os.path.join(MLRS_DIR,file)+'" ' # asm needs it at end
    
    #print(cmd)
    #if not is_asm: exit(1)
    
    # create folder as needed
    buildpath = os.path.join(MLRS_BUILD_DIR,target.build_dir,file_path)
    create_dir(buildpath)
    
    # execute
    #print('run')
    os.system(cmd)


def mlrs_link_target(target):
    # generate object list from actually created .o files
    objlist = []
    for path, subdirs, files in os.walk(os.path.join(MLRS_BUILD_DIR,target.build_dir)):
        for file in files:
            if os.path.splitext(file)[1] == '.o':
                obj = os.path.join(path,file).replace(os.path.join(MLRS_BUILD_DIR,target.build_dir), '')
                objlist.append(obj.replace('\\','/'))
    
    # check for a STM32CubeIDE object list
    objlist_cube_file = os.path.join(MLRS_DIR,target.target,'Release','objects.list')
    if not os.path.exists(objlist_cube_file):
        # generate object list
        printWarning('WARNING: no objects.list found, use generated objects.list')
        F = open(os.path.join(MLRS_BUILD_DIR,target.build_dir,'objects.list'), mode='w')
        for obj in sorted(objlist): # we use sorted, this at least makes it that it is somehow standardized, thus repeatable            
            F.write('"'+os.path.join(MLRS_BUILD_DIR,target.build_dir).replace('\\','/')+obj+'"\n')
        F.close()
    else :
        # read STM32CubeIDE object list, and modify paths accordingly
        print('use available objects.list')
        F = open(objlist_cube_file, mode='r')
        objlist_cube = F.read()
        F.close()
        objlist_cube_list = objlist_cube.split()
        # correct list
        objlist_cube_list_corrected = []
        for obj in objlist_cube_list:
            #o = obj.replace('./', os.path.join(MLRS_BUILD_DIR,target.build_dir)+'/')
            o = obj.replace('./', '/').replace('"', '')
            if obj[3:7] == 'Core':
                o = o.replace('/Core', '/'+target.target+'/Core')
            if obj[3:10] == 'Drivers':
                o = o.replace('/Drivers', '/'+target.target+'/Drivers')
            objlist_cube_list_corrected.append(o.replace('\\','/'))
        # check lists
        for obj in objlist:
            if not obj in objlist_cube_list_corrected:
                printWarning('objlist: '+obj+' not in Cube objlist')
        for obj in objlist_cube_list_corrected:
            if not obj in objlist:
                printWarning('Cube objlist: '+obj+' not in objlist')
        # write out
        F = open(os.path.join(MLRS_BUILD_DIR,target.build_dir,'objects.list'), mode='w')
        for obj in objlist_cube_list_corrected:
            F.write('"'+os.path.join(MLRS_BUILD_DIR,target.build_dir).replace('\\','/')+obj+'"\n')
        F.close()
        
    # generate command line
    cmd = ''
    cmd += os.path.join(GCC_DIR,'arm-none-eabi-g++') + ' '
    cmd += '-o "'+os.path.join(MLRS_BUILD_DIR,target.build_dir,target.elf_name+'.elf')+'" '
    cmd += '@"'+os.path.join(MLRS_BUILD_DIR,target.build_dir,'objects.list')+'" '
    cmd += '-T"'+os.path.join(MLRS_DIR,target.target,target.linker_script)+'" ' 
    for mcu_option in target.mcu_option_list:
        cmd += mcu_option+' '
    cmd += '-mthumb '
    cmd += '--specs=nano.specs '
    cmd += '--specs=nosys.specs '
    cmd += '-static '
    cmd += '-Wl,-Map="'+os.path.join(MLRS_BUILD_DIR,target.build_dir,target.target+'.map')+'" '
    cmd += '-Wl,--gc-sections '
    cmd += '-Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group '

    #print(cmd)

    #print('run')
    os.system(cmd)


def mlrs_build_target(target, cmdline_D_list):
    if cmdline_D_list != []:
        #target.extra_D_list = cmdline_D_list
        target.extra_D_list += cmdline_D_list
    print('------------------------------------------------------------')
    print('target', target.target, target.extra_D_list)

    buildpath = os.path.join(MLRS_BUILD_DIR,target.build_dir)
    #erase_dir(buildpath)
    create_clean_dir(buildpath)
    
#    mlrs_compile_file(target, MLRS_SOURCES_MODULES[0])
#    return
#    mlrs_compile_file(target, os.path.join(target.target,MLRS_SOURCES_HAL_STM32F1[0]))
#    mlrs_compile_file(target, os.path.join(target.target,MLRS_STARTUP_SCRIPT_STM32F1[0]))

    print('compiling')
    
    mlrs_compile_file(target, os.path.join(target.target,'Core','Startup',target.startup_script))

    for file in target.MLRS_SOURCES_HAL:
        mlrs_compile_file(target, os.path.join(target.target,file))

    for file in target.MLRS_SOURCES_CORE:
        mlrs_compile_file(target, os.path.join(target.target,file))

    for file in MLRS_SOURCES_MODULES:
        mlrs_compile_file(target, file)
        
    for file in MLRS_SOURCES_COMMON:
        mlrs_compile_file(target, file)
        
    for file in target.MLRS_SOURCES_EXTRA:
        mlrs_compile_file(target, os.path.join(target.target,file))
        
    MLRS_SOURCES_RXTX = []
    if target.rx_or_tx == 'rx':
        MLRS_SOURCES_RXTX = MLRS_SOURCES_RX
    elif target.rx_or_tx == 'tx':
        MLRS_SOURCES_RXTX = MLRS_SOURCES_TX
    else:
        printError('aköhdfkahsfkuhafkhasfkdh')
        exit(1)
    for file in MLRS_SOURCES_RXTX:
        mlrs_compile_file(target, file)

    print('linking')

    mlrs_link_target(target)
    os.system(os.path.join(GCC_DIR,'arm-none-eabi-size')+' '+os.path.join(MLRS_BUILD_DIR,target.build_dir,target.elf_name+'.elf'))
    
    os.system(
        os.path.join(GCC_DIR,'arm-none-eabi-objcopy') + ' -O ihex ' +
        os.path.join(MLRS_BUILD_DIR,target.build_dir,target.elf_name+'.elf') + ' ' +
        os.path.join(MLRS_BUILD_DIR,target.build_dir,target.elf_name+'.hex')
        )

    print('------------------------------------------------------------')


#-- mcu family generic targets

class cTargetF1(cTarget):
    def __init__(self, target, target_D, mcu_D, startup_script, linker_script, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            mcu_D, 'STM32F1xx', 
            startup_script, linker_script, 
            ['-mcpu=cortex-m3', '-mfloat-abi=soft'], 
            extra_D_list, build_dir, elf_name)

class cTargetG4(cTarget):
    def __init__(self, target, target_D, mcu_D, startup_script, linker_script, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            mcu_D, 'STM32G4xx', 
            startup_script, linker_script, 
            ['-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard'],
            extra_D_list, build_dir, elf_name)

class cTargetWL(cTarget):
    def __init__(self, target, target_D, mcu_D, startup_script, linker_script, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            mcu_D, 'STM32WLxx', 
            startup_script, linker_script, 
            ['-mcpu=cortex-m4', '-mfloat-abi=soft'],
            extra_D_list, build_dir, elf_name)

class cTargetL4(cTarget):
    def __init__(self, target, target_D, mcu_D, startup_script, linker_script, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            mcu_D, 'STM32L4xx', 
            startup_script, linker_script, 
            ['-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard'],
            extra_D_list, build_dir, elf_name)

class cTargetF0(cTarget):
    def __init__(self, target, target_D, mcu_D, startup_script, linker_script, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            mcu_D, 'STM32F0xx', 
            startup_script, linker_script, 
            ['-mcpu=cortex-m0', '-mfloat-abi=soft'],
            extra_D_list, build_dir, elf_name)


#-- mcu specific targets

class cTargetF103C8(cTargetF1):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            'STM32F103xB', 'startup_stm32f103c8tx.s', 'STM32F103C8TX_FLASH.ld', 
            extra_D_list, build_dir, elf_name)

class cTargetF103CB(cTargetF1):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            'STM32F103xB', 'startup_stm32f103cbtx.s', 'STM32F103CBTX_FLASH.ld', 
            extra_D_list, build_dir, elf_name)

class cTargetF103RB(cTargetF1):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            'STM32F103xB', 'startup_stm32f103rbhx.s', 'STM32F103RBHX_FLASH.ld', 
            extra_D_list, build_dir, elf_name)


class cTargetG431KB(cTargetG4):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            'STM32G431xx', 'startup_stm32g431kbux.s', 'STM32G431KBUX_FLASH.ld',
            extra_D_list, build_dir, elf_name)

class cTargetG441KB(cTargetG4): #is code compatible to G431KB!?
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            'STM32G441xx', 'startup_stm32g441kbux.s', 'STM32G441KBUX_FLASH.ld',
            extra_D_list, build_dir, elf_name)

class cTargetG491RE(cTargetG4):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            'STM32G491xx', 'startup_stm32g491retx.s', 'STM32G491RETX_FLASH.ld',
            extra_D_list, build_dir, elf_name)


class cTargetWLE5CC(cTargetWL):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            'STM32WLE5xx', 'startup_stm32wle5ccux.s', 'STM32WLE5CCUX_FLASH.ld',
            extra_D_list, build_dir, elf_name)
            
class cTargetWLE5JC(cTargetWL):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            'STM32WLE5xx', 'startup_stm32wle5jcix.s', 'STM32WLE5JCIX_FLASH.ld',
            extra_D_list, build_dir, elf_name)

    
class cTargetL433CBUX(cTargetL4):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            'STM32L433xx', 'startup_stm32l433cbux.s', 'STM32L433CBUX_FLASH.ld',
            extra_D_list, build_dir, elf_name)
            
class cTargetL433CBYX(cTargetL4): # identical to CBUX, but unfortunately used
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            'STM32L433xx', 'startup_stm32l433cbyx.s', 'STM32L433CBYX_FLASH.ld',
            extra_D_list, build_dir, elf_name)


class cTargetF070CB(cTargetF0):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D, 
            'STM32F070xB', 'startup_stm32f070cbtx.s', 'STM32F070CBTX_FLASH.ld',
            extra_D_list, build_dir, elf_name)


#--------------------------------------------------
# application
#--------------------------------------------------
    
#-- list of targets

TLIST = [
    { 
#RX    
        'target' : 'rx-diy-board01-f103cb',             'target_D' : 'RX_DIY_BOARD01_F103CB',
        'extra_D_list' : [], 'appendix' : '' 
    },{
        'target' : 'rx-diy-e22-g441kb',                 'target_D' : 'RX_DIY_E22_G441KB', 
        'extra_D_list' : [], 'appendix' : '' 
    },{ 
        'target' : 'rx-diy-e28dual-board02-f103cb',     'target_D' : 'RX_DIY_E28DUAL_BOARD02_F103CB',
        'extra_D_list' : [], 'appendix' : '' 
    },{ 
        'target' : 'rx-diy-e28-g441kb',                 'target_D' : 'RX_DIY_E28_G441KB',
        'extra_D_list' : [], 'appendix' : '' 
    },{ 
        'target' : 'rx-diy-WioE5-E22-dual-wle5jc',      'target_D' : 'RX_DIY_WIOE5_E22_WLE5JC',
        'extra_D_list' : [], 'appendix' : '' 
    },{
        'target' : 'rx-E77-MBLKit-wle5cc',              'target_D' : 'RX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_868_MHZ','MLRS_FEATURE_915_MHZ_FCC'],
        'appendix' : '-900' 
    },{
        'target' : 'rx-E77-MBLKit-wle5cc',              'target_D' : 'RX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_433_MHZ'], 
        'appendix' : '-400' 
    },{
        'target' : 'rx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'RX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_NO_DIVERSITY'],
        'appendix' : '' 
    },{
        'target' : 'rx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'RX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_DIVERSITY'],
        'appendix' : '-diversity' 
    },{ 
        'target' : 'rx-R9M-f103c8',                     'target_D' : 'RX_R9M_868_F103C8',
        'extra_D_list' : [], 'appendix' : '' 
    },{ 
        'target' : 'rx-R9MM-f103rb',                    'target_D' : 'RX_R9MM_868_F103RB',
        'extra_D_list' : [], 'appendix' : '' 
    },{ 
        'target' : 'rx-R9MX-l433cb',                    'target_D' : 'RX_R9MX_868_L433CB',
        'extra_D_list' : [], 'appendix' : '' 
    },{ 
        'target' : 'rx-Wio-E5-Grove-wle5jc',            'target_D' : 'RX_WIO_E5_GROVE_WLE5JC',
        'extra_D_list' : [], 'appendix' : '' 
    },{
        'target' : 'rx-Wio-E5-Mini-wle5jc',             'target_D' : 'RX_WIO_E5_MINI_WLE5JC',
        'extra_D_list' : [], 'appendix' : '' 
    },{
        'target' : 'rx-FRM303-f070cb',                  'target_D' : 'RX_FRM303_F070CB',
        'extra_D_list' : [], 'appendix' : '',
    },{
    
#TX    
        'target' : 'tx-diy-e22dual-module02-g491re',    'target_D' : 'TX_DIY_E22DUAL_MODULE02_G491RE', 
        'extra_D_list' : [] , 'appendix' : '' 
    },{ 
        'target' : 'tx-diy-e22-g431kb',                 'target_D' : 'TX_DIY_E22_G431KB',
        'extra_D_list' : [], 'appendix' : ''  
    },{
        'target' : 'tx-diy-e28dual-board02-f103cb',     'target_D' : 'TX_DIY_E28DUAL_BOARD02_F103CB',
        'extra_D_list' : [], 'appendix' : '' 
    },{
        'target' : 'tx-diy-e28dual-module02-g491re',    'target_D' : 'TX_DIY_E28DUAL_MODULE02_G491RE', 
        'extra_D_list' : [], 'appendix' : '' 
    },{
        'target' : 'tx-diy-sxdual-module02-g491re',     'target_D' : 'TX_DIY_SXDUAL_MODULE02_G491RE', 
        'extra_D_list' : [], 'appendix' : '' 
    },{ 
        'target' : 'tx-diy-WioE5-E22-dual-wle5jc',      'target_D' : 'TX_DIY_WIOE5_E22_WLE5JC',
        'extra_D_list' : [], 'appendix' : '' 
        
    },{
        'target' : 'tx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'TX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_NO_DIVERSITY','MLRS_FEATURE_JRPIN5'],
        'appendix' : '-jrpin5' 
    },{
        'target' : 'tx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'TX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_DIVERSITY','MLRS_FEATURE_JRPIN5'],
        'appendix' : '-jrpin5-diversity'
    },{
        'target' : 'tx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'TX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_NO_DIVERSITY','MLRS_FEATURE_IN'],
        'appendix' : '-in' 
    },{
        'target' : 'tx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'TX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_DIVERSITY','MLRS_FEATURE_IN'],
        'appendix' : '-in-diversity'
        
    },{
        'target' : 'tx-E77-MBLKit-wle5cc',              'target_D' : 'TX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_868_MHZ','MLRS_FEATURE_915_MHZ_FCC'],
        'appendix' : '-900' 
    },{
        'target' : 'tx-E77-MBLKit-wle5cc',              'target_D' : 'TX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_433_MHZ'], 
        'appendix' : '-400' 
    },{ 
        'target' : 'tx-R9M-f103c8',                     'target_D' : 'TX_R9M_868_F103C8',
        'extra_D_list' : [], 'appendix' : '' 
    },{ 
        'target' : 'tx-R9MX-l433cb',                    'target_D' : 'TX_R9MX_868_L433CB',
        'extra_D_list' : [], 'appendix' : '' 
    },{
        'target' : 'tx-Wio-E5-Mini-wle5jc',             'target_D' : 'TX_WIO_E5_MINI_WLE5JC',
        'extra_D_list' : [], 'appendix' : '' 
        
    },{
        'target' : 'tx-E77-MBLKit-wle5cc',              'target_D' : 'TX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_868_MHZ','MLRS_FEATURE_915_MHZ_FCC','MLRS_DEV_FEATURE_JRPIN5_SDIODE'],
        'appendix' : '-900-sdiode' 
    },{
        'target' : 'tx-E77-MBLKit-wle5cc',              'target_D' : 'TX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_433_MHZ','MLRS_DEV_FEATURE_JRPIN5_SDIODE'], 
        'appendix' : '-400-sdiode' 
    },{
        'target' : 'tx-Wio-E5-Mini-wle5jc',             'target_D' : 'TX_WIO_E5_MINI_WLE5JC',
        'extra_D_list' : ['MLRS_DEV_FEATURE_JRPIN5_SDIODE'], 
        'appendix' : '-sdiode' 
        
    },{
        'target' : 'tx-FRM303-f070cb',                  'target_D' : 'TX_FRM303_F070CB',
        'extra_D_list' : ['STDSTM32_USE_USB'],
        'appendix' : '-usb',
    },{
        'target' : 'tx-FRM303-f070cb',                  'target_D' : 'TX_FRM303_F070CB',
        'extra_D_list' : ['STDSTM32_USE_USB','MLRS_FEATURE_OLED'],
        'appendix' : '-oled',
        
    }
    ]


def mlrs_create_targetlist(appendix, extra_D_list):
    tlist = []
    for t in TLIST:
        build_dir = t['target']+t['appendix']
        elf_name = t['target']+t['appendix']+appendix
        if 'f103c8' in t['target']:
            tlist.append( cTargetF103C8(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name) )
        elif 'f103cb' in t['target']:
            tlist.append( cTargetF103CB(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name) )
        elif 'f103rb' in t['target']:
            tlist.append( cTargetF103RB(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name) )
        elif 'g431kb' in t['target']:
            tlist.append( cTargetG431KB(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name) )
        elif 'g441kb' in t['target']:
            tlist.append( cTargetG441KB(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name) )
        elif 'g491re' in t['target']:
            tlist.append( cTargetG491RE(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name) )
        elif 'wle5cc' in t['target']:
            tlist.append( cTargetWLE5CC(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name) )
        elif 'wle5jc' in t['target']:
            tlist.append( cTargetWLE5JC(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name) )
        elif 'l433cb' in t['target']:
            # grrr, sad that ux and yx are used
            if 'rx' in t['target']:
                tlist.append( cTargetL433CBYX(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name) )
            else:
                tlist.append( cTargetL433CBUX(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name) )
        elif 'f070cb' in t['target']:
            tlist.append( cTargetF070CB(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name) )
        else:
            print('<ljfl<iasdjfljsdfljlsdjfljsdlfjlsdjf')
            exit(1)
    return tlist


def mlrs_copy_all_hex():
    print('copying .hex files')
    firmwarepath = os.path.join(MLRS_BUILD_DIR,'firmware')
    create_clean_dir(firmwarepath)
    for path, subdirs, files in os.walk(MLRS_BUILD_DIR):
        for file in files:
            if 'firmware' in path:
                continue
            if os.path.splitext(file)[1] == '.hex':
                shutil.copy(os.path.join(path,file), os.path.join(firmwarepath,file))


#-- here we go

cmdline_target = ''
cmdline_D_list = []

cmd_pos = -1
for cmd in sys.argv:
    cmd_pos += 1
    if cmd == '--target' or cmd == '-t' or cmd == '-T':
        if sys.argv[cmd_pos+1] != '':
            cmdline_target = sys.argv[cmd_pos+1]
    if cmd == '--define' or cmd == '-d' or cmd == '-D':
        if sys.argv[cmd_pos+1] != '':
            cmdline_D_list.append(sys.argv[cmd_pos+1])
        
#cmdline_target = 'tx-diy-e22dual-module02-g491re'
#cmdline_target = 'tx-diy-sxdualXXX'

mlrs_set_version()
mlrs_set_branch()

create_clean_dir(MLRS_BUILD_DIR)

targetlist = mlrs_create_targetlist(BRANCHSTR+'-'+VERSIONONLYSTR, [])

target_cnt = 0
for target in targetlist:
    if ((cmdline_target == '') or
        (cmdline_target[0] != '!' and cmdline_target in target.target) or
        (cmdline_target[0] == '!' and not cmdline_target[1:] in target.target)):
        mlrs_build_target(target, cmdline_D_list)
        target_cnt +=1 
        
if cmdline_target == '' or target_cnt > 0: 
    mlrs_copy_all_hex()

os.system("pause")
