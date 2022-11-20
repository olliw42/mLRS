#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 run_copy_st_drivers.py
 copy CMSIS and HAL files to project target folders
********************************************************
'''
import os
import shutil
import re
import sys


mLRSProjectdirectory = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
mLRSdirectory = os.path.join(mLRSProjectdirectory,'mLRS')


def delete_files_in_dir(directory):
    dirlist = os.listdir(directory)
    for f in dirlist:
        if os.path.isfile(os.path.join(directory,f)):
            os.remove(os.path.join(directory,f))


def delete_dir(directory):
    dirlist = os.listdir(directory)
    for f in dirlist:
        if os.path.isfile(os.path.join(directory,f)):
            os.remove(os.path.join(directory,f))
        if os.path.isdir(os.path.join(directory,f)):
            delete_files_in_dir(os.path.join(directory,f))


def copy_files_in_dir(target, source):
    dirlist = os.listdir(source)
    for f in dirlist:
        if os.path.isfile(os.path.join(source,f)):
            shutil.copy(os.path.join(source,f), target)


def copy_dir(target, source):
    dirlist = os.listdir(source)
    for f in dirlist:
        if os.path.isfile(os.path.join(source,f)):
            shutil.copy(os.path.join(source,f), target)
        if os.path.isdir(os.path.join(source,f)):
            if not os.path.exists(os.path.join(target,f)):
                os.makedirs(os.path.join(target,f))
            copy_files_in_dir(os.path.join(target,f),os.path.join(source,f))


def copy_wexclude(target, source, f, files_to_exclude=[]):
    if os.path.isfile(os.path.join(source,f)):
        if f not in files_to_exclude:
            #print('-',f)
            shutil.copy(os.path.join(source,f), target)
            

# copy from
#  \tools\st-drivers\stm32f1xx_hal_driver\Inc
# to
#  \mLRS\rx-diy-board01-f103cb\Drivers\STM32F1xx_HAL_Driver\Inc
#  and all other folders with f103

f1xx_hal_files_to_include = [
  'stm32f1xx_hal.c',
  'stm32f1xx_hal_cortex.c',
  'stm32f1xx_hal_pwr.c',
  'stm32f1xx_hal_pwr_ex.c',
  'stm32f1xx_hal_rcc.c',
  'stm32f1xx_hal_rcc_ex.c',
  'stm32f1xx_hal_flash.c',
  'stm32f1xx_hal_flash_ex.c',
  'stm32f1xx_hal_i2c.c',
]

l4xx_hal_files_to_include = [
  'stm32l4xx_hal.c',
  'stm32l4xx_hal_cortex.c',
  'stm32l4xx_hal_pwr.c',
  'stm32l4xx_hal_pwr_ex.c',
  'stm32l4xx_hal_rcc.c',
  'stm32l4xx_hal_rcc_ex.c',
  'stm32l4xx_hal_flash.c',
  'stm32l4xx_hal_flash_ex.c',
  'stm32l4xx_hal_i2c.c',
]

g4xx_hal_files_to_include = [
  'stm32g4xx_hal.c',
  'stm32g4xx_hal_cortex.c',
  'stm32g4xx_hal_pwr.c',
  'stm32g4xx_hal_pwr_ex.c',
  'stm32g4xx_hal_rcc.c',
  'stm32g4xx_hal_rcc_ex.c',
  'stm32g4xx_hal_flash.c',
  'stm32g4xx_hal_flash_ex.c',
  'stm32g4xx_hal_dma.c',
  'stm32g4xx_hal_i2c.c',
  'stm32g4xx_hal_i2c_ex.c',
]

wlxx_hal_files_to_include = [
  'stm32wlxx_hal.c',
  'stm32wlxx_hal_cortex.c',
  'stm32wlxx_hal_pwr.c',
  'stm32wlxx_hal_pwr_ex.c',
  'stm32wlxx_hal_rcc.c',
  'stm32wlxx_hal_rcc_ex.c',
  'stm32wlxx_hal_flash.c',
  'stm32wlxx_hal_flash_ex.c',
  'stm32wlxx_hal_i2c.c',
]


def copy_cmsis_driver(folder, chip):
    print('--- COPY CMSIS ---')
    print('folder:', folder)
    print('chip:  ', chip)
    chip_short = chip[:2]
    chip_short_upper = chip_short.upper()
    if chip_short == 'f1' or chip_short == 'l4' or chip_short == 'g4' or chip_short == 'wl':
        source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','cmsis_device_'+chip_short,'Include')
        print('src:   ', source)
        target = os.path.join(mLRSdirectory,folder,'Drivers','CMSIS','Device','ST','STM32'+chip_short_upper+'xx','Include')
        print('target:', target)
        delete_dir(target)
        copy_dir(target, source)


def create_exclude_list(dirlist, chip_short):
    files_to_exclude = []
    if chip_short == 'f1':
        files_to_include = f1xx_hal_files_to_include
    if chip_short == 'l4':
        files_to_include = l4xx_hal_files_to_include
    if chip_short == 'g4':
        files_to_include = g4xx_hal_files_to_include
    if chip_short == 'wl':
        files_to_include = wlxx_hal_files_to_include
    # exclude all hal files which are not included
    for f in dirlist:
        if '_ll_' in f or 'Legacy' in f:
            continue
        if not f in files_to_include:
            files_to_exclude.append(f)
    return files_to_exclude        


def copy_hal_driver(folder, chip):
    print('--- COPY HAL ---')
    print('folder:', folder)
    print('chip:  ', chip)
    chip_short = chip[:2]
    chip_short_upper = chip_short.upper()
    if chip_short == 'f1' or chip_short == 'l4' or chip_short == 'g4' or chip_short == 'wl':
        # copy Inc
        source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','stm32'+chip_short+'xx_hal_driver','Inc')
        print('src:   ', source)
        target = os.path.join(mLRSdirectory,folder,'Drivers','STM32'+chip_short_upper+'xx_HAL_Driver','Inc')
        print('target:', target)
        delete_dir(target)
        copy_dir(target, source)
        # copy Src
        source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','stm32'+chip_short+'xx_hal_driver','Src')
        print('src:   ', source)
        target = os.path.join(mLRSdirectory,folder,'Drivers','STM32'+chip_short_upper+'xx_HAL_Driver','Src')
        print('target:', target)
        delete_dir(target)
        dirlist = os.listdir(source)
        files_to_exclude = create_exclude_list(dirlist, chip_short)
        for f in dirlist:
            copy_wexclude(target, source, f, files_to_exclude)


def do_for_each_folder():
    print('----------------------------------------')
    print(' copy CMSIS and HAL files to project target folders')
    print('----------------------------------------')
    dirlist = os.listdir(mLRSdirectory)
    for f in dirlist:
        if f[:3] == "rx-" or f[:3] == "tx-":
            print('#############################')
            print('*', f)
            chip = re.findall(r"\w+-(\w+)$", f)[0]
            copy_cmsis_driver(f, chip)
            copy_hal_driver(f, chip)
            #break

    print('# DONE #')


do_for_each_folder()
os.system("pause")
