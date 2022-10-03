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


def copy_woverwrite(target, source, f, files_to_exclude=[]):
    if os.path.isfile(os.path.join(source,f)):
        if os.path.exists(os.path.join(target,f)):
            os.remove(os.path.join(target,f))
        if f not in files_to_exclude:
            #print('-',f)
            shutil.copy(os.path.join(source,f), target)

# copy from
#  \tools\st-drivers\stm32f1xx_hal_driver\Inc
# to
#  \mLRS\rx-diy-board01-f103cb\Drivers\STM32F1xx_HAL_Driver\Inc
#  and all other folders with f103

f1xx_hal_files_to_exclude = [
  'stm32f1xx_hal_msp_template.c',
  'stm32f1xx_hal_timebase_rtc_alarm_template.c',
  'stm32f1xx_hal_timebase_tim_template.c'
]

l4xx_hal_files_to_exclude = [
  'stm32l4xx_hal_msp_template.c',
  'stm32l4xx_hal_timebase_rtc_alarm_template.c',
  'stm32l4xx_hal_timebase_tim_template.c'
]

g4xx_hal_files_to_exclude = [
  'stm32g4xx_hal_msp_template.c',
  'stm32g4xx_hal_timebase_tim_template.c',
]


def copy_cmsis_driver(folder, chip):
    print('--- COPY CMSIS ---')
    print('folder:', folder)
    print('chip:  ', chip)
    chip_short = chip[:2]
    chip_short_upper = chip_short.upper()
    if chip_short == 'f1' or chip_short == 'l4' or chip_short == 'g4':
        source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','cmsis_device_'+chip_short,'Include')
        print('src:   ', source)
        target = os.path.join(mLRSdirectory,folder,'Drivers','CMSIS','Device','ST','STM32'+chip_short_upper+'xx','Include')
        print('target:', target)
        dirlist = os.listdir(source)
        for f in dirlist:
            copy_woverwrite(target, source, f)


def copy_hal_driver(folder, chip):
    print('--- COPY HAL ---')
    print('folder:', folder)
    print('chip:  ', chip)
    chip_short = chip[:2]
    chip_short_upper = chip_short.upper()
    if chip_short == 'f1' or chip_short == 'l4' or chip_short == 'g4' :
        # copy Inc
        source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','stm32'+chip_short+'xx_hal_driver','Inc')
        print('src:   ', source)
        target = os.path.join(mLRSdirectory,folder,'Drivers','STM32'+chip_short_upper+'xx_HAL_Driver','Inc')
        print('target:', target)
        dirlist = os.listdir(source)
        for f in dirlist:
            copy_woverwrite(target, source, f)
        # copy Src
        source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','stm32'+chip_short+'xx_hal_driver','Src')
        print('src:   ', source)
        target = os.path.join(mLRSdirectory,folder,'Drivers','STM32'+chip_short_upper+'xx_HAL_Driver','Src')
        print('target:', target)
        dirlist = os.listdir(source)
        files_to_exclude = []        
        if chip_short == 'f1':
            files_to_exclude = f1xx_hal_files_to_exclude
        if chip_short == 'l4':
            files_to_exclude = l4xx_hal_files_to_exclude
        if chip_short == 'g4':
            files_to_exclude = g4xx_hal_files_to_exclude
        for f in dirlist:
            copy_woverwrite(target, source, f, files_to_exclude)
       


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
