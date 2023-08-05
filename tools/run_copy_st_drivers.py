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


def create_clean_dir(directory):
    if os.path.exists(directory):
        if os.name == 'posix':
            os.system('rm -r -f "'+directory+'"')
        else:
            os.system('rmdir /s /q "'+directory+'"')

    if os.name == 'posix':
        os.system('mkdir -p "'+directory+'"')
    else:
        os.system('md "'+directory+'"')

            
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
#  \tools\st-drivers\STM32F1xx_HAL_Driver\Inc
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
  'stm32f1xx_hal_dma.c',
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
#  'stm32wlxx_hal_dma.c', # why not ??
  'stm32wlxx_hal_i2c.c',
  'stm32wlxx_hal_i2c_ex.c',
]

f0xx_hal_files_to_include = [
  'stm32f0xx_hal.c',
  'stm32f0xx_hal_cortex.c',
  'stm32f0xx_hal_pwr.c',
  'stm32f0xx_hal_pwr_ex.c',
  'stm32f0xx_hal_rcc.c',
  'stm32f0xx_hal_rcc_ex.c',
  'stm32f0xx_hal_flash.c',
  'stm32f0xx_hal_flash_ex.c',
  'stm32f0xx_hal_dma.c', # why not ??
  'stm32f0xx_hal_i2c.c',
  'stm32f0xx_hal_i2c_ex.c',
  'stm32f0xx_hal_pcd.c',
  'stm32f0xx_hal_pcd_ex.c',
]


def copy_cmsis_driver(folder, chip, clean=False, silent=False):
    print('--- COPY CMSIS ---')
    if not silent: print('folder:', folder)
    if not silent: print('chip:  ', chip)
    chip_short = chip[:2]
    chip_short_upper = chip_short.upper()
    if chip_short == 'f1' or chip_short == 'l4' or chip_short == 'g4' or chip_short == 'wl' or chip_short == 'f0':

        # clean target
        target = os.path.join(mLRSdirectory,folder,'Drivers','CMSIS','Device','ST','STM32'+chip_short_upper+'xx','Include')
        create_clean_dir(target)
        if clean: return

        # copy Include
        #source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','STM32'+chip_short_upper+'xx','Include')
        source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','cmsis_device_'+chip_short,'Include')
        if not silent: print('src:   ', source)
        #target = os.path.join(mLRSdirectory,folder,'Drivers','CMSIS','Device','ST','STM32'+chip_short_upper+'xx','Include')
        if not silent: print('target:', target)
        copy_dir(target, source)

        # copy licence
        #shutil.copy(os.path.join(source,'..','LICENSE.md'), os.path.join(target,'..'))


def copy_cmsis_core(folder, clean=False, silent=False):
    print('--- COPY CMSIS CORE ---')
    if not silent: print('folder:', folder)

    # clean target
    target = os.path.join(mLRSdirectory,folder,'Drivers','CMSIS','Include')
    create_clean_dir(target)
    if clean: return

    # copy Include
    source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','cmsis_core','Include')
    if not silent: print('src:   ', source)
    #target = os.path.join(mLRSdirectory,folder,'Drivers','CMSIS','Include')
    if not silent: print('target:', target)
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
    if chip_short == 'f0':
        files_to_include = f0xx_hal_files_to_include

    # exclude all hal files which are not included
    for f in dirlist:
        if '_ll_' in f or 'Legacy' in f:
            continue
        if not f in files_to_include:
            files_to_exclude.append(f)
    return files_to_exclude


def copy_hal_driver(folder, chip, clean=False, silent=False):
    print('--- COPY HAL ---')
    if not silent: print('folder:', folder)
    if not silent: print('chip:  ', chip)
    chip_short = chip[:2]
    chip_short_upper = chip_short.upper()
    if chip_short == 'f1' or chip_short == 'l4' or chip_short == 'g4' or chip_short == 'wl' or chip_short == 'f0':

        # clean target
        target_path = os.path.join(mLRSdirectory,folder,'Drivers','STM32'+chip_short_upper+'xx_HAL_Driver')
        create_clean_dir(target_path)
        if clean: return

        # copy Inc
        #source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','STM32'+chip_short_upper+'xx_HAL_Driver','Inc')
        source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','stm32'+chip_short+'xx_hal_driver','Inc')
        if not silent: print('src:   ', source)
        target = os.path.join(target_path,'Inc')
        if not silent: print('target:', target)
        create_clean_dir(target)
        copy_dir(target, source)

        # copy Src
        #source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','STM32'+chip_short_upper+'xx_HAL_Driver','Src')
        source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','stm32'+chip_short+'xx_hal_driver','Src')
        if not silent: print('src:   ', source)
        target = os.path.join(target_path,'Src')
        if not silent: print('target:', target)
        create_clean_dir(target)
        dirlist = os.listdir(source)
        files_to_exclude = create_exclude_list(dirlist, chip_short)
        for f in dirlist:
            copy_wexclude(target, source, f, files_to_exclude)

        # copy licence
        #shutil.copy(os.path.join(source,'..','LICENSE.md'), os.path.join(target,'..'))


def do_for_each_target(clean=False, silent=False):
    print('----------------------------------------')
    print(' copy CMSIS and HAL files to project target folders')
    print('----------------------------------------')
    dirlist = os.listdir(mLRSdirectory)
    for f in dirlist:
        if f[:3] == "rx-" or f[:3] == "tx-":
            print('#############################')
            print('*', f)
            chip = re.findall(r"\w+-(\w+)$", f)[0]

            target = os.path.join(mLRSdirectory,f,'Drivers')
            create_clean_dir(target)
            # copy licences, to ensure the folders are not empty
            source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','cmsis_core')
            target = os.path.join(mLRSdirectory,f,'Drivers','CMSIS')
            create_clean_dir(target)
            shutil.copy(os.path.join(source,'LICENSE.txt'), target)
            if clean: continue

            copy_cmsis_core(f, clean, silent)
            copy_cmsis_driver(f, chip, clean, silent)
            copy_hal_driver(f, chip, clean, silent)
            #break
            #exit(1)


def copy_usb_device_library_driver(folder, clean=False, silent=False):
    print('--- COPY USB Device Library ---')
    if not silent: print('folder:', folder)

    target = os.path.join(mLRSdirectory,folder,'Drivers','STM32_USB_Device_Library')
    create_clean_dir(target)
    if clean: return

    source = os.path.join(mLRSProjectdirectory,'tools','st-drivers','STM32_USB_Device_Library')
    if not silent: print('src:   ', source)
    if not silent: print('target:', target)
    
    loc = os.path.join('Class','CDC','Inc')
    create_clean_dir(os.path.join(target,loc))
    copy_files_in_dir(os.path.join(target,loc), os.path.join(source,loc))
    loc = os.path.join('Class','CDC','Src')
    create_clean_dir(os.path.join(target,loc))
    copy_files_in_dir(os.path.join(target,loc), os.path.join(source,loc))

    loc = os.path.join('Core','Inc')
    create_clean_dir(os.path.join(target,loc))
    copy_files_in_dir(os.path.join(target,loc), os.path.join(source,loc))
    loc = os.path.join('Core','Src')
    create_clean_dir(os.path.join(target,loc))
    copy_files_in_dir(os.path.join(target,loc), os.path.join(source,loc))



silent = False
if '-silent' in sys.argv: silent = True
clean = False
if '-clean' in sys.argv: clean = True

#do_for_each_folder(clean=True)
do_for_each_target(clean=clean,silent=silent)

copy_usb_device_library_driver('tx-FRM303-f070cb', clean=clean,silent=silent)

print('# DONE #')
if not silent:
    os.system("pause")
