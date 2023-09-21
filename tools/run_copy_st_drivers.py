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
 version 21.09.2023
********************************************************
'''
import os
import shutil
import re
import sys


mLRSProjectdirectory = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
mLRSdirectory = os.path.join(mLRSProjectdirectory,'mLRS')


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


def create_clean_dir(directory):
    if os.path.exists(directory):
        remake_dir('"'+directory+'"')
    make_dir('"'+directory+'"')

            
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


#--------------------------------------------------
# main stuff
#--------------------------------------------------
# copy from
#  \tools\st-drivers\STM32F1xx_HAL_Driver\Inc
# to
#  \mLRS\rx-diy-board01-f103cb\Drivers\STM32F1xx_HAL_Driver\Inc
#  and all other folders with f103

# we copy all _ll_ and _hal_ .h files
# we copy all _ll_ .c files
# we copy only selected _hal_ .c files
# we can extract them from run_make_firmware3.py, MLRS_SOURCES_HAL_STM32xx

from run_make_firmwares3 import *

f1xx_hal_files_to_include = []
for f in MLRS_SOURCES_HAL_STM32F1:
    if '_hal' in f:
        #print(os.path.basename(f))    
        f1xx_hal_files_to_include.append(os.path.basename(f))

l4xx_hal_files_to_include = []
for f in MLRS_SOURCES_HAL_STM32L4:
    if '_hal' in f:
        l4xx_hal_files_to_include.append(os.path.basename(f))

g4xx_hal_files_to_include = []
for f in MLRS_SOURCES_HAL_STM32G4:
    if '_hal' in f:
        g4xx_hal_files_to_include.append(os.path.basename(f))

wlxx_hal_files_to_include = []
for f in MLRS_SOURCES_HAL_STM32WL:
    if '_hal' in f:
        wlxx_hal_files_to_include.append(os.path.basename(f))

f0xx_hal_files_to_include = []
for f in MLRS_SOURCES_HAL_STM32F0:
    if '_hal' in f:
        f0xx_hal_files_to_include.append(os.path.basename(f))


# some targets also need the USB driver
# we can go through TLIST and watch for 'STDSTM32_USE_USB' to determine which do

targets_with_usb_to_include = []
for t in TLIST:
    if 'STDSTM32_USE_USB' in t['extra_D_list']:
        if t['target'] not in targets_with_usb_to_include:
            targets_with_usb_to_include.append(t['target'])


def copy_cmsis_driver(folder, chip, clean=False, silent=False):
    print('--- COPY CMSIS ---')
    if not silent: print('folder:', folder)
    if not silent: print('chip:  ', chip)
    chip_short = chip[:2]
    chip_short_upper = chip_short.upper()

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
    if os.name == 'nt': # do only on win, linux's case sensitivity makes it more complicated
        shutil.copy(os.path.join(source,'..','LICENSE.md'), os.path.join(target,'..'))


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
    if os.name == 'nt': # do only on win, linux's case sensitivity makes it more complicated
        shutil.copy(os.path.join(source,'..','LICENSE.md'), os.path.join(target,'..'))


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
            
            if not clean:
                copy_cmsis_core(f, clean, silent)
                copy_cmsis_driver(f, chip, clean, silent)
                copy_hal_driver(f, chip, clean, silent)
            
            if f in targets_with_usb_to_include:
                copy_usb_device_library_driver(f,clean,silent)
            
            #break
            #exit(1)


#-- here we go

cmdline_silent = False
cmdline_clean = False

for cmd in sys.argv:
    if cmd == '-silent':
        cmdline_silent = True
    if cmd == '-clean':
        cmdline_clean = True

#do_for_each_folder(clean=True)
do_for_each_target(clean=cmdline_clean,silent=cmdline_silent)


print('# DONE #')
if not cmdline_silent:
    os.system("pause")
