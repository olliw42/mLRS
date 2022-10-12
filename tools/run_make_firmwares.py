#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 run_make_firmwares.py
 goes through target folders and calls make
********************************************************
'''
import os
import shutil
import re
import sys


# installation dependent
# can we find this automatically?
STdirectory = os.path.join("C:/",'ST','STM32CubeIDE_1.7.0','STM32CubeIDE','plugins')
GNUdirectory = 'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.9-2020-q2-update.win32_2.0.0.202105311346'
Makedirectory = 'com.st.stm32cube.ide.mcu.externaltools.make.win32_2.0.0.202105311346'


mLRSProjectdirectory = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
mLRSdirectory = os.path.join(mLRSProjectdirectory,'mLRS')
firmwaredirectory = os.path.join(mLRSProjectdirectory,'firmware')


# we need to modify the PATH so that the correct toolchain/compiler is used
# why does sys.path.insert(0,xxx) not work?
path = os.environ["PATH"]
path = os.path.join(STdirectory,GNUdirectory,'tools','bin') + ';' + path
path = os.path.join(STdirectory,Makedirectory,'tools','bin') + ';' + path
os.environ["PATH"] = path


# global variables
VERSIONONLYSTR = 'v0.0.00'


def do_common_conf_h():
    global VERSIONONLYSTR
    F = open(os.path.join(mLRSdirectory,'Common','common_conf.h'), mode='r')
    content = F.read()
    F.close()
    
    v = re.search(r'VERSIONONLYSTR\s+"(\S+)"', content)
    if v:
        VERSIONONLYSTR = v.groups()[0]
        print('VERSIONONLYSTR =', VERSIONONLYSTR)
    else:
        print('----------------------------------------')
        print('ERROR: VERSIONONLYSTR not found')
        os.system('pause')
        exit()


def make_target(target):
    print('----------------------------------------')
    print(' make target', target)
    print('----------------------------------------')
    releasedirectory = os.path.join(mLRSdirectory,target,'Release')
    os.chdir(releasedirectory)
    #dirlist = os.listdir('.')
    #for f in dirlist:
    #    print('*', f)
    print('make clean')
    os.system('make clean 1>nul')
    print('make -j8 all')
    exit_code = os.system('make -j8 all 1>nul')
    if exit_code >= 2: #0: ok, 1: warnings, 2: errors
        print('----------------------------------------')
        print('ERROR with EXIT code', exit_code)
        os.system('pause')
        exit()
    os.system('arm-none-eabi-objcopy -O ihex '+target+'.elf '+target+'.hex')
    print('copy to firmware folder')
    # shutil.copy() does overwrite file, but folder must exist
    try:
        os.makedirs(firmwaredirectory)
    except:
        pass #is raised if dir already exists, but ok
    try:
        #shutil.copy(os.path.join(releasedirectory,target+'.bin'), os.path.join(firmwaredirectory,target+'-'+VERSIONONLYSTR+'.bin'))
        shutil.copy(os.path.join(releasedirectory,target+'.hex'), os.path.join(firmwaredirectory,target+'-'+VERSIONONLYSTR+'.hex'))
    except:
        print('----------------------------------------')
        print("ERROR: error while copying file")
        os.system('pause')
        exit()
    #exit()


def make_all_targets():
    print('----------------------------------------')
    print(' make for all targets')
    print('----------------------------------------')
    dirlist = os.listdir(mLRSdirectory)
    for f in dirlist:
        #print('*', f)
        if f[0:3] == 'rx-' or f[0:3] == 'tx-' :
            make_target(f)
    print('# DONE #')


do_common_conf_h()
make_all_targets()
os.system("pause")
