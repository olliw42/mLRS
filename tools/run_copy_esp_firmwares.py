#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 run_copy_esp_firmwares.py
 requires you to generate esp fimrware files into .pio/build manually (using VSC)
 renames and copies files into tools/esp-build/firmware
 version 8.04.2024
********************************************************
'''
import os
import pathlib
import shutil
import re
import sys



#-- mLRS directories

MLRS_PROJECT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

MLRS_DIR = os.path.join(MLRS_PROJECT_DIR,'mLRS')

MLRS_TOOLS_DIR = os.path.join(MLRS_PROJECT_DIR,'tools')
MLRS_BUILD_DIR = os.path.join(MLRS_PROJECT_DIR,'tools','build3')

MLRS_PIO_BUILD_DIR = os.path.join(MLRS_PROJECT_DIR,'.pio','build')
MLRS_ESP_BUILD_DIR = os.path.join(MLRS_PROJECT_DIR,'tools','esp-build')



#-- current version and branch

VERSIONONLYSTR = ''
BRANCHSTR = ''
HASHSTR = ''

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


def mlrs_set_branch_hash(version_str):
    global BRANCHSTR
    global HASHSTR
    import subprocess

    git_branch = subprocess.getoutput("git branch --show-current")
    if not git_branch == 'main':
        BRANCHSTR = '-'+git_branch
    if BRANCHSTR != '':
        print('BRANCHSTR =', BRANCHSTR)

    git_hash = subprocess.getoutput("git rev-parse --short HEAD")
    v_patch = int(version_str.split('.')[2])
    if v_patch % 2 == 1: # odd firmware patch version, so is dev, so add git hash
        HASHSTR = '-@'+git_hash
    if HASHSTR != '':
        print('HASHSTR =', HASHSTR)


#-- helper

def remake_dir(path): # os dependent
    if os.name == 'posix':
        os.system('rm -r -f '+path)
    else:
        os.system('rmdir /s /q "'+path+'"')

def make_dir(path): # os dependent
    if os.name == 'posix':
        os.system('mkdir -p '+path)
    else:
        os.system('md "'+path+'"')


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
# application
#--------------------------------------------------

def mlrs_copy_all_esp_elf_etc():
    print('copying .elf files')
    firmwarepath = os.path.join(MLRS_ESP_BUILD_DIR,'firmware')
    create_clean_dir(firmwarepath)
    for subdir in os.listdir(MLRS_PIO_BUILD_DIR): #os.walk(MLRS_PIO_BUILD_DIR):
        if os.path.isdir(os.path.join(MLRS_PIO_BUILD_DIR,subdir)): # needs to use full path for check
            print(subdir)
            file = os.path.join(MLRS_PIO_BUILD_DIR,subdir,'firmware.elf')
            shutil.copy(file, os.path.join(firmwarepath,subdir+'-'+VERSIONONLYSTR+BRANCHSTR+HASHSTR+'.elf'))


#-- here we go
if __name__ == "__main__":
    cmdline_target = ''
    cmdline_D_list = []
    cmdline_nopause = False
    cmdline_version = ''

    cmd_pos = -1
    for cmd in sys.argv:
        cmd_pos += 1
        if cmd == '--target' or cmd == '-t' or cmd == '-T':
            if sys.argv[cmd_pos+1] != '':
                cmdline_target = sys.argv[cmd_pos+1]
        if cmd == '--define' or cmd == '-d' or cmd == '-D':
            if sys.argv[cmd_pos+1] != '':
                cmdline_D_list.append(sys.argv[cmd_pos+1])
        if cmd == '--nopause' or cmd == '-np':
                cmdline_nopause = True
        if cmd == '--version' or cmd == '-v' or cmd == '-V':
            if sys.argv[cmd_pos+1] != '':
                cmdline_version = sys.argv[cmd_pos+1]

    #cmdline_target = 'tx-diy-e22dual-module02-g491re'
    #cmdline_target = 'tx-diy-sxdualXXX'

    if cmdline_version == '':
        mlrs_set_version()
        mlrs_set_branch_hash(VERSIONONLYSTR)
    else:
        VERSIONONLYSTR = cmdline_version

    mlrs_copy_all_esp_elf_etc()

    if not cmdline_nopause:
        os.system("pause")
