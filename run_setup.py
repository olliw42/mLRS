#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 run_setup.py
 1. run git submodule update --init --recursive
 2. calls run_copy_st_drivers.py to populate the target ST Driver folders
 3. calls fmav_generate_c_library.py to generate MAVLink library files
 4. calls dronecan_generate_c_library.py to generate DroneCAN library files
 version 25.01.2026
********************************************************
'''
import os
import subprocess
import re
import sys


mLRSProjectdirectory = os.path.dirname(os.path.abspath(__file__))
mLRSdirectory = os.path.join(mLRSProjectdirectory,'mLRS')

python_cmd = '' # 'python' or 'python3' depending on installation
silent = False


def os_system(arg):
    if silent:
        res = subprocess.call(arg, stdout=subprocess.DEVNULL)
    else:
        res = subprocess.call(arg)
    if res != 0:
        print('# ERROR (errno =',res,') DONE #')
        print('Press Enter to continue')
        input()
        exit(1)


def _check_python_version(required_version):
    try:
        res = subprocess.check_output([required_version, "--version"], text=True)
        major_version = re.findall(r'\d', res)[0]
        return int(major_version)
    except:
        return 0


def check_python():
    # check if Python is installed and find which Python cmd to use
    global python_cmd
    if _check_python_version('python') == 3:
        python_cmd = 'python'
    elif _check_python_version('python3') == 3:
        python_cmd = 'python3'
    else:
        print("ERROR: Python 3 not found on your system. Please make sure Python 3 is available.")
        print('Press Enter to continue')
        input()
        exit(1)


def git_submodules_update():
    print('----------------------------------------')
    print(' run git submodule update --init --recursive')
    print('----------------------------------------')
    os_system(['git', 'submodule', 'update', '--init', '--recursive'])
    print('# DONE #')


def copy_st_drivers():
    print('----------------------------------------')
    print(' run run_copy_st_drivers.py')
    print('----------------------------------------')
    os.chdir(os.path.join(mLRSProjectdirectory,'tools'))
    os_system([python_cmd, os.path.join('.' , 'run_copy_st_drivers.py'), '-silent'])
    print('# DONE #')


def generate_mavlink_c_library():
    print('----------------------------------------')
    print(' run fmav_generate_c_library.py')
    print('----------------------------------------')
    os.chdir(os.path.join(mLRSdirectory,'Common','mavlink'))
    os_system([python_cmd, os.path.join('.','fmav_generate_c_library.py')])
    print('# DONE #')


def generate_dronecan_c_library():
    print('----------------------------------------')
    print(' run dronecan_generate_c_library.py')
    print('----------------------------------------')
    os.chdir(os.path.join(mLRSdirectory,'Common','dronecan'))
    os_system([python_cmd, os.path.join('.','dronecan_generate_c_library.py'), '-np'])
    print('# DONE #')


cmdline_submodules_update = False
cmdline_copy_st_drivers = False
cmdline_mavlink = False
cmdline_dronecan = False
hascmd = False

cmd_pos = -1
for cmd in sys.argv:
    cmd_pos += 1
    if cmd == '--silent' or cmd == '--s':
        silent = True
    if cmd == '--submodules' or cmd == '-g' or cmd == '-G':
        cmdline_submodules_update = True
        hascmd = True
    if cmd == '--copy' or cmd == '-c' or cmd == '-C':
        cmdline_copy_st_drivers = True
        hascmd = True
    if cmd == '--mavlink' or cmd == '-m' or cmd == '-M':
        cmdline_mavlink = True
        hascmd = True
    if cmd == '--dronecan' or cmd == '-d' or cmd == '-D':
        cmdline_dronecan = True
        hascmd = True

check_python()
if cmdline_submodules_update or not hascmd:
    git_submodules_update()
if cmdline_copy_st_drivers or not hascmd:
    copy_st_drivers()
if cmdline_mavlink or not hascmd:
    generate_mavlink_c_library()
if cmdline_dronecan or not hascmd:
    generate_dronecan_c_library()

print('Press Enter to continue')
input()
