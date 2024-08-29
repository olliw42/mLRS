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
 3. calls fmav_generate_c_library.py to generated mavlink library files
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
        os.system("pause")
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
    elif check_python('python3') == 3:
        python_cmd = 'python3'
    else:
        print("ERROR: Python 3 not found on your system. Please make sure Python 3 is available.")
        os.system("pause")
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


if '-s' in sys.argv or '--silent' in sys.argv:
    silent = True

check_python()
git_submodules_update()
copy_st_drivers()
generate_mavlink_c_library()

os.system("pause")
