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

verbose = False
if '--verbose' in sys.argv:
    verbose = True

mLRSProjectdirectory = os.path.dirname(os.path.abspath(__file__))
mLRSdirectory = os.path.join(mLRSProjectdirectory,'mLRS')


def os_system(arg):
    stdout = None if verbose else subprocess.DEVNULL
    res = subprocess.call(arg, stdout=stdout)
    if res != 0:
        print('# ERROR (errno =',res,') DONE #')
        os.system("pause")
        exit(1)


def check_python(cmd):
    try: 
        ans = subprocess.check_output([cmd, "--version"], text=True )
        major_version = re.findall(r'\d', ans)[0]
        return int(major_version)
    except Exception as e:
        return 0


def git_submodules_update():
    print('----------------------------------------')
    print(' run git submodule update --init --recursive')
    print('----------------------------------------')
    os_system(['git', 'submodule', 'update', '--init', '--recursive'])
    print('# DONE #')


def copy_st_drivers(python_cmd):
    print('----------------------------------------')
    print(' run run_copy_st_drivers.py')
    print('----------------------------------------')
    os.chdir(os.path.join(mLRSProjectdirectory,'tools'))
    os_system([python_cmd, os.path.join('.' , 'run_copy_st_drivers.py'), '-silent'])    
    print('# DONE #')


def generate_mavlink_c_library(python_cmd):
    print('----------------------------------------')
    print(' run fmav_generate_c_library.py')
    print('----------------------------------------')
    os.chdir(os.path.join(mLRSdirectory,'Common','mavlink'))
    os_system([python_cmd, os.path.join('.','fmav_generate_c_library.py')])
    print('# DONE #')


# Find which Python cmd to use
if check_python("python") == 3:
    python_cmd = "python"
elif check_python("python3") == 3:
    python_cmd = "python3"
else:
    raise Exception("Please make sure Python 3 is available on your system")


git_submodules_update()
copy_st_drivers(python_cmd)
generate_mavlink_c_library(python_cmd)

os.system("pause")
