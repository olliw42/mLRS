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
import shutil
import re
import sys


mLRSProjectdirectory = os.path.dirname(os.path.abspath(__file__))
mLRSdirectory = os.path.join(mLRSProjectdirectory,'mLRS')


def git_submodules_update():
    print('----------------------------------------')
    print(' run git submodule update --init --recursive')
    print('----------------------------------------')
    os.system('git submodule update --init --recursive')
    print('# DONE #')


def copy_st_drivers():
    print('----------------------------------------')
    print(' run run_copy_st_drivers.py')
    print('----------------------------------------')
    os.chdir(os.path.join(mLRSProjectdirectory,'tools'))
    #os.system(os.path.join('.','fmav_generate_c_library.py'))
    os.system('run_copy_st_drivers.py -silent')
    print('# DONE #')


def generate_mavlink_c_library():
    print('----------------------------------------')
    print(' run fmav_generate_c_library.py')
    print('----------------------------------------')
    os.chdir(os.path.join(mLRSdirectory,'Common','mavlink'))
    os.system(os.path.join('.','fmav_generate_c_library.py'))
    print('# DONE #')


git_submodules_update()
copy_st_drivers()
generate_mavlink_c_library()

os.system("pause")

