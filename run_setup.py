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
 2. goes through STM32CubeIDE configuration files and adapts them as needed to work
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


def call_replace(filename):
    F = open(filename, mode='r')
    content = F.read()
    F.close()

    print(os.path.join(filename+'-orig'))

    F = open(os.path.join(filename+'-orig'), mode='w')
    F.write(content)
    F.close()

    d = mLRSProjectdirectory.replace('\\','/')
    content1 = content.replace('C:/Users/Olli/Documents/GitHub/mlrs',d) #TODO: use a regex

    F = open(filename, mode='w')
    F.write(content1)
    F.close()

def correct_folders_in_project_files():
    print('----------------------------------------')
    print(' correct .project and .mxproject files for folders')
    print('----------------------------------------')
    dirlist = os.listdir(mLRSdirectory)
    for f in dirlist:
        print('*', f)
        pf = os.path.join(mLRSdirectory,f,'.project')
        print('  ',pf)

        if os.path.isfile(pf):
            call_replace(pf)

        mxpf = os.path.join(mLRSdirectory,f,'.mxproject')
        print('  ',mxpf)

        if os.path.isfile(mxpf):
            print('    .mxproject exists')
            call_replace(mxpf)
    print('# DONE #')


def generate_mavlink_c_library():
    print('----------------------------------------')
    print(' run fmav_generate_c_library.py')
    print('----------------------------------------')
    os.chdir(os.path.join(mLRSdirectory,'Common','mavlink'))
    os.system('fmav_generate_c_library.py')
    print('# DONE #')




git_submodules_update()
correct_folders_in_project_files()
generate_mavlink_c_library()