#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 run_replace_names_by_variables.py
 goes through STM32CubeIDE configuration files and adapts them as needed to work
********************************************************
'''
import os
import shutil
import re
import sys


mLRSProjectdirectory = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
mLRSdirectory = os.path.join(mLRSProjectdirectory,'mLRS')


def call_replace(filename):
    F = open(filename, mode='r')
    content = F.read()
    F.close()

    #F = open(os.path.join(filename+'-orig'), mode='w')
    #F.write(content)
    #F.close()

    # U can use STM32CUBEIDE's PATH Variables(like: WORKSPACE_LOC, PROJECT_LOC ....) in the path
    content1 = content.replace('C:/Users/Olli/Documents/GitHub/mlrs/mLRS',"WORKSPACE_LOC") #TODO: use a regex

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
            call_replace(mxpf)

    print('# DONE #')


correct_folders_in_project_files()
