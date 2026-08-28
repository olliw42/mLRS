#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 ******************************************************
 run_flash_stm32_linux.py
 Linux or MacOS flash utility for STM32 mLRS targets
 uses STM32CubeProgrammer CLI when available, else dfu-util (DFU) or st-flash (SWD)
 version 12.08.2026
********************************************************
'''
import os
import sys
import shutil
import glob
import argparse
import subprocess


#-- directories

MLRS_PROJECT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
MLRS_TOOLS_DIR = os.path.join(MLRS_PROJECT_DIR, 'tools')
MLRS_BUILD_DIR = os.path.join(MLRS_TOOLS_DIR, 'build')
MLRS_FIRMWARE_DIR = os.path.join(MLRS_BUILD_DIR, 'firmware')
RUN_MAKE_FIRMWARES = os.path.join(MLRS_TOOLS_DIR, 'run_make_firmwares.py')

STM32_DFU_VID_PID = '0483:df11' # STM32 system (ROM) bootloader, as seen by dfu-util
STM32_FLASH_BASE = 0x08000000   # all STM32 mLRS targets load here

# STM32CubeProgrammer CLI install locations, globs are tried in order
# set the STM32_PROGRAMMER_CLI env var to override
CUBEPROG_CLI_NAME = 'STM32_Programmer_CLI'
CUBEPROG_GLOBS = [
    # MacOS
    '/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/' + CUBEPROG_CLI_NAME,
    os.path.expanduser('~/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/' + CUBEPROG_CLI_NAME),
    # Linux
    '/opt/st/stm32cubeprog*/bin/' + CUBEPROG_CLI_NAME,
    '/opt/stm32cubeprog*/bin/' + CUBEPROG_CLI_NAME,
    '/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/' + CUBEPROG_CLI_NAME,
    os.path.expanduser('~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/' + CUBEPROG_CLI_NAME),
    os.path.expanduser('~/st/stm32cubeprog*/bin/' + CUBEPROG_CLI_NAME),
]


#-- build & firmware lookup

def build_target(target, defines):
    # build the matching target(s) via run_make_firmwares.py; return its exit code
    cmd = [sys.executable, RUN_MAKE_FIRMWARES, '-t', target, '-np']
    for d in defines:
        cmd += ['-d', d]
    print('building:', ' '.join(cmd))
    return subprocess.call(cmd)


def find_firmware_hex(target):
    # .hex files in tools/build/firmware/ whose name contains the target substring (case insensitive)
    if not os.path.isdir(MLRS_FIRMWARE_DIR):
        return []
    hexes = []
    for path in sorted(glob.glob(os.path.join(MLRS_FIRMWARE_DIR, '*.hex'))):
        if target.lower() in os.path.basename(path).lower():
            hexes.append(path)
    return hexes


def choose_firmware(hexes):
    # prompt the user to pick one firmware when several match
    print('Multiple firmwares match the target:')
    for i, path in enumerate(hexes):
        print('  [%d] %s' % (i, os.path.basename(path)))
    while True:
        sel = input('Select firmware to flash [0-%d] (or q to abort): ' % (len(hexes)-1)).strip()
        if sel.lower() == 'q':
            return None
        if sel.isdigit() and 0 <= int(sel) < len(hexes):
            return hexes[int(sel)]
        print('invalid selection')


def hex_to_bin(hex_path):
    # convert .hex to a .bin next to it via objcopy
    objcopy = shutil.which('arm-none-eabi-objcopy') or shutil.which('objcopy')
    if objcopy is None:
        print('ERROR: objcopy not found on PATH (need arm-none-eabi-objcopy)')
        return None
    bin_path = os.path.splitext(hex_path)[0] + '.bin'
    rc = subprocess.call([objcopy, '-I', 'ihex', '-O', 'binary', '--gap-fill', '0xFF', hex_path, bin_path])
    if rc != 0:
        print('ERROR: objcopy failed (return code %d)' % rc)
        return None
    print('converted %s -> %s' % (os.path.basename(hex_path), os.path.basename(bin_path)))
    return bin_path


#-- STM32CubeProgrammer

no_cubeprog = False # set by --no-cubeprog, forces the dfu-util / st-flash fallbacks


def find_cubeprog():
    # locate the STM32CubeProgrammer CLI, env var first, then PATH, then the usual install dirs
    if no_cubeprog:
        return None
    cli = os.environ.get('STM32_PROGRAMMER_CLI')
    if cli:
        if os.path.isfile(cli) and os.access(cli, os.X_OK):
            return cli
        print('WARNING: STM32_PROGRAMMER_CLI is set but not an executable file:', cli)
    cli = shutil.which(CUBEPROG_CLI_NAME)
    if cli is not None:
        return cli
    for pattern in CUBEPROG_GLOBS:
        for path in sorted(glob.glob(pattern), reverse=True): # prefer the newest versioned dir
            if os.path.isfile(path) and os.access(path, os.X_OK):
                return path
    return None


def flash_cubeprog(cli, hex_path, connect_args, reset=True):
    # flash a .hex with the CubeProgrammer CLI; it reads ihex directly, so no objcopy needed
    cmd = [cli, '-c'] + connect_args + ['-w', hex_path, '-v']
    if reset:
        cmd += ['-rst']
    print('flashing (cubeprogrammer):', ' '.join(cmd))
    return subprocess.call(cmd)


#-- flash methods

def flash_dfu(hex_path, target=None):
    # flash via USB DFU; board must already be in DFU mode
    cli = find_cubeprog()
    if cli is not None:
        # no -rst, reset is only available on the JTAG/SWD interface and would fail the run
        rc = flash_cubeprog(cli, hex_path, ['port=usb1'], reset=False)
        if rc == 0:
            print('flash complete - power-cycle / reset the board to run the new firmware')
        return rc
    if shutil.which('dfu-util') is None:
        print('ERROR: neither STM32CubeProgrammer CLI nor dfu-util found')
        print('       install STM32CubeProgrammer, or set STM32_PROGRAMMER_CLI, or e.g. brew install dfu-util')
        return 1
    bin_path = hex_to_bin(hex_path)
    if bin_path is None:
        return 1
    # append a DFU suffix so newer dfu-util versions accept the file
    if shutil.which('dfu-suffix'):
        vid, pid = STM32_DFU_VID_PID.split(':')
        subprocess.call(['dfu-suffix', '-a', bin_path, '-v', '0x'+vid, '-p', '0x'+pid], stdout=subprocess.DEVNULL)
    cmd = ['dfu-util', '-a', '0', '-d', STM32_DFU_VID_PID, '-s', '0x%08X' % STM32_FLASH_BASE, '-D', bin_path]
    print('flashing (dfu):', ' '.join(cmd))
    rc = subprocess.call(cmd)
    if rc != 0:
        return rc
    print('flash complete - power-cycle / reset the board to run the new firmware')
    return 0


def flash_swd(hex_path, target=None):
    # flash via SWD, CubeProgrammer preferred as st-flash lacks newer chips (H503, C5, ...)
    cli = find_cubeprog()
    if cli is not None:
        return flash_cubeprog(cli, hex_path, ['port=SWD', 'mode=UR'])
    if shutil.which('st-flash') is None:
        print('ERROR: neither STM32CubeProgrammer CLI nor st-flash found')
        print('       install STM32CubeProgrammer, or set STM32_PROGRAMMER_CLI, or e.g. brew install stlink')
        return 1
    cmd = ['st-flash', '--reset', '--format', 'ihex', 'write', hex_path]
    print('flashing (swd):', ' '.join(cmd))
    return subprocess.call(cmd)


FLASH_METHODS = {
    'dfu': flash_dfu,
    'swd': flash_swd,
}


#-- app

def main():
    parser = argparse.ArgumentParser(description='Build an STM32 mLRS target and flash it to the board.')
    parser.add_argument('-t', '--target', required=True,
        help='target name (substring, case insensitive) to build and flash, e.g. tx-matek-mr24-30-g431kb')
    parser.add_argument('-m', '--method', choices=sorted(FLASH_METHODS.keys()), default='dfu',
        help='flashing method (default: dfu)')
    parser.add_argument('-d', '--define', action='append', default=[], metavar='DEFINE',
        help='extra -D define forwarded to the build (repeatable)')
    parser.add_argument('--no-cubeprog', action='store_true',
        help='do not use STM32CubeProgrammer, force the dfu-util (DFU) / st-flash (SWD) fallbacks')
    args = parser.parse_args()

    global no_cubeprog
    no_cubeprog = args.no_cubeprog

    ret = build_target(args.target, args.define)
    if ret != 0:
        print('ERROR: build failed (run_make_firmwares.py returned %d)' % ret)
        return ret

    hexes = find_firmware_hex(args.target)
    if not hexes:
        print('ERROR: no firmware .hex matching "%s" in %s' % (args.target, MLRS_FIRMWARE_DIR))
        return 1
    if len(hexes) == 1:
        hex_path = hexes[0]
        print('firmware:', os.path.basename(hex_path))
    else:
        hex_path = choose_firmware(hexes)
        if hex_path is None:
            print('aborted')
            return 1

    ret = FLASH_METHODS[args.method](hex_path, args.target)
    if ret == 0:
        print('done')
    else:
        print('flashing failed (return code %d)' % ret)
    return ret


if __name__ == '__main__':
    sys.exit(main())
