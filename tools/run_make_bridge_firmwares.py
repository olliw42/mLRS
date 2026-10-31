#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 run_make_bridge_firmwares.py
 generate wireless bridge firmware files with arduino-cli
 copies files into firmware/wirelessbridge, updates its README.md
 version 26.10.2026

 Requires arduino-cli, which is a separate install, it does not come with PlatformIO or Python:
   Windows, Linux: https://arduino.github.io/arduino-cli/latest/installation/
   Mac: brew install arduino-cli
 Arduino IDE 2 bundles a copy, but not on the PATH, set the ARDUINO_CLI env var to use that one.
 On the first run add -u, this installs the esp32 and esp8266 cores.
********************************************************
'''
import os
import shutil
import re
import sys
import json
import datetime
import subprocess


#-- mLRS directories

MLRS_PROJECT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

MLRS_SKETCH_DIR = os.path.join(MLRS_PROJECT_DIR,'esp','mlrs-wireless-bridge')
MLRS_SKETCH_FILE = os.path.join(MLRS_SKETCH_DIR,'mlrs-wireless-bridge.ino')

MLRS_BRIDGE_BUILD_DIR = os.path.join(MLRS_PROJECT_DIR,'tools','esp-build','wirelessbridge')
MLRS_BRIDGE_FIRMWARE_DIR = os.path.join(MLRS_PROJECT_DIR,'firmware','wirelessbridge')


#-- board manager urls, so -u also works on a fresh arduino-cli install

BOARD_MANAGER_URLS = [
    'https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json',
    'https://arduino.esp8266.com/stable/package_esp8266com_index.json',
]


#-- targets
# name: firmware file is mlrs-wireless-bridge-<name>.ino.bin, the Web Flasher matches on this

TARGETS = [
    { 'name': 'esp32c3',
      'module': 'MODULE_ESP32C3_ELRS_TX',
      'fqbn': 'esp32:esp32:esp32c3:PartitionScheme=no_ota',
      'core': 'esp32:esp32',
      'note': 'ESP32-C3 Dev Module, Partition Scheme "No OTA (2MB APP/2MB SPIFFS)"' },
    { 'name': 'esp8266',
      'module': 'MODULE_ESP82XX_ELRS_TX',
      'fqbn': 'esp8266:esp8266:generic:eesz=1M,FlashMode=dout',
      'core': 'esp8266:esp8266',
      'note': 'Generic ESP8266 Module, Flash Size 1MB, Flash Mode DOUT' },
]


#-- helper

def printWarning(txt):
    print('\033[93m'+txt+'\033[0m') # light Yellow


def printError(txt):
    print('\033[91m'+txt+'\033[0m') # light Red


def find_arduino_cli():
    # ARDUINO_CLI env var overrides the PATH, e.g. for the arduino-cli bundled with Arduino IDE 2
    cli = os.environ.get('ARDUINO_CLI') or shutil.which('arduino-cli')
    if not cli or not os.path.exists(cli):
        printError('ERROR: arduino-cli not found, install it, add it to the PATH or set ARDUINO_CLI')
        sys.exit(1)
    return cli


def update_cores(cli, core_versions):
    print('Update index, install or upgrade cores')
    urls = ['--additional-urls', ','.join(BOARD_MANAGER_URLS)]
    subprocess.call([cli, 'core', 'update-index'] + urls)
    for core in sorted(set(t['core'] for t in TARGETS)):
        cmd = 'upgrade' if core in core_versions else 'install'
        subprocess.call([cli, 'core', cmd, core] + urls)


def get_core_versions(cli):
    res = subprocess.run([cli, 'core', 'list', '--json'], capture_output=True, text=True, encoding='utf-8')
    data = json.loads(res.stdout or '{}')
    versions = {}
    for p in data.get('platforms', []):
        versions[p['id']] = p.get('installed_version', '')
    return versions


def get_sketch_version():
    # sketch date from the header, e.g. "// 11. Oct. 2026" -> "11.10.2026"
    F = open(MLRS_SKETCH_FILE, mode='r', encoding='utf-8')
    content = F.read()
    F.close()
    v = re.search(r'^//\s*(\d+)\.\s*(\w{3})\w*\.?\s*(\d{4})\s*$', content, re.MULTILINE)
    if not v:
        printWarning('WARNING: sketch date not found')
        return '?'
    months = ['jan','feb','mar','apr','may','jun','jul','aug','sep','oct','nov','dec'] # strptime %b is locale dependent
    if v.group(2).lower() not in months:
        printWarning('WARNING: sketch date month not recognized')
        return '?'
    month = months.index(v.group(2).lower()) + 1
    return '%02d.%02d.%s' % (int(v.group(1)), month, v.group(3))


#--------------------------------------------------
# build system
#--------------------------------------------------

def mlrs_bridge_compile(cli, target):
    # no --output-dir: the esp32 core's export hooks would then also copy into the sketch folder
    build_path = os.path.join(MLRS_BRIDGE_BUILD_DIR, target['name'])
    args = [cli, 'compile',
            '--fqbn', target['fqbn'],
            '--build-path', build_path,
            '--build-property', 'compiler.cpp.extra_flags=-D'+target['module'],
            '--clean',
            MLRS_SKETCH_DIR]
    print('Build', target['name'], '('+target['module']+')')
    if subprocess.call(args) != 0:
        printError('ERROR: build failed for '+target['name'])
        return None
    return os.path.join(build_path, 'mlrs-wireless-bridge.ino.bin')


#--------------------------------------------------
# application
#--------------------------------------------------

def bridge_bin_name(target):
    return 'mlrs-wireless-bridge-'+target['name']+'.ino.bin'


def mlrs_bridge_read_readme_dates():
    # keeps the dates of binaries not rebuilt in this run
    dates = {}
    readme = os.path.join(MLRS_BRIDGE_FIRMWARE_DIR, 'README.md')
    if not os.path.exists(readme):
        return dates
    F = open(readme, mode='r', encoding='utf-8')
    content = F.read()
    F.close()
    for m in re.finditer(r'^(\S+\.bin)\s*\n- date of creation: (\S+)\s*\n- sketch version: (\S+)', content, re.MULTILINE):
        dates[m.group(1)] = (m.group(2), m.group(3))
    return dates


def mlrs_bridge_write_readme(dates, core_versions):
    s = '# mLRS - Firmware - WirelessBridge #\n\n'
    s += 'This folder contains pre-compiled wireless-bridge firmware binaries.\n\n'
    for target in TARGETS:
        fname = bridge_bin_name(target)
        if fname not in dates:
            continue
        s += fname+'\n'
        s += '- date of creation: '+dates[fname][0]+'\n'
        s += '- sketch version: '+dates[fname][1]+'\n\n'
    s += '## Notes for Devs ##\n\n'
    s += 'The binaries are built with tools/run_make_bridge_firmwares.py, which needs arduino-cli with the cores below.\n'
    s += 'Use -t <name> to build only one target, -u to first install or update the cores.\n'
    s += 'Install arduino-cli: Windows, Linux see https://arduino.github.io/arduino-cli/latest/installation/, Mac brew install arduino-cli.\n\n'
    for target in TARGETS:
        s += '- '+target['name']+': '+target['note']+', '+target['module']+', core '+target['core']+' '+core_versions.get(target['core'], '?')+'\n'
    F = open(os.path.join(MLRS_BRIDGE_FIRMWARE_DIR, 'README.md'), mode='w', encoding='utf-8', newline='\n')
    F.write(s)
    F.close()


#-- here we go
if __name__ == "__main__":
    cmdline_target = ''
    cmdline_nopause = False
    cmdline_upgrade = False

    cmd_pos = -1
    for cmd in sys.argv:
        cmd_pos += 1
        if cmd == '--target' or cmd == '-t' or cmd == '-T':
            if cmd_pos+1 < len(sys.argv) and sys.argv[cmd_pos+1] != '':
                cmdline_target = sys.argv[cmd_pos+1].lower()
        if cmd == '--nopause' or cmd == '-np':
            cmdline_nopause = True
        if cmd == '--upgrade' or cmd == '-u':
            cmdline_upgrade = True

    cli = find_arduino_cli()
    core_versions = get_core_versions(cli)
    if cmdline_upgrade:
        update_cores(cli, core_versions)
        core_versions = get_core_versions(cli)
    sketch_version = get_sketch_version()
    print('sketch version =', sketch_version)

    targets = [t for t in TARGETS if cmdline_target == '' or t['name'] == cmdline_target]
    if not targets:
        printError('ERROR: unknown target '+cmdline_target)
        sys.exit(1)

    dates = mlrs_bridge_read_readme_dates()
    today = datetime.date.today().strftime('%d.%m.%Y')
    failed = []
    for target in targets:
        if target['core'] not in core_versions:
            printError('ERROR: core '+target['core']+' not installed, run with -u to install it')
            failed.append(target['name'])
            continue
        binfile = mlrs_bridge_compile(cli, target)
        if not binfile:
            failed.append(target['name'])
            continue
        shutil.copy(binfile, os.path.join(MLRS_BRIDGE_FIRMWARE_DIR, bridge_bin_name(target)))
        dates[bridge_bin_name(target)] = (today, sketch_version)
        print('copied', bridge_bin_name(target))

    mlrs_bridge_write_readme(dates, core_versions)
    if failed:
        printError('FAILED: '+', '.join(failed))

    if os.name != 'posix' and not cmdline_nopause:
        os.system("pause")
    sys.exit(1 if failed else 0)
