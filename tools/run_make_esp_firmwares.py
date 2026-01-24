#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 run_make_esp_firmwares.py
 generate esp fimrware files
 renames and copies files into tools/esp-build/firmware
 version 24.01.2026
********************************************************

COMMAND LINE ARGUMENTS:
  --target, -t, -T <target_name>
      build only the specified target environment.
      if omitted, all environments defined in platformio.ini are built.
      note: currently parsed but not fully implemented in build logic.
      example: --target esp32-wroom

  --define, -d, -D <DEFINE>
      add a preprocessor define to the build (can be specified multiple times).
      each occurrence adds another define to the compilation.
      note: currently parsed but not fully implemented in build logic.
      example: --define DEBUG_MODE

  --nopause, -np, -NP
      skip the "Press Enter to continue..." prompt at the end of execution.
      useful for automated builds or CI/CD pipelines.

  --version, -v, -V <version_string>
      override the version string from common_conf.h.
      if omitted, the version is read from VERSIONONLYSTR in common_conf.h.
      the version is used in the output .bin filename.
      example: --version 1.2.3

  --no-clean, -nc, -NC
      skip the full clean step before building.
      by default, a full clean is performed before each build.
      use this flag for incremental builds (faster but may have stale artifacts).

  --file-jobs, -fj, -FJ <number>
      number of parallel file compilation jobs per target.
      controls file-level parallelism within each PlatformIO environment.
      default: (cpu_count * 2) / target_jobs - coordinated with target parallelism.
      example: --file-jobs 4

  --target-jobs, -tj, -TJ <number>
      number of parallel target builds.
      controls how many PlatformIO environments are built concurrently.
      default: min(num_targets, cpu_count / 2) - scales with available cores.
      example: --target-jobs 2

  --list-targets, -lt, -LT
      list all available ESP32 targets and exit without building.
      prints environment names from platformio.ini.
      use this to discover correct target names for --target flag.

  --flash, -f, -F
      flash the built firmware via USB serial after successful build.
      automatically uses platformio's upload command.
      requires exactly one target to be built (use --target flag).
      flashing baud rate: 921600 (configured in platformio.ini).
      example: --target rx-generic-2400-d-pa --flash

usage examples:
  # build all ESP targets with default settings (clean + parallel)
  python run_make_esp_firmwares.py

  # build with incremental compilation (no clean)
  python run_make_esp_firmwares.py --no-clean

  # build with custom parallelism settings
  python run_make_esp_firmwares.py --file-jobs 4 --target-jobs 2

  # build with custom version and no pause (for CI/CD)
  python run_make_esp_firmwares.py --version 1.2.3 --nopause

  # fast incremental build
  python run_make_esp_firmwares.py --no-clean --nopause

  # build and flash a specific target
  python run_make_esp_firmwares.py --target rx-generic-2400-d-pa --flash

notes:
  - uses platformio to build all environments defined in platformio.ini
  - compiled .bin files are copied to tools/esp-build/firmware/ with version suffix
  - parallel builds significantly speed up compilation time
  - clean builds ensure no stale artifacts but take longer

********************************************************
'''
import os
import shutil
import re
import sys
import subprocess
import multiprocessing
import time
import configparser
from concurrent.futures import ThreadPoolExecutor, as_completed


#-- installation dependent
#-- auto-detect platformio


def find_platformio():
    """find platformio executable in system PATH or common locations"""
    
    # first try to find in PATH
    pio_cmd = shutil.which('platformio')
    if pio_cmd:
        return 'platformio'
    
    pio_cmd = shutil.which('pio')
    if pio_cmd:
        return 'pio'
    
    # try common install locations
    if os.name == 'posix':  # mac/linux
        common_paths = [
            os.path.expanduser('~/.platformio/penv/bin/platformio'),
            os.path.expanduser('~/.local/bin/platformio'),
            '/usr/local/bin/platformio',
        ]
    else:  # windows
        common_paths = [
            os.path.join(os.path.expanduser('~'), '.platformio', 'penv', 'Scripts', 'platformio.exe'),
        ]
    
    for path in common_paths:
        if os.path.exists(path):
            return path
    
    return None


PIO_CMD = find_platformio()

if PIO_CMD is None:
    print('ERROR: platformio not found in PATH')
    print('Install with: pip install platformio')
    print('Or: brew install platformio (macOS)')
    sys.exit(1)

print(f'Using platformio: {PIO_CMD}')



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
        wait_for_user()
        sys.exit(1)


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

def create_clean_dir(path):
    if os.path.exists(path):
        shutil.rmtree(path, ignore_errors=True)
    os.makedirs(path, exist_ok=True)


def printWarning(txt):
    print('\033[93m'+txt+'\033[0m') # light yellow


def printError(txt):
    print('\033[91m'+txt+'\033[0m') # light red


def wait_for_user(prompt="Press Enter to continue..."):
    """cross-platform wait for user input"""
    try:
        input(prompt)
    except KeyboardInterrupt:
        print()
        sys.exit(1)


def validate_arguments():
    """validate command-line arguments and suggest corrections for typos"""
    
    # define all valid flags (both long and short forms)
    valid_flags = {
        '--target', '-t', '-T',
        '--define', '-d', '-D',
        '--nopause', '-np', '-NP',
        '--version', '-v', '-V',
        '--no-clean', '-nc', '-NC',
        '--file-jobs', '-fj', '-FJ',
        '--target-jobs', '-tj', '-TJ',
        '--list-targets', '-lt', '-LT',
        '--flash', '-f', '-F',
    }
    
    # flags that expect a value argument
    value_flags = {
        '--target', '-t', '-T',
        '--define', '-d', '-D',
        '--version', '-v', '-V',
        '--file-jobs', '-fj', '-FJ',
        '--target-jobs', '-tj', '-TJ',
    }
    
    # track which arguments are expected values (not flags)
    skip_next = False
    unknown_args = []
    
    for i, arg in enumerate(sys.argv[1:], 1):  # skip script name
        if skip_next:
            skip_next = False
            continue
            
        # check if this looks like a flag
        if arg.startswith('-'):
            if arg not in valid_flags:
                unknown_args.append((i, arg))
            elif arg in value_flags:
                skip_next = True  # next arg is the value
    
    if unknown_args:
        printError('Error: Unrecognized command-line argument(s):')
        for pos, arg in unknown_args:
            printError(f'  {arg}')
        
        print()
        printError('See script header for valid arguments and usage examples')
        sys.exit(1)


#--------------------------------------------------
# build system
#--------------------------------------------------

def get_platformio_environments():
    """parse platformio.ini to get list of build environments"""
    
    config = configparser.ConfigParser()
    config.read(os.path.join(MLRS_PROJECT_DIR, 'platformio.ini'))
    
    environments = []
    for section in config.sections():
        if section.startswith('env:'):
            env_name = section[4:]  # remove 'env:' prefix
            environments.append(env_name)
    
    return environments


def build_single_environment(env_name, file_jobs, clean):
    """build a single PlatformIO environment"""
    start_time = time.time()
    try:
        if clean:
            # "fast clean": directly delete the environment's build directory in .pio/build.
            # this avoids the platformio "startup tax" (2-4s) of a separate 'pio run -t clean' call.
            env_build_dir = os.path.join(MLRS_PIO_BUILD_DIR, env_name)
            if os.path.exists(env_build_dir):
                shutil.rmtree(env_build_dir, ignore_errors=True)
        
        # build the environment with file-level parallelism
        build_cmd = [PIO_CMD, 'run', '--project-dir', MLRS_PROJECT_DIR, 
                    '-e', env_name, '-j', str(file_jobs)]
        
        result = subprocess.run(build_cmd, capture_output=True, text=True)
        
        elapsed = time.time() - start_time
        if result.returncode == 0:
            return (True, env_name, None, elapsed)
        else:
            return (False, env_name, result.stderr, elapsed)
    except Exception as e:
        elapsed = time.time() - start_time
        return (False, env_name, str(e), elapsed)


def display_build_summary(build_info_list, build_start_time):
    """display build summary with compilation times.
    
    args:
        build_info_list: list of dicts with 'env_name', 'compilation_time', and 'success'
        build_start_time: start time for total elapsed calculation
    """
    total_time = time.time() - build_start_time
    
    print('------------------------------------------------------------')
    print('BUILD SUMMARY')
    print('------------------------------------------------------------')
    for info in build_info_list:
        status = '[OK]' if info['success'] else '[FAIL]'
        print(f"{status} {info['env_name']}: {info['compilation_time']:.2f}s")
    
    print('------------------------------------------------------------')
    
    failed_envs = [info['env_name'] for info in build_info_list if not info['success']]
    if failed_envs:
        printError(f'Build completed with {len(failed_envs)} failures in {total_time:.2f}s:')
        for env in failed_envs:
            printError(f'  - {env}')
    else:
        print(f'[OK] All {len(build_info_list)} environments built successfully in {total_time:.2f}s')
    print('------------------------------------------------------------')


def mlrs_esp_compile_all(clean=False, file_jobs=None, target_jobs=None, target_filter=''):
    """compile all ESP targets using platformio with parallel builds
    
    args:
        clean: whether to do a full clean before building
        file_jobs: number of parallel file compilation jobs per target
        target_jobs: number of parallel target builds
        target_filter: optional filter to build only specific targets (environment names)
    
    Returns:
        List of successfully built environment names
    """
    # get all environments from platformio.ini
    environments = get_platformio_environments()
    
    # filter environments if target_filter is specified
    if target_filter:
        environments = [env for env in environments if target_filter in env]
        if not environments:
            printError(f'No environments match target filter: {target_filter}')
            return []
    
    # coordinated parallelism: limit total concurrent jobs to cpu_count * 2
    # this allows I/O overlap while preventing excessive context switching
    total_concurrent = multiprocessing.cpu_count() * 2
    
    # calculate parallelism based on number of environments
    num_envs = len(environments)
    
    if target_jobs is None:
        # target-level parallelism: scale based on CPU count
        # using cpu_count // 2 balances target vs file parallelism across different machines
        target_jobs = max(2, min(num_envs, multiprocessing.cpu_count() // 2))
    
    if file_jobs is None:
        # file-level parallelism: distribute available capacity across concurrent targets
        # this ensures full utilization even if building many targets (where target_jobs is capped at 4)
        file_jobs = max(2, total_concurrent // target_jobs)
    
    total_envs = len(environments)
    
    print(f'Found {total_envs} environments to build')
    
    # determine parallelism based on number of targets
    use_parallel = total_envs > 1
    if use_parallel:
        print(f'Using {target_jobs} parallel targets, {file_jobs} parallel files per target')
    else:
        print(f'Building single target with {file_jobs} parallel files')
    print('------------------------------------------------------------')
    
    print(f'Building {total_envs} environment(s)...')
    
    # build environments
    build_start_time = time.time()
    build_info_list = []
    
    # always use executor for consistent behavior
    with ThreadPoolExecutor(max_workers=target_jobs) as executor:
        # submit all build tasks
        futures = {executor.submit(build_single_environment, env, file_jobs, clean): env 
                  for env in environments}
        
        # collect results as they complete
        for future in as_completed(futures):
            env = futures[future]
            try:
                success, env_name, error, elapsed = future.result()
                build_info_list.append({
                    'env_name': env_name,
                    'success': success,
                    'compilation_time': elapsed
                })
                if success:
                    print(f'[OK] [{len(build_info_list)}/{total_envs}] {env_name}')
                else:
                    printError(f'[FAIL] [{len(build_info_list)}/{total_envs}] {env_name} FAILED')
                    if error:
                        print(f'  Error: {error[:200]}')  # show first 200 chars of error
            except Exception as exc:
                build_info_list.append({
                    'env_name': env,
                    'success': False,
                    'compilation_time': 0
                })
                printError(f'[FAIL] {env} generated exception: {exc}')
    
    display_build_summary(build_info_list, build_start_time)
    
    # return list of successfully built environments
    return [info['env_name'] for info in build_info_list if info['success']]


def flash_esp_target(env_name):
    """flash ESP target using platformio upload command at 921600 baud"""
    
    print('------------------------------------------------------------')
    print(f'Flashing environment: {env_name}')
    print(f'Upload speed: 921600 baud (from platformio.ini)')
    print()
    
    # build platformio upload command
    cmd = [
        PIO_CMD,
        'run',
        '--project-dir', MLRS_PROJECT_DIR,
        '-e', env_name,
        '--target', 'upload'
    ]
    
    print(f'Executing: {" ".join(cmd)}')
    print()
    
    start_time = time.time()
    result = subprocess.run(cmd, capture_output=False, text=True)
    elapsed = time.time() - start_time
    
    if result.returncode == 0:
        print()
        print(f'[OK] Flash completed successfully in {elapsed:.1f}s')
        return True
    else:
        printError(f'[FAIL] Flash failed with return code {result.returncode}')
        return False



#--------------------------------------------------
# application
#--------------------------------------------------

def mlrs_esp_copy_all_bin(environments):
    print('copying .bin files')
    firmwarepath = os.path.join(MLRS_ESP_BUILD_DIR,'firmware')
    create_clean_dir(firmwarepath)
    for subdir in environments:
        env_dir = os.path.join(MLRS_PIO_BUILD_DIR, subdir)
        if os.path.isdir(env_dir):
            print(subdir)
            file = os.path.join(env_dir, 'firmware.bin')
            if os.path.exists(file):
                dest_file = os.path.join(firmwarepath, subdir + '-' + VERSIONONLYSTR + BRANCHSTR + HASHSTR + '.bin')
                shutil.copy(file, dest_file)
            else:
                printWarning(f'Warning: firmware.bin not found for {subdir}')


#-- here we go
if __name__ == "__main__":
    cmdline_target = ''
    cmdline_D_list = []
    cmdline_nopause = False
    cmdline_version = ''
    cmdline_clean = True  # clean by default
    cmdline_file_jobs = None
    cmdline_target_jobs = None
    cmdline_flash = False
    cmdline_list_targets = False

    cmd_pos = -1
    for cmd in sys.argv:
        cmd_pos += 1
        if cmd == '--target' or cmd == '-t' or cmd == '-T':
            if sys.argv[cmd_pos+1] != '':
                cmdline_target = sys.argv[cmd_pos+1]
        if cmd == '--define' or cmd == '-d' or cmd == '-D':
            if sys.argv[cmd_pos+1] != '':
                cmdline_D_list.append(sys.argv[cmd_pos+1])
        if cmd == '--nopause' or cmd == '-np' or cmd == '-NP':
                cmdline_nopause = True
        if cmd == '--version' or cmd == '-v' or cmd == '-V':
            if sys.argv[cmd_pos+1] != '':
                cmdline_version = sys.argv[cmd_pos+1]
        if cmd == '--no-clean' or cmd == '-nc' or cmd == '-NC':
                cmdline_clean = False
        if cmd == '--file-jobs' or cmd == '-fj' or cmd == '-FJ':
            if cmd_pos + 1 < len(sys.argv) and sys.argv[cmd_pos+1].isdigit():
                cmdline_file_jobs = int(sys.argv[cmd_pos+1])
        if cmd == '--target-jobs' or cmd == '-tj' or cmd == '-TJ':
            if cmd_pos + 1 < len(sys.argv) and sys.argv[cmd_pos+1].isdigit():
                cmdline_target_jobs = int(sys.argv[cmd_pos+1])
        if cmd == '--flash' or cmd == '-f' or cmd == '-F':
                cmdline_flash = True
        if cmd == '--list-targets' or cmd == '-lt' or cmd == '-LT':
                cmdline_list_targets = True

    # validate arguments before doing anything expensive
    validate_arguments()

    # initialize global compilation thread pool
    if True:

        try:
            # handle --list-targets early exit
            if cmdline_list_targets:
                print('------------------------------------------------------------')
                print('Available ESP32 targets:')
                print('------------------------------------------------------------')
                environments = get_platformio_environments()
                for env in sorted(environments):
                    print(f'  {env}')
                print('------------------------------------------------------------')
                print(f'Total: {len(environments)} target(s)')
                sys.exit(0)
            
            if cmdline_version == '':
                mlrs_set_version()
                mlrs_set_branch_hash(VERSIONONLYSTR)
            else:
                VERSIONONLYSTR = cmdline_version

            # validate flash requirement: must have exactly one target
            if cmdline_flash and cmdline_target == '':
                printError('Error: --flash requires --target to specify exactly one target')
                printError('Example: python run_make_esp_firmwares.py --target rx-generic-2400-d-pa --flash')
                sys.exit(1)
            
            built_environments = mlrs_esp_compile_all(
                clean=cmdline_clean, 
                file_jobs=cmdline_file_jobs, 
                target_jobs=cmdline_target_jobs,
                target_filter=cmdline_target
            )
            
            if built_environments:
                mlrs_esp_copy_all_bin(built_environments)
            
            # flash via platformio if requested
            if cmdline_flash and len(built_environments) == 1:
                flash_esp_target(built_environments[0])
            elif cmdline_flash and len(built_environments) != 1:
                printError(f'Error: --flash requires exactly one target. Built {len(built_environments)} targets.')
                printError('Use --target to specify a single environment.')

            if not cmdline_nopause:
                wait_for_user()
        finally:
            pass