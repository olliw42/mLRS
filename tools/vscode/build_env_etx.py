'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
*******************************************************
 build_env_etx.py
 PlatformIO extra script for EdgeTX passthrough upload
 Sends EdgeTX CLI commands to put the internal module
 into bootloader mode, then flashes via esptool
*******************************************************
'''
Import("env")
import serial
import time


def wait_for_prompt(s, timeout=1.0):
    deadline = time.time() + timeout
    buf = b''
    while time.time() < deadline:
        n = s.in_waiting
        if n:
            buf += s.read(n)
            if b'> ' in buf:
                return True
        else:
            time.sleep(0.01)
    return False


def send_command(s, cmd):
    s.write(cmd.encode('utf-8') + b'\n')
    if not wait_for_prompt(s):
        print("  WARNING: no prompt after '%s'" % cmd)


def etx_passthrough_init(port, baud):
    print("======== ETX PASSTHROUGH INIT ========")
    print("  Port: %s @ %s" % (port, baud))

    s = serial.Serial(port=port, baudrate=baud,
        bytesize=8, parity='N', stopbits=1,
        timeout=1, xonxoff=0, rtscts=0)

    s.read(s.in_waiting)  # flush

    send_command(s, 'set pulses 0')
    send_command(s, 'set rfmod 0 power off')
    time.sleep(0.5)
    send_command(s, 'set rfmod 0 bootpin 1')
    time.sleep(0.1)
    send_command(s, 'set rfmod 0 power on')
    time.sleep(0.1)
    send_command(s, 'set rfmod 0 bootpin 0')
    time.sleep(0.1)

    print("  Enabling serial passthrough...")
    cmd = 'serialpassthrough rfmod 0 %s' % baud
    print("  CMD: '%s'" % cmd)
    s.write(cmd.encode('utf-8') + b'\n')
    time.sleep(0.2)

    s.close()
    print("======== PASSTHROUGH DONE ========")


def init_passthrough(source, target, env):
    env.AutodetectUploadPort([env])
    port = env['UPLOAD_PORT']
    etx_passthrough_init(port, env['UPLOAD_SPEED'])


chip = "esp32"
if "ESP32S3" in env.BoardConfig().get("build.mcu", "esp32").upper():
    chip = "esp32s3"

env.Replace(UPLOADERFLAGS=[
    "-b", "$UPLOAD_SPEED", "-p", "$UPLOAD_PORT",
    "-c", chip, "--before", "no_reset", "--after", "hard_reset",
    "write_flash", "-z", "--flash_mode", "dio", "--flash_freq", "80m",
    "--flash_size", "detect"
])
env.AddPreAction("upload", init_passthrough)
