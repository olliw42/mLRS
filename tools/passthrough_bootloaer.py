#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 passthrough_bootloader.py
********************************************************
'''


import argparse
import os
import sys
import time


from pymavlink import mavutil
from pymavlink import mavparm

parser = argparse.ArgumentParser()
parser.add_argument("device", help="Serial device name corresponding to Flight controller USB connection.  examples: com5, /dev/ttyACM0")
parser.add_argument("serial", help="Ardupilot serial port number for Rx (MAVLink)", type=int)
parser.add_argument("baud", help="Speed of Rx serial connection (MavLink)", type=int)
args = parser.parse_args()

uart = args.device
baud = args.baud # must match speed of serial link to receiver
serial = args.serial

os.environ['MAVLINK20'] = '1'
mavutil.set_dialect("all")


print('connect...')

link = mavutil.mavlink_connection(uart, baud)

print('wait for heartbeat...')

msg = link.recv_match(type = 'HEARTBEAT', blocking = True)
print(msg.to_dict())

link.wait_heartbeat()

# Note: link targets appear to come out always 1,0
print('received (sysid %u compid %u)' % (link.target_system, link.target_component))

print('connected to FC')


cmd_tlast = 0

# Set up passthrough with no timeout, power cycle to end
mavparm.MAVParmDict().mavset(link, "SERIAL_PASSTIMO", 0)

mavparm.MAVParmDict().mavset(link, "SERIAL_PASS2", serial)

time.sleep(1.5) # Wait for passthrough to start

# Verify connection to mLRS Rx

while True:

    time.sleep(0.01)

    msg = link.recv_match(type = 'COMMAND_ACK')
    if msg is not None and msg.get_type() != 'BAD_DATA':
        msgd = msg.to_dict() 
        if (msgd['mavpackettype'] == 'COMMAND_ACK' and
            msgd['command'] == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN and
            msgd['result_param2'] == 1234321):
            print('ACK:', msgd)
            #print(mavutil.mavlink.enums['MAV_RESULT'][msgd['result']].description)
            break
        
    tnow = time.time()
    if tnow - cmd_tlast >= 0.5:
        cmd_tlast = tnow
        print('send probe')
        link.mav.command_long_send(
            51, 68,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, # confirmation, send 0 to ping, send 1 to arm, then 2 to execute
            0, 0, 
            0, # param 3: Component action
            68, # param 4: Component ID
            0, 0, 
            1234321)
            
print('mLRS Rx connected')

# Arm reboot

while True:
    time.sleep(0.01)
    
    msg = link.recv_match(type = 'COMMAND_ACK')
    if msg is not None and msg.get_type() != 'BAD_DATA':
        msgd = msg.to_dict() 
        if (msgd['mavpackettype'] == 'COMMAND_ACK' and
            msgd['command'] == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN and
            msgd['result_param2'] == 1234321 and
            msgd['result'] == mavutil.mavlink.MAV_RESULT_ACCEPTED and
            msgd['progress'] == 1):
            print('ACK:', msgd)
            #print(mavutil.mavlink.enums['MAV_RESULT'][msgd['result']].description)
            break
        
    tnow = time.time()
    if tnow - cmd_tlast >= 0.5:
        cmd_tlast = tnow
        print('send arm')
        link.mav.command_long_send(
            51, 68,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            1, # confirmation, send 0 to ping, send 1 to arm, then 2 to execute
            0, 0, 
            3, # param 3: Component action
            68, # param 4: Component ID
            0, 0, 
            1234321)
            
print('mLRS reboot shutdown armed')


# Reboot shutdown

while True:
    time.sleep(0.01)
    
    msg = link.recv_match(type = 'COMMAND_ACK')
    if msg is not None and msg.get_type() != 'BAD_DATA':
        msgd = msg.to_dict() 
        if (msgd['mavpackettype'] == 'COMMAND_ACK' and
            msgd['command'] == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN and
            msgd['result_param2'] == 1234321 and
            msgd['result'] == mavutil.mavlink.MAV_RESULT_ACCEPTED and
            msgd['progress'] == 2):
            print('ACK:', msgd)
            #print(mavutil.mavlink.enums['MAV_RESULT'][msgd['result']].description)
            break
        
    tnow = time.time()
    if tnow - cmd_tlast >= 0.5:
        cmd_tlast = tnow
        print('send execute')
        link.mav.command_long_send(
            51, 68,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            2, # confirmation, send 0 to ping, send 1 to arm, then 2 to execute
            0, 0, 
            3, # param 3: Component action
            68, # param 4: Component ID
            0, 0, 
            1234321)

print('mLRS reboot shutdown done')

print('Passthrough mode ready for programming tool')

exit(0)
