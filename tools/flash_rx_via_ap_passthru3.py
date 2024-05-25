#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 flash_rx_via_ap_passthru3.py
********************************************************
 Does this:
 - determines required baud rate
 - opens serial passthrough in ArduPilot flight controller
 - sets receiver into bootloader mode
 - flashes firmware if firmware file specified
'''

import os
import sys
import time
import argparse


sys.path.append(os.path.join("..",'mLRS','modules','mavlink'))
from pymavlink import mavutil
from pymavlink import mavparm
os.environ['MAVLINK20'] = '1'
mavutil.set_dialect("all")


#--------------------------------------------------
# Connect to ArduPilot flight controller via MAVLink
#--------------------------------------------------

def ardupilot_read_parameter(link, param):
    cmd_tlast = 0
    while True:
        time.sleep(0.01)
        # Check for PARAM_VALUE
        msg = link.recv_match(type = 'PARAM_VALUE')
        if msg is not None and msg.get_type() != 'BAD_DATA':
            msgd = msg.to_dict()
            if msgd['mavpackettype'] == 'PARAM_VALUE':
                return msg
        # Send PARAM_REQUEST_READ
        tnow = time.time()
        if tnow - cmd_tlast >= 0.5:
            cmd_tlast = tnow
            link.mav.param_request_read_send(
                link.target_system, 1, #link.target_component,
                param, #b'SERIAL2_BAUD',
                -1)

def ardupilot_read_baud(link, serialx):
    param = b'SERIAL' + bytes(str(serialx),'ascii') + b'_BAUD'
    msg = ardupilot_read_parameter(link, param)
    baud = int(msg.param_value)
    if baud == 19:
        baud = 19200
    elif baud == 38:    
        baud = 38400
    elif baud == 57:    
        baud = 57600
    elif baud == 115:    
        baud = 115200
    elif baud == 230:    
        baud = 230400
    elif baud == 460:    
        baud = 460800
    return baud


def ardupilot_connect(uart, serialx):
    print('connect to flight controller...')
    link = mavutil.mavlink_connection(uart, 115200)

    print('  wait for heartbeat...')

    msg = link.recv_match(type = 'HEARTBEAT', blocking = True)
    print(' ',msg.to_dict())

    link.wait_heartbeat()

    # Note: link.target_component appears to come out always as 0
    print('  received (sysid %u compid %u)' % (link.target_system, link.target_component))

    baud = ardupilot_read_baud(link, 2)
    if baud != 115200:
        link.close()
        link = mavutil.mavlink_connection(uart, baud)
        #print('  wait for heartbeat...')
        msg = link.recv_match(type = 'HEARTBEAT', blocking = True)
        #print(' ',msg.to_dict())
        #print('  received (sysid %u compid %u)' % (link.target_system, link.target_component))
    
    print('connected to flight controller')
    return link, baud


#--------------------------------------------------
# Open serial passthrough on ArduPilot flight controller
#--------------------------------------------------

def ardupilot_open_passthrough(link, serialx, passthru_tmo_s):
    print('open serial passthrough...')
    # Set up passthrough with no timeout, power cycle to end
    print('  set SERIAL_PASSTIMO =', passthru_tmo_s)
    mavparm.MAVParmDict().mavset(link, "SERIAL_PASSTIMO", passthru_tmo_s)
    print('  set SERIAL_PASS2 =', serialx)
    mavparm.MAVParmDict().mavset(link, "SERIAL_PASS2", serialx)
    # Wait for passthrough to start
    time.sleep(1.5)
    print('serial passthrough opened')


def ardupilot_close_passthrough(link, serialx):
    print('close serial passthrough...')
    print('  set SERIAL_PASS2 = -1')
    mavparm.MAVParmDict().mavset(link, "SERIAL_PASS2", -1)
    time.sleep(1.5)
    print('serial passthrough closed')


#--------------------------------------------------
# Verify connection to mLRS receiver
# Reboot mLRS receiver
#--------------------------------------------------

# confirmation, send 0 to ping, send 1 to arm, then 2 to execute
def mlrs_cmd_preflight_reboot_shutdown(link, cmd_confirmation, cmd_action, sysid=51, compid=68):
    cmd_tlast = 0
    cmd_tmo_counter = 10 # don't do more than that many attempts
    while True:
        time.sleep(0.01)
        # Check for ACK 
        msg = link.recv_match(type = 'COMMAND_ACK')
        if msg is not None and msg.get_type() != 'BAD_DATA':
            msgd = msg.to_dict()
            if (msgd['mavpackettype'] == 'COMMAND_ACK' and
                msgd['command'] == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN and
                msgd['result_param2'] == 1234321):
                print('  ACK:', msgd)
                #print(mavutil.mavlink.enums['MAV_RESULT'][msgd['result']].description)
                return True
        # Send COMMAND
        tnow = time.time()
        if tnow - cmd_tlast >= 0.5:
            cmd_tlast = tnow
            print('  send probe')
            link.mav.command_long_send(
                sysid, compid,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                cmd_confirmation, # confirmation, send 0 to ping, send 1 to arm, then 2 to execute
                0, 0,
                cmd_action, # param 3: Component action
                compid, # param 4: Component ID
                0, 0,
                1234321)
            cmd_tmo_counter -= 1
            if cmd_tmo_counter <= 0: return False            


def mlrs_put_into_systemboot(link, sysid=51, compid=68):
    print('check connection to mLRS receiver...')
    res = mlrs_cmd_preflight_reboot_shutdown(link, 0, 0, sysid=sysid, compid=compid)
    if not res:
        print('ERROR: Could not connect to mLRS receiver!')
        exit(1)
    print('mLRS receiver connected')
    # Arm receiver to allow reboot
    print('arm mLRS receiver for reboot shutdown...')
    res = mlrs_cmd_preflight_reboot_shutdown(link, 1, 3, sysid=sysid, compid=compid)
    if not res:
        print('ERROR: Could not arm mLRS receiver for reboot shutdown!')
        exit(1)
    print('mLRS receiver armed for reboot shutdown')
    # Reboot shutdown
    print('mLRS receiver reboot shutdown...')
    res = mlrs_cmd_preflight_reboot_shutdown(link, 2, 3, sysid=sysid, compid=compid)
    # sometimes we don't get an ACK, usually receiver is nevertheless in system boot, so just ignore
    print('mLRS receiver reboot shutdown DONE')
    # Done    
    print('mLRS receiver jumps to system bootloader in 5 seconds')


def mlrs_run_all(uart, serialx, passthru_tmo_s = 0):
    print('USB port:', uart)
    print('SERIALx number:', serialx)
    print('------------------------------------------------------------')
    link, baud = ardupilot_connect(uart, serialx)
    print('Baud rate:', baud)
    print('------------------------------------------------------------')
    ardupilot_open_passthrough(link, serialx, passthru_tmo_s)
    print('------------------------------------------------------------')
    mlrs_put_into_systemboot(link)
    print('------------------------------------------------------------')
    #link.close()
    print('')
    print('PASSTHROUGH READY FOR PROGRAMMING TOOL')
    return link, baud


#--------------------------------------------------
# STM32CubeProgrammer
#--------------------------------------------------

def mlrs_flash_frimware_file(uart, baud, file, full_erase_flag):
    print('')
    print('------------------------------------------------------------')
    print('flashing firmware file...')
    arg = '-c port=' + uart + ' br=' + baud
    #' -rdu'
    if full_erase_flag:
        arg += ' -e all'
    arg += ' -w "' + file + '"'
    arg += ' -v'
    arg += ' -g'
    res = os.system(os.path.join('st-programmer','bin','STM32_Programmer_CLI.exe') + ' ' + arg)
    if res > 0:
        print('ERROR: Flashing firmware failed!')
        exit(1)
    print('------------------------------------------------------------')
    print('firmware flashed')


#--------------------------------------------------
# Main
#--------------------------------------------------

if len(sys.argv) > 1:
    parser = argparse.ArgumentParser()
    parser.add_argument("com", help="Com port corresponding to flight controller USB port. Examples: com5, /dev/ttyACM0")
    parser.add_argument("serial", help="ArduPilot SERIALx number of the receiver", type=int)
    args = parser.parse_args()

    uart = args.com
    serial = args.serial
    link, baud = mlrs_run_all(uart, serial)
    link.close()

else:
    from tkinter import *
    import tkinter.filedialog
    import tkinter.messagebox

    app_title = "mLRS Receiver Flash Tool"
    
    import serial.tools.list_ports
    coms = serial.tools.list_ports.comports(include_links = False)
    comport_list = []
    for com in sorted(coms):
        comport_list.append(com.device)

    class Application(Frame):
        def __init__(self, master=None):
            Frame.__init__(self, master)
            self.pack_propagate(0)
            self.grid(sticky = N + S + E + W)
            self.createWidgets()

        # Creates the gui and all of its content.
        def createWidgets(self):
            row = 0
            self.usbport_value = StringVar()
            self.usbport_choices = comport_list #['COM1','COM2','COM5']
            self.usbport_value.set(comport_list[0]) #set('COM1')
            self.usbport_label = Label(self, text = "USB port")
            self.usbport_label.grid(row = row, column = 0, sticky = W)
            self.usbport_menu = OptionMenu(self, self.usbport_value, *self.usbport_choices)
            self.usbport_menu.config(width = 10)
            self.usbport_menu.grid(row = row, column = 1, sticky = W)
            row = 1
            self.serial_value = StringVar()
            self.serial_choices = ['1','2','3','4','5','6','7','8']
            self.serial_value.set('1')
            self.serial_label = Label(self, text = "SERIALx No.")
            self.serial_label.grid(row = row, column = 0, sticky = W)
            self.serial_menu = OptionMenu(self, self.serial_value, *self.serial_choices)
            self.serial_menu.config(width = 10)
            self.serial_menu.grid(row = row, column = 1, sticky = W)
            row = 3
            self.firmware_file = StringVar()
            self.firmware_file.set('')
            self.firmware_file_label = Label(self, text = "Firmware binary")
            self.firmware_file_label.grid(row = row, column = 0, sticky = W)
            self.firmware_file_entry = Entry(self, width = 80, textvariable = self.firmware_file)
            self.firmware_file_entry.grid(row = row, column = 1)
            self.firmware_file_button = Button(self, text="Browse", command = self.browseFirmwareFile)
            self.firmware_file_button.grid(row = row, column = 2)
            row = 4
            self.full_erase_value = BooleanVar()
            self.full_erase_label = Label(self, text = "Full erase")
            self.full_erase_label.grid(row = row, column = 0)
            self.full_erase_button = Checkbutton(self, variable = self.full_erase_value, onvalue = True, offvalue = False)
            self.full_erase_value.set(False)
            self.full_erase_button.grid(row = row, column = 1, sticky = W)
            row = 5
            self.ok_button = Button(self, text = "OK", command = self.okRun)
            self.ok_button.config(width = 10)
            self.ok_button.grid(row = row, column = 1, sticky = W)

        def browseFirmwareFile(self):
            file = tkinter.filedialog.askopenfilename(parent = self, title = 'Choose a firmware file', initialdir = os.path.join('..','firmware'))
            if file != None:
                self.firmware_file.set(file)

        def okRun(self):
            uart = self.usbport_value.get()
            serialx = self.serial_value.get()
            link, baud = mlrs_run_all(uart, serialx, passthru_tmo_s = 30)
            binary = self.firmware_file.get()
            full_erase_flag = self.full_erase_value.get()
            if binary != '':
                print('')
                print('waiting...')
                link.close()    
                time.sleep(5.0)
                mlrs_flash_frimware_file(uart, str(baud), binary, full_erase_flag)
            #exit(0)

    #if __name__ == '__main__':
    app = Application()
    app.master.minsize(300, 180)
    app.master.title(app_title)
    app.mainloop()

