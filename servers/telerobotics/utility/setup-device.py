#!/usr/bin/env python2.7

import PyTango
import sys

# A function which prints the received message and returns the input
def prompt(message):

    # display the message prompt to standard output

    sys.stdout.write(message)

    # Using Python version agnostic functions to take input efficiently
    user_input = sys.stdin.readline()
    user_input = user_input.replace("\n", "")
    return user_input

# check for validity of arguments
if len(sys.argv) != 1:
    sys.stdout.write(
        "This interactive script takes no arguments\nFormat: python setup-device.py\n")
    sys.exit(255)

sys.stdout.write(
    "This script is an easy way to setup a Device in the Tango" \
    " database from the command line\n\n")

# 'device_info' is a structure containing available information for a device with
# the following members:
#    - name : (str) name
#    - _class : (str) device class
#    - server : (str) server

device_info = PyTango.DbDevInfo()
device_info.server = prompt(
    "Server Name (eg. telerobotics/instance_name) : ")
device_info._class = prompt("Class Name (eg. PyTelerobotics): ")
device_info.name = prompt(
    "Canonical Device Name (eg. C3/GUI/telerobotics) : ")

# Get an Instance of the Tango Database (device server)
tango_database = PyTango.Database()

# Add the device to the database
tango_database.add_device(device_info)

# Need to add PyTango exceptions here
sys.stdout.write("Device Successfully added!\n")
