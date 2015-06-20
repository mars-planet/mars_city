"""Command Line Interface to setup or delete a Device from the Tango database."""
from __future__ import print_function
import PyTango
import sys

try:
    input = raw_input
except NameError:
    pass

# A function which prints the received message and returns the input
def prompt(message):

    # Using Python version agnostic functions to take input efficiently
    user_input = input(message)
    user_input = user_input.replace("\n", "")
    return user_input

def register(tango_database, device_info):

    try:
        # Add the device to the database
        tango_database.add_device(device_info)
        print("\nDevice Successfully added and registered!\n")
    except PyTango.DevFailed as err:
        PyTango.Except.print_exception(err)
        sys.exit()

def unregister(tango_database, device_info):

    try:
        # Remove the device from the database
        tango_database.delete_server(device_info.server)
        tango_database.delete_device(device_info.name)
        print("\nDevice Successfully unregistered and removed!\n")

    except PyTango.DevFailed as err:
        PyTango.Except.print_exception(err)
        sys.exit()

def init():

    # check for validity of arguments
    if len(sys.argv) != 2:
        print(
            "This interactive script takes one argument\n"
            "Register format: python setup-device.py <add,setup,register>\n"
            "Unregister format: python setup-device.py <del,delete,unregister>\n")
        sys.exit(255)

    print(
        "This script is an easy way to setup/delete a Device in the Tango"
        " database from the command line\nEnter the Device details below:\n")
        # 'device_info' is a structure containing available information for
    # a device with the following members:
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

    if sys.argv[1] in ('add', 'setup', 'register'):
        register(tango_database, device_info)

    elif sys.argv[1] in ('del', 'delete', 'unregister'):
        unregister(tango_database, device_info)

    else:
        print("Argument not understood. Try again.\n")

if __name__ == '__main__' :
    init()
