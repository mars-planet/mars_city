import PyTango
import sys

# a faster way to take input from 'stdin' is -
# os.read(0, BUFFER_SIZE)

sys.stdout.write(
    "This script is an easy way to setup a Device in the Tango database\nfrom the command line\n\n")
# A structure containing available information for a device with
# the following members:
#    - name : (str) name
#    - _class : (str) device class
#    - server : (str) server

# 'raw_input(Prompt)'' has been renamed to 'input(Prompt)' in Python 3.X '
device_info = PyTango.DbDevInfo()
device_info.server = raw_input(
    "Server Name (eg. telerobotics/instance_name) : ")
device_info._class = raw_input("Class Name (eg. PyTelerobotics): ")
device_info.name = raw_input(
    "Canonical Device Name (eg. C3/GUI/telerobotics) : ")

# Instance of the Tango Database (device server)

tango_database = PyTango.Database()

# Add the device to the database

tango_database.add_device(device_info)

# Need to add PyTango exceptions here

sys.stdout.write("Device Successfully added!\n")
