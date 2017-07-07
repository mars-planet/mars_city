from __future__ import absolute_import, division, print_function
import PyTango

__author__ = 'abhijith'

'''
Script to register the Biometric Monitor Tango Server
'''

dev_info = PyTango.DbDevInfo()
dev_info.server = "Monitor/monitor"
dev_info._class = "Monitor"
dev_info.name = "C3/biometric_monitor/1"

print("Creating device: %s" % dev_info.name)
db = PyTango.Database()
db.add_device(dev_info)
