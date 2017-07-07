from __future__ import absolute_import, division, print_function
import PyTango

__author__ = 'abhijith'

'''
Client side program for starting the Biometric Tango Device Server
'''

bm = PyTango.DeviceProxy("C3/biometric_monitor/1")

try:
    bm.start_monitoring()
except:
    pass
