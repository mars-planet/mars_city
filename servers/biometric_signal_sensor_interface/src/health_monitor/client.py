from __future__ import absolute_import, division, print_function
import PyTango

__author__ = 'abhijith'

'''
Client side program for starting the Biometric Tango Device Server
'''

bm = PyTango.DeviceProxy("C3/biometric_monitor/1")

bm.poll_command("rt_to_gui", 50000)
print(bm.get_command_poll_period("rt_to_gui"))
print(bm.rt_to_gui())
