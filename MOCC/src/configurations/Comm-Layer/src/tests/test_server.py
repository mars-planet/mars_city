import unittest
import sys
sys.path.insert(0, '..')
from server import Device, command

class DeviceProxy(Device):
    def __init__(self):
        super().__init__()

    @command
    def test_command(self, arg1, arg2=None):
        return "999"


dp = DeviceProxy()
dp.test_command(10)
dp.run_server()
