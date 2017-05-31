import unittest
import PyTango


class ResourceManagerTestCase(unittest.TestCase):

    def test_ping_non_negative(self):
        device = PyTango.DeviceProxy('test/resource_manager/1')
        self.assertTrue(device.ping() > 0)

    def test_get_resource_water(self):
        device = PyTango.DeviceProxy('test/resource_manager/1')
        self.assertTrue(device.ask_resource("water"), 'water # consumable # 2017-05-31 07:35:34.433790 # 2017-05-31 07:35:34.433790 # 74.443 # 5254.2342')


if __name__ == '__main__':
    unittest.main()
