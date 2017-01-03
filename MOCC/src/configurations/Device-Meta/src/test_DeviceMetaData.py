import unittest
import PyTango


class DeviceMetaTests(unittest.TestCase):

    def test_response(self):
        a = PyTango.DeviceProxy("devicemetadata/test/1")
        response = '{"attribute_types": "TANGO", "commands": [{"Init"\
: {"arguments": {"Uninitialised": {"position": 1, "type": "DevVoid"}}, \
"return_object": {"Uninitialised": {"type": "DevVoid"}}}}, {"State":\
 {"arguments": {"Uninitialised": {"position": 1, "type": "DevVoid"}},\
 "return_object": {"Device state": {"type": "DevState"}}}}, {"Status": \
{"arguments": {"Uninitialised": {"position": 1, "type": "DevVoid"}},\
 "return_object": {"Device status": {"type": "DevString"}}}},\
 {"get_device_meta": {"arguments": {"Device id": {"position": 1, "type":\
 "DevString"}}, "return_object": {"JSON response - device metadata":\
 {"type": "DevString"}}}}], "attributes": [{"State": {"writeable":\
 false, "readable": true, "type": "19"}}, {"Status": {"writeable":\
 false, "readable": true, "type": "8"}}]}'
        self.assertEqual(a.get_device_meta("devicemetadata/test/1"), response)


if __name__ == '__main__':
    unittest.main()
