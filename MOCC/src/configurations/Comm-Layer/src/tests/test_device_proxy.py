import unittest
import sys
sys.path[0] = '../'
from httmock import urlmatch, HTTMock
import json
import device_proxy
import datetime


@urlmatch(netloc=r'(.*\.)?127\.0\.0\.1.*$')
def server_mock(url, request):
    return json.dumps({'tango_addr': 'tango_dev_test', 'ip_addr': 'http://192.168.1.1'})


@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1.*$', path='/tango_dev_test/black_box')
def black_box_mock(url, request):
    return json.dumps([{'command': 'xyz', 'time': datetime.datetime.today().isoformat()}, {'command': 'abc', 'time': datetime.datetime.today().isoformat()}])


@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1.*$', path='/tango_dev_test/attr_list')
def attr_list_mock(url, request):
    return json.dumps(['attr1', 'attr2', 'attr3', 'attr4'])


@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1.*$', path='/tango_dev_test/functions')
def functions_mock(url, request):
    return json.dumps({'functions': ['xyz', 'abc']})


class DeviceProxyTest(unittest.TestCase):
    def setUp(self):
        with HTTMock(server_mock, functions_mock):
            self.dev_proxy = device_proxy.DeviceProxy('tango_dev_test'
                    , 'http://127.0.0.1')

    def test_black_box(self):
        print("Testing /<device>/black_box")
        with HTTMock(black_box_mock):
            n = 2
            black_box_commands = self.dev_proxy.black_box(n)
            assert type(black_box_commands) is list 
            assert len(black_box_commands) == n
            assert black_box_commands[0]['command'] == 'xyz'
            assert black_box_commands[1]['command'] == 'abc'

    def test_attr_list(self):
        print("Testing <device>/attr_list")
        with HTTMock(attr_list_mock):
            attr_list = self.dev_proxy.attribute_list_query()
            assert type(attr_list) is list
            assert len(attr_list) == 4
            assert attr_list[0] == 'attr1'
            assert attr_list[1] == 'attr2'
            assert attr_list[2] == 'attr3'
            assert attr_list[3] == 'attr4'




if __name__ == '__main__':
    unittest.main()
