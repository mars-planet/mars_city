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


@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1.*$', path='/tango_dev_test/read_attr')
def read_attr_mock(url, request):
    attr_value_dict = {'abc': 4.0, 'def': 10}
    attr_name = url[3].split('=')[1]
    return json.dumps(attr_value_dict[attr_name])


@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1.*$', path='/tango_dev_test/write_attr', method='post')
def write_attr_mock(url, request):
    attr_value_dict = {'abc': 4.0, 'def': 10}
    attr_name = request.body.split('=')[0]
    attr_value = request.body.split('=')[1]
    if attr_name in attr_value_dict:
        attr_value_dict[attr_name] = attr_value
    else:
        return json.dumps({'message': 'error', 'error': 'Attribute ' + str(attr_name) + ' is undefined'})

    return json.dumps({'message': 'successful'})


@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1.*$', path='/tango_dev_test/information')
def info_mock(url, request):
    return json.dumps({'last_updated': datetime.datetime.now().isoformat(), 'description': 'Hello world! This is a description of the device tango_dev_test'})


@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1.*$', path='/tango_dev_test/status')
def status_mock(url, request):
    return json.dumps({'status': 'active', 'last_updated': datetime.datetime.now().isoformat()})


@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1.*$', path='/tango_dev_test/write_attrs')
def write_attrs_mock(url, request):
    attr_value_dict = {'abc': 4.0, 'def': 11}
    pairs = request.body.split('&')
    count = 0
    for pair in pairs:
        attr_name = pair.split('=')[0]
        attr_value = pair.split('=')[1]
        try:
            attr_value_dict[attr_name] = attr_value
        except Exception as e:
            return json.dumps({'message': 'error', 'error': 'Attribute ' + str(attr_name) + ' is undefined'})
        count += 1
    return json.dumps({'message': 'successful', 'count': count})

class DeviceProxyTest(unittest.TestCase):
    def setUp(self):
        with HTTMock(server_mock, functions_mock):
            self.dev_proxy = device_proxy.DeviceProxy('tango_dev_test'
                    , 'http://127.0.0.1')

    def test_black_box(self):
        print("Testing <device>/black_box")
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

    def test_read_attribute(self):
        print("Testing <device>/read_attribute")
        with HTTMock(read_attr_mock):
            attribute_value = self.dev_proxy.read_attribute('abc')
            assert type(attribute_value) is float
            assert attribute_value == 4.0

            attribute_value = self.dev_proxy.read_attribute('def')
            assert type(attribute_value) is int
            assert attribute_value == 10

    def test_write_attribute(self):
        print("Testing <device>/write_attribute")
        with HTTMock(write_attr_mock):
            write_result = self.dev_proxy.write_attribute('abc', 5.0)
            assert 'successful' in write_result['message']
            assert write_result.get('error', None) is None

            write_result = self.dev_proxy.write_attribute('xyz', 'on')
            assert 'error' in write_result['message']
            assert write_result.get('error', None) is not None

    def test_write_attributes(self):
        print("Testing <device>/write_attributes")
        with HTTMock(write_attrs_mock):
            write_result = self.dev_proxy.write_attributes({'abc': 5.0, 'def': 10})
            assert 'error' not in write_result.keys()
            assert 'successful' in write_result['message']
            assert write_result['count'] == 2

    def test_information(self):
        print("Testing <device>/information")
        with HTTMock(info_mock):
            information = self.dev_proxy.info()
            assert information['last_updated'] is not None
            assert type(information['description']) is str

    def test_status(self):
        print("Testing <device>/status")
        with HTTMock(status_mock):
            status = self.dev_proxy.status()
            assert status['last_updated'] is not None
            assert status['status'] == 'active'


if __name__ == '__main__':
    unittest.main()
