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


@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1.*$', path='/tango_dev_test/write_attrs', method='post')
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

@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1\.*$', path='/tango_dev_test/read_attrs')
def read_attrs_mock(url, request):
    attr_value_dict = {'abc': 4.0, 'def': 10}
    attr_names = eval(url[3].split('=')[1])
    return_dict = dict()
    for name in attr_names:
        try:
            return_dict[name] = attr_value_dict[name]
        except Exception as e:
            return json.dumps({'_error': e.message()})
    return json.dumps(return_dict)


@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1\.*$', path='/tango_dev_test/command_history')
def cmd_hist_mock(url, request):
    cmd_history = [
        {'name': 'abc', 'time': datetime.datetime.isoformat(datetime.datetime.now())}, 
        {'name': 'def', 'time': datetime.datetime.isoformat(datetime.datetime.now())}
        ]
    return json.dumps(cmd_history)

@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1\.*$', path='/tango_dev_test/command_query')
def cmd_query_mock(url, request):
    cmd_description_dict = {
        'abc': {
            'description': 'This command performs the function abc',
            'last_used': datetime.datetime.isoformat(datetime.datetime.now())
        }
    }
    command_name = url[3].split('=')[1]
    return json.dumps(cmd_description_dict[command_name])

@urlmatch(netloc=r'(.*\.)?192\.168\.1\.1\.*$', path='/tango_dev_test/delete_property', method='delete')
def del_prop_mock(url, request):
    available_properties = set(['property1', 'property2'])
    prop_name = request.body.split('=')[1]
    try:
        available_properties.remove(prop_name)
    except KeyError:
        return json.dumps({'message': 'error', 'error': 'The requested property is not available'})

    return json.dumps({'message': 'Successfully removes property ' + prop_name})

class DeviceProxyTest(unittest.TestCase):
    def setUp(self):
        with HTTMock(server_mock, functions_mock):
            self.dev_proxy = device_proxy.DeviceProxy('tango_dev_test', 'http://127.0.0.1')

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

    def test_read_attributes(self):
        print("Testing <device>/read_attributes")
        with HTTMock(read_attrs_mock):
            attr_list = ['abc', 'def']
            attribute_values = self.dev_proxy.read_attributes(attr_list)
            assert '_error' not in attribute_values.keys()
            assert type(attribute_values) is dict
            assert len(attribute_values.keys()) == len(attr_list)

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

    def test_command_history(self):
        print("Testing <device>/command_history")
        with HTTMock(cmd_hist_mock):
            cmd_history = self.dev_proxy.command_history()
            assert len(cmd_history) == 2

    def test_command_query(self):
        print("Testing <device>/command_query")
        with HTTMock(cmd_query_mock):
            cmd_query_result = self.dev_proxy.command_query('abc')
            assert type(cmd_query_result['description']) is str
            assert type(cmd_query_result) is dict

    def test_delete_propert(self):
        print("Testing <device>/delete_property")
        with HTTMock(del_prop_mock):
            del_prop_result = self.dev_proxy.delete_property('property1')
            assert type(del_prop_result) is dict
            assert 'error' not in del_prop_result
            assert 'Successfully' in del_prop_result['message']

if __name__ == '__main__':
    unittest.main()
