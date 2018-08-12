import requests
from multiprocessing import Process
from time import sleep
import unittest
from unittest import TestCase
import uuid

from server import ServerBase, Command, Attribute


class ServerWithCommand(ServerBase):
    @Command
    def cmd1(self, arg1, arg2):
        ret_val = {
            'arg1': arg1,
            'arg2': arg2,
        }
        return ret_val


class ServerWithGetter(ServerBase):
    def __init__(self, name):
        super().__init__(name=name)
        self._attr1 = 'some str'

    @Attribute
    @property
    def attr1(self):
        return self._attr1


class ServerWithGetterAndSetter(ServerBase):
    def __init__(self, name):
        super().__init__(name=name)
        self._attr1 = 'some str'

    @property
    def attr1(self):
        return self._attr1

    @Attribute
    @attr1.setter
    def attr1(self, val):
        self._attr1 = val


class ServerTests(TestCase):
    def setUp(self):
        server_with_command = ServerWithCommand(__name__)
        server_with_command.config['TESTING'] = True
        self.command_client = server_with_command.test_client()

        server_with_getter = ServerWithGetter(__name__)
        server_with_getter.config['TESTING'] = True
        self.getter_client = server_with_getter.test_client()

        server_with_getter_and_setter = ServerWithGetterAndSetter(__name__)
        server_with_getter_and_setter.config['TESTING'] = True
        self.getter_setter_client = server_with_getter_and_setter.test_client()

    def test_cmd1_client(self):
        data = {
            'arg1': str(uuid.uuid4()),
            'arg2': str(uuid.uuid4()),
        }
        rv = self.command_client.post('/cmd1', json=data)
        self.assertEqual(data, rv.json)

    def test_cmd1_instance(self):
        data = {
            'arg1': str(uuid.uuid4()),
            'arg2': str(uuid.uuid4()),
        }
        server = ServerWithCommand(__name__)
        rv = server.cmd1(**data)
        self.assertEqual(data, rv)

    def test_cmd1_app_server(self):
        server = ServerWithCommand(__name__)
        host = 'localhost'
        port = 9090
        p = Process(target=server.run, kwargs={'host': host, 'port': port})
        try:
            p.start()
            sleep(1)
            data = {
                'arg1': str(uuid.uuid4()),
                'arg2': str(uuid.uuid4()),
            }
            rv = requests.post(f'http://{host}:{port}/cmd1', json=data)
            self.assertEqual(data, rv.json())
        finally:
            p.terminate()

    def test_attr1_getter_client(self):
        expected_value = 'some str'
        rv = self.getter_client.get('/attr1')
        self.assertEqual(expected_value, rv.json)

    def test_attr1_getter_instance(self):
        expected_value = 'some str'
        server = ServerWithGetter(__name__)
        rv = server.attr1
        self.assertEqual(expected_value, rv)

    def test_attr1_getter_app_server(self):
        server = ServerWithGetter(__name__)
        host = 'localhost'
        port = 9090
        p = Process(target=server.run, kwargs={'host': host, 'port': port})
        try:
            p.start()
            sleep(1)
            expected_value = 'some str'
            rv = requests.get(f'http://{host}:{port}/attr1')
            self.assertEqual(expected_value, rv.json())
        finally:
            p.terminate()

    def test_attr1_getter_setter_client(self):
        expected_value = 'some other str'
        self.getter_setter_client.put('/attr1', json=expected_value)
        rv = self.getter_setter_client.get('/attr1')
        self.assertEqual(expected_value, rv.json)

    def test_attr1_getter_setter_instance(self):
        expected_value = 'some other str'
        server = ServerWithGetterAndSetter(__name__)
        server.attr1 = expected_value
        rv = server.attr1
        self.assertEqual(expected_value, rv)

    def test_attr1_getter_setter_app_server(self):
        server = ServerWithGetterAndSetter(__name__)
        host = 'localhost'
        port = 9090
        p = Process(target=server.run, kwargs={'host': host, 'port': port})
        try:
            p.start()
            sleep(1)
            expected_value = 'some other str'
            requests.put(f'http://{host}:{port}/attr1', json=expected_value)
            rv = requests.get(f'http://{host}:{port}/attr1')
            self.assertEqual(expected_value, rv.json())
        finally:
            p.terminate()


if __name__ == '__main__':
    unittest.main()