import requests
from multiprocessing import Process
from time import sleep
from unittest import TestCase
import uuid

from server import ServerBase, Command


class Server(ServerBase):
    """
    Sample server.
    """
    @Command
    def cmd1(self, arg1, arg2):
        ret_val = {
            'arg1': arg1,
            'arg2': arg2,
        }
        return ret_val


class ServerTests(TestCase):
    def setUp(self):
        server = Server(__name__)
        server.config['TESTING'] = True
        self.client = server.test_client()

    def test_cmd1_client(self):
        data = {
            'arg1': str(uuid.uuid4()),
            'arg2': str(uuid.uuid4()),
        }
        rv = self.client.post('/cmd1', json=data)
        self.assertEqual(data, rv.json)

    def test_cmd1_instance(self):
        data = {
            'arg1': str(uuid.uuid4()),
            'arg2': str(uuid.uuid4()),
        }
        server = Server(__name__)
        rv = server.cmd1(**data)
        self.assertEqual(data, rv)

    def test_cmd1_app_server(self):
        server = Server(__name__)
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
