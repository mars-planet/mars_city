import pytest
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


@pytest.fixture
def client():
    server = Server(__name__)
    server.config['TESTING'] = True
    client = server.test_client()

    yield client


def test_cmd1(client):
    data = {
        'arg1': str(uuid.uuid4()),
        'arg2': str(uuid.uuid4()),
    }
    rv = client.post('/cmd1', json=data)
    assert data == rv.json


if __name__ == '__main__':
    pytest.main([__file__])

