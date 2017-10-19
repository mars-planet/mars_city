import requests


class DeviceProxy:
    def __init__(self, resource_path_URI):
        self.resource_path_URI = resource_path_URI
        self.bind_functions()

    def test_connection(self):
        return 'SUCCESS'

    # Getting the functions of the device server
    def get_function_list(self):
        uri = 'http://127.0.0.1:5000' + self.resource_path_URI + '/functions'
        req = requests.get(uri)
        return req.json()

    # Getting the attributes of the device server
    def get_attribute_list(self):
        uri = 'http://127.0.0.1:5000' + self.resource_path_URI + '/attributes'
        req = requests.get(uri)
        return req.json()

    def add_fn(self, name):
        def fn():
            # Make appropriate REST call here
            uri = 'http://127.0.0.1:5000' + self.resource_path_URI
            uri += '/functions/' + name
            req = requests.get(uri)
            return req.json()
        setattr(self, name, fn)

    def bind_functions(self):
        functions_dict = self.get_function_list()
        functions = functions_dict['functions']

        for name in functions:
            self.add_fn(name)
