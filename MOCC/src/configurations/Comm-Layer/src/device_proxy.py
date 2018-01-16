import requests
from typing import List, Dict

class DeviceProxy:
    def __init__(self, dev_name, rest_server_add):
        self.dev_name = dev_name
        r = requests.get(url=rest_server_add + '/get_addr/' + dev_name)
        data = r.json()
        self.ip_addr = data['ip_addr']
        self.bind_functions()

    def test_connection(self):
        return 'SUCCESS'

    def add_fn(self, name: str):
        def fn():
            # Make appropriate REST call here
            uri = self.ip_addr + '/' + self.dev_name
            uri += '/functions/' + name
            req = requests.get(uri)
            return req.json()
        setattr(self, name, fn)

    # Get admin name of the device to send admin commands
    def adm_name(self):
        return self._admin_name

    # Get device alias if defined
    def alias(self):
        return self._alias

    # Query the device for info on all attributes
    def attribute_list_query(self):
        url = self.ip_addr + '/' + self.dev_name + '/attr_list'
        req = requests.get(url)
        return req.json()

    def bind_functions(self):
        functions_dict = self.get_function_list()
        functions = functions_dict['functions']

        for name in functions:
            self.add_fn(name)

    # Returns n previous commands
    def black_box(self, n: int) -> List[Dict]: 
        url = self.ip_addr + '/' + self.dev_name + '/black_box?n=' + str(n)
        req = requests.get(url)
        return req.json()

    # Retreive command history from the command polling buffer
    def command_history(self):
        url = self.ip_addr + '/' + self.dev_name + '/command_history'
        req = requests.get(url)
        return req.json()

    # Query the device for information in all commands
    def command_list_query(self):
        raise NotImplementedError()

    # Query the device for information about a single command
    def command_query(self, command):
        url = self.ip_addr + '/' + self.dev_name + '/command_query?name=' + str(command)
        req = requests.get(url)
        return req.json()

    # Delete a given property of this device
    def delete_property(self, value):
        raise NotImplementedError()

    # Get device description
    def description(self):
        return self._description

    # Return the attribute configuration for a single attribute
    def get_attribute_config(self, names):
        raise NotImplementedError()

    # Getting the attributes of the device server
    def get_attribute_list(self) -> List[str]:
        uri = self.ip_addr + '/' + \
                self.dev_name + '/attributes'
        req = requests.get(uri)
        return req.json()

    # Return the command configuration for all commands
    def get_command_config(self):
        raise NotImplementedError()

    # Return the names of all commands implemented for this device
    def get_command_list(self):
        raise NotImplementedError

    # Returns the internal database reference
    def get_device_db(self):
        raise NotImplementedError()

    # Getting the functions of the device server
    def get_function_list(self) -> List[str]:
        uri = self.ip_addr + '/' + self.dev_name + '/functions'
        req = requests.get(uri)
        return req.json()

    # Get a (list) of property(ies) for a device
    def get_property(self, propname, value=None):
        raise NotImplementedError()

    # Get the list of property names for this device
    def get_property_list(self, filter, array=None):
        raise NotImplementedError()

    # Method which returns information on the device
    def info(self) -> Dict:
        url = self.ip_addr + '/' + self.dev_name + '/information'
        req = requests.get(url)
        return req.json()

    # Return the device name from the device itself
    def name(self):
        return self.dev_name

    # A method which sends a ping to the device
    def ping(self):
        raise NotImplementedError()

    # Read a single attribute
    def read_attribute(self, attr_name: str) -> Dict:
        url = self.ip_addr + '/' + self.dev_name + '/read_attr?name=' + attr_name
        req = requests.get(url)
        return req.json()

    # Read a single attribute async
    def read_attribute_asynch(self, attr_name):
        raise NotImplementedError()

    def read_attribute_reply():
        raise NotImplementedError()

    # Read the list of specified attributes
    def read_attributes(self, attr_names: list) -> Dict:
        url = self.ip_addr + '/' + self.dev_name + '/read_attrs?list=' + str(attr_names).replace(' ', '')
        req = requests.get(url)
        return req.json()

    # Set attribute configuration for the specified attribute
    def set_attribute_config(self, attr_info):
        raise NotImplementedError()

    # Returns the state of the device
    def state(self):
        raise NotImplementedError()

    # Returns the status of the device as a string
    def status(self) -> Dict:
        url = self.ip_addr + '/' + self.dev_name + '/status'
        _status = requests.get(url)
        return _status.json()

    # Write a single attribute
    def write_attribute(self, attr_name: str, value: str) -> Dict:
        url = self.ip_addr + '/' + self.dev_name + '/write_attr'
        req = requests.post(url, data={attr_name: value})
        return req.json()

    # Write a single attribute async
    def write_attribute_asynch(attr_name, value, cb=None):
        raise NotImplementedError()

    # Write the specified attributes
    def write_attributes(self, name_val_dict: Dict) -> Dict:
        url = self.ip_addr + '/' + self.dev_name + '/write_attrs'
        req = requests.post(url, data=name_val_dict)
        return req.json()

    # Write then read a single attribute in a single network call.
    def write_read_attribute(self, write_attr_name: str, read_attr_name: str, value: str) -> Dict:
        self.write_attribute(write_attr_name, value)
        return self.read_attribute(read_attr_name)

    # Write then read attribute(s) in a single network call
    def write_read_attributes(self, write_attr_dict, read_attr_names):
        self.write_attributes(write_attr_dict)
        return self.read_attributes(read_attr_names)
