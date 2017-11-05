import requests


class DeviceProxy:
    def __init__(self, dev_name):
        self._name = dev_name 
        self.bind_functions()

    def test_connection(self):
        return 'SUCCESS'

    def add_fn(self, name):
        def fn():
            # Make appropriate REST call here
            uri = 'http://127.0.0.1:5000' + self._name
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
        return None

    def bind_functions(self):
        functions_dict = self.get_function_list()
        functions = functions_dict['functions']

        for name in functions:
            self.add_fn(name)

    # Returns n previous commands
    def black_box(self, n):
        return None

    # Retreive command history from the comman polling buffer
    def command_history(self):
        return None

    # Query the device for information in all commands
    def command_list_query(self):
        return None

    # Query the device for information about a single command
    def command_query(self, command):
        return None

    # Delete a given property of this device
    def delete_property(self, value):
        return None

    # Get device description
    def description(self):
        return self._description

    # Return the attribute configuration for a single attribute
    def get_attribute_config(self, names):
        return None

    # Getting the attributes of the device server
    def get_attribute_list(self):
        uri = 'http://127.0.0.1:5000' + self.resource_path_URI + '/attributes'
        req = requests.get(uri)
        return req.json()

    # Return the command configuration for all commands
    def get_command_config(self):
        return None

    # Return the names of all commands implemented for this device
    def get_command_list(self):
        return None

    # Returns the internal database reference
    def get_device_db(self):
        return None

    # Getting the functions of the device server
    def get_function_list(self):
        uri = 'http://127.0.0.1:5000' + self.resource_path_URI + '/functions'
        req = requests.get(uri)
        return req.json()

    # Get a (list) of property(ies) for a device
    def get_property(self, propname, value=None):
        return None

    # Get the list of property names for this device
    def get_property_list(self, filter, array=None):
        return None

    # Method which returns information on the device
    def info(self):
        return self._info

    # Return the device name from the device itself
    def name(self):
        return self._name

    # A method which sends a ping to the device
    def ping(self):
        return None

    # Read a single attribute
    def read_attribute(slef, attr_name):
        return None

    # Read a single attribute async
    def read_attribute_asynch(self, attr_name):
        return None

    def read_attribute_reply():
        return None

    # Read the list of specified attributes
    def read_attributes(self, attr_names):
        return None

    # Set attribute configuration for the specified attribute
    def set_attribute_config(self, attr_info):
        return None

    # Returns the state of the device
    def state(self):
        return None

    # Returns the status of the device as a string
    def status(self):
        return None

    # Write a single attribute
    def write_attribute(self, attr_name, value):
        return None

    # Write a single attribute async
    def write_attribute_asynch(attr_name, value, cb=None):
        return None

    # Write the specified attributes
    def write_attributes(self, name_val):
        return None

    # Write then read a single attribute in a single network call.
    def write_read_attribute(self, attr_name, value):
        return None

    # Write then read attribute(s) in a single network call
    def write_read_attributes(self, name_val, attr_names):
        return None
