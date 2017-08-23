"""
ResourceManager tango device server
"""
from models import Resources
from string_manager import StringManager
from dateutil import parser
from PyTango.server import Device, DeviceMeta, command, server_run


class ResourceManager(Device):
    __metaclass__ = DeviceMeta

    def init_device(self):
        Device.init_device(self)

    @command(dtype_in=str, dtype_out=str)
    def ask_resource(self, name):
        """
        returns data related to tuple with name name
        """
        print "request for ", name, " received"
        # resource_string = get_resource_string(name)
        resource = Resources.get(Resources.name == name)

        resource_string = "" + resource.name + " # " + resource.type
        resource_string += " # " + str(resource.availability_start)
        resource_string += " # " + str(resource.availability_end)
        resource_string += " # " + str(resource.rate)
        resource_string += " # " + str(resource.amount)

        # Sending the resource as a string
        return str(resource_string)

    @command(dtype_in=str, dtype_out=str)
    def set_updated_resource_values(self, resource_object_string):
        """
        updates the tuple with name = resource_object_string.name
        """
        # input strings are expected to have columns seperated with a '#'
        strManager = StringManager()
        resource_string_object = strManager.str_to_object(
            resource_object_string)
        resource = Resources.get(Resources.name == resource_string_object.name)

        # Updating tuple
        resource.rate = resource_string_object.rate
        resource.amount = resource_string_object.amount
        resource.availability_start = parser.parse(
            resource_string_object.availability_start)
        resource.availability_end = parser.parse(
            resource_string_object.availability_end)

        if(resource.save()):
            print resource_string_object.name, " tuple successfully updated"
            return "SUCCESS"
        else:
            print "error while updating tuple ", resource_string_object.name
            return "FAILURE"


if __name__ == "__main__":
    server_run([ResourceManager])

# Removed for now to pass flake8 test. To be used later:
# from PyTango import AttrQuality, AttrWriteType, DispLevel, DevState, DebugIt
# from PyTango.server import device_property
