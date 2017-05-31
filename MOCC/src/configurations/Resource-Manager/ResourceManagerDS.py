"""
ResourceManager tango device server
"""
from models import Resources

from PyTango.server import Device, DeviceMeta, command, server_run


class ResourceManager(Device):
    __metaclass__ = DeviceMeta

    def init_device(self):
        Device.init_device(self)

    @command(dtype_in=str, dtype_out=str)
    def ask_resource(self, name):
        print "Hello!"
        # resource_string = get_resource_string(name)
        resource = Resources.get(Resources.name == name)
        resource_string = "" + resource.name + " # " + resource.type + " # " + str(resource.availability_start) + " # " + str(resource.availability_end) + " # " + str(resource.rate) + " # " + str(resource.amount)
        # Sending the resource as a string
        return str(resource_string)


if __name__ == "__main__":
    server_run([ResourceManager])

# Remove for now to pass flake8 test. To be used later:
# from PyTango import AttrQuality, AttrWriteType, DispLevel, DevState, DebugIt
# from PyTango.server import device_property
