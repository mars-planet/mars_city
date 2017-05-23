import PyTango

device = PyTango.DeviceProxy("test/resource_manager/1")

print device.ping()," device pinged\n"

resource_data = device.ask_resource("Battery")

print "Resource data received is :\n",resource_data