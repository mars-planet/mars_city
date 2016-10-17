import PyTango
pub = PyTango.DeviceProxy("test/publisher/1")
pub.set_coord()
print tuple(pub.coordinates)
