import PyTango

bm = PyTango.DeviceProxy("C3/biometric_monitor/1")

try:
	bm.start_monitoring()
except:
	pass