import PyTango

dev_info = PyTango.DbDevInfo()
dev_info.server = "Monitor/monitor"
dev_info._class = "Monitor"
dev_info.name = "C3/biometric_monitor/1"

print("Creating device: %s" % dev_info.name)
db = PyTango.Database()
db.add_device(dev_info)