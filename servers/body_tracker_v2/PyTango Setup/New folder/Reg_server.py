import PyTango

dev_info = PyTango.DbDevInfo()
dev_info.server = "time/test"
dev_info._class = "clock"
dev_info.name = "test/timing/1"

db = PyTango.Database()
db.add_device(dev_info)
