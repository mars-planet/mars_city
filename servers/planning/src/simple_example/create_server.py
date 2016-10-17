import PyTango

dev_info = PyTango.DbDevInfo()
dev_info.server = "Publisher/test"
dev_info._class = "Publish"
dev_info.name = "test/publisher/1"

db = PyTango.Database()
db.add_device(dev_info)
