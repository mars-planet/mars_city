import PyTango

'''Script for creating the device server for Resource Manager'''

dev_info = PyTango.DbDevInfo()

dev_info.server = "ResourceManagerDS/test"
dev_info._class = "ResourceManager"
dev_info.name = "test/resource_manager/1"

db = PyTango.Database()
db.add_device(dev_info)
