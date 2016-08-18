from PyTango import Database, DbDevInfo

#  A reference on the DataBase
db = Database()


new_device_name1 = "C3/unity/eras1"


# Define the Tango Class served by this  DServer
new_device_info_mouse = DbDevInfo()
new_device_info_mouse._class = "PyTracker"
new_device_info_mouse.server = "tango_pygame/eras1" #servername/instance

# add the first device
print("Creating device: %s" % new_device_name1)
new_device_info_mouse.name = new_device_name1
db.add_device(new_device_info_mouse)

