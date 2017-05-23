"""resource manager tango device server"""

import time
import numpy
import MySQLdb
import datetime
import PyTango
import logging

from PyTango import AttrQuality, AttrWriteType, DispLevel, DevState, DebugIt
from PyTango.server import Device, DeviceMeta, attribute, command, server_run
from PyTango.server import device_property

# class defined to make Resource objects
class Resource:
    def __init__(self, resource_name, resource_type, availability_start, availability_end, amount, rate):
        self.resource_name = resource_name
        self.resource_type = resource_type
        self.availability_end = availability_end
        self.availability_start = availability_start
        self.amount = amount
        self.rate = rate

# Function used later for asking for a resource
def ask_resource(cursor, resource_name):
    cursor.execute("SELECT * FROM Resources WHERE resource_name = %s", (resource_name))
    return cursor.fetchone()

class ResourceManager(Device):
    __metaclass__ = DeviceMeta

    def init_device(self):
        Device.init_device(self)

    @command(dtype_in=str, dtype_out=str)
    def ask_resource(self, resource_name):
        db = MySQLdb.connect("localhost","root","","TESTDB" )
        cursor = db.cursor()

        resource = ask_resource(cursor, resource_name)
        resource_string = "" + resource[0] +" "+ resource[1] +" "+ str(resource[2]) +" "+ str(resource[3]) +" "+ str(resource[4]) +" "+ str(resource[5])

        return resource_string

if __name__ == "__main__":
    server_run([ResourceManager])
    
    
'''
Database methods to be updated later

def give_resource(cursor, resource_name):
    cursor.execute("SELECT * FROM Resources WHERE resource_name = %s", (resource_name))
    res_tuple =  cursor.fetchone()
    resource = Resource(res_tuple[0], res_tuple[1], res_tuple[2], res_tuple[3], res_tuple[4], res_tuple[5])
    return resource

def set_resource_update(cursor, resource_name, new_amt, new_avail_end):
    cursor.execute("UPDATE Resources set amount = %s, availability_end = %s WHERE resource_name = %s", (new_amt, new_avail_end, resource_name))
    '''