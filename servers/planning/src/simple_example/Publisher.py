import time
import numpy
import random
import json
import os
import re
from PyTango import *
from PyTango import AttrQuality, AttrWriteType, DispLevel, DevState, DebugIt
from PyTango.server import Device, DeviceMeta, attribute, command,run
from PyTango.server import device_property

coordinates={"rock1":(1,1),"rock2":(3,3),"rock3":(2,2),"rock4":(9,9)}
# Connect to planner device via PyTango
planner= DeviceProxy("C3/pso/1")

def get_actions():
    """
    get actions and objects from plan log.
    """
    return planner.get_actions()

def get_objects():
    return planner.get_objects()


actions = json.loads(get_actions())
print "EUROPA PLAN:",actions["Rover"],"\n"


def cal_co_ord(actions):
    for n, action in enumerate(actions["Rover"]):
        event = action["events"][0]
        print "PLAN EVENT CALLED:",event
        if ".Go" in event:
            destination=re.search("dest={(.*)[/(]", event).group(1)
            #print destination
            return(coordinates.get("{0}".format(destination)))


co_ord= cal_co_ord(actions)


class Publish(Device):
    __metaclass__ = DeviceMeta
    POLLING = 30

    coordinates = attribute(label = "Destination co-ordinates",
                          dtype = (int,),
                          unit = "(meters, meters)",
                          access=AttrWriteType.READ,
                          polling_period = POLLING,
                          fget="get_coordinates",
                          fset="set_coordinates",
                          max_dim_x = 100,
                          max_dim_y = 100,
                          doc="An attribute for Linear and angular \
                               displacements")


    def init_device(self):
        Device.init_device(self)
        self.__coordinates = (0,0)
        self.set_state(DevState.STANDBY)

    @command
    def set_coord(self):
        self.__coordinates = tuple(co_ord)
        self.push_change_event('coordinates', tuple(self.__coordinates), 2)


    def get_coordinates(self):
        return self.__coordinates

    def set_coordinates(self, co_ordinates):
        # should set the power supply coordinates
        self.__coordinates = co_ordinates


    @command
    def TurnOn(self):
        # turn on the actual power supply here
        self.set_state(DevState.ON)

    @command
    def TurnOff(self):
        # turn off the actual power supply here
        self.set_state(DevState.OFF)




if __name__ == "__main__":
    run([Publish])
