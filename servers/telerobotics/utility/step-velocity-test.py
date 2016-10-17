#!/usr/bin/env python
from __future__ import print_function

__author__ = "Siddhant Shrivastava"
__email__ = "sidhu94@gmail.com"
__status__ = "Development"
__copyright__ = "Italian Mars Society"

"""
ROS-Independent Test Implementation of the Telerobotics and Bodytracking Interface.
The events are subscribed to the Bodytracking moves and the received data
is processed and converted to Linear and Angular velocity which the UGV
can understand.
"""

import PyTango
import time
import datetime
from collections import deque
"""
Plotting to be implemented
"""
# import matplotlib.pyplot as plt

# Get currentent time
TIME_START = time.time()

# Configure Bodytracking Tango Device name
DEVICE_NAME = "c3/mac/eras-1"

# Log file to keep track of events
MOVES_FILE = "moves_file.log"
moves_file = open("MOVES_FILE", 'w')

# Refresh Rate for Kinect is 30 fps
REFRESH_RATE = 1/30

# Create a Device Proxy for the Bodytracking device
device_proxy = PyTango.DeviceProxy(DEVICE_NAME)

# Configuration Variables for the Robot
linear_speed, angular_speed = 0, 0

# Event containers for time, position, and orientation
time_events = deque()
position_events = deque()
orientation_events = deque()

#ROS Husky topic name for velocity
HUSKY_TOPIC = '/husky_velocity_controller/cmd_vel'
ROS_NODE_NAME = 'Telerobot'

# Trigger to enable/disable the interface
TRIGGER = True

"""
Callback class attributes handle all communication and processing
"""

class Callback:

    """
    push_event(event) is the callback method for the Bodytracking
    Event Subcriber. It is triggered when an event change is
    encountered. The method handles the input events and
    processes the velocity components. It makes use of High-performance
    containers type called deque to buffer the events and compute
    delta between the event attributes - position, orientation, timestamps

    It processes the inputs into a ROS Robot-friendly form. Logging is
    also supported.
    """

    def push_event(self,event):

        if not event.err:
            # Extract Position and Orientation values from the event attribute
            [position, orientation] =  event.attr_value.value

            # Get Time Data about the triggered event
            event_time_val = event.get_date()

            # Keep track of number of events in the deque
            num_events = len(time_events)

            # If 0 events then it's the first instance of the event pair
            if(num_events is 0):
                time_events.append(event_time_val.todatetime())
                position_events.append(position)
                orientation_events.append(orientation)

            # One event ensures we have what we need to compute deltas
            elif(num_events is 1):
                # For Python 3 Time conversion
                # timestamp = (event_time_data - datetime.datetime(1970, 1, 1))
                #                       / datetime.timedelta(seconds=1)

                # Pop previous event from deque
                time_previous= time_events.pop()

                # Convert PyTango's TimeVal data type to a datetime object
                time_current =  event_time_val.todatetime()

                # Compute Time Delta
                time_difference= time_current -time_previous
                time_delta = time_difference.total_seconds()

                # Position and Linear Velocity Processing
                position_previous = position_events.pop()
                position_current = position
                linear_displacement = position_current - position_previous
                linear_speed = linear_displacement / time_delta

                # Orientation and Angular Velocity Processing
                orientation_previous= orientation_events.pop()
                orientation_current = orientation
                angular_displacement = orientation_current - orientation_previous
                angular_speed = angular_displacement / time_delta

                if(linear_speed !=0 and angular_speed != 0):
                    print("{} :  Linear Speed= {} , Angular Speed= {} ".format(time_current, linear_speed, angular_speed))
            # Log the Event data
            moves_file.write(str(position) + "\t" + str(orientation) + "\n")

        # Tango Error Handling - Exceptions TBD
        else:
            print(event.errors)
            moves_file.write(str(event.errors))


if __name__ == "__main__":

    # Create the callback object
    cb = Callback()

    while TRIGGER:
        # Subscribe to the 'moves' event from the Bodytracking interface
        moves_event = device_proxy.subscribe_event(
                                                   'moves',
                                                    PyTango.EventType.CHANGE_EVENT,
                                                    cb, [])
        # Wait for at least REFRESH_RATE Seconds for the next callback.
        time.sleep(REFRESH_RATE)
