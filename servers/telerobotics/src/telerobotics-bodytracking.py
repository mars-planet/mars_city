#!/usr/bin/env python
from __future__ import print_function

__author__ = "Siddhant Shrivastava"
__email__ = "sidhu94@gmail.com"
__status__ = "Development"
__copyright__ = "Italian Mars Society"

"""
Implementation of the Telerobotics and Bodytracking Interface.
The events are subscribed to the Bodytracking moves and the received data
is processed and converted to Linear and Angular velocity which the UGV
can understand.
"""

import PyTango
import time
import datetime
from collections import deque
import rospy
from geometry_msgs.msg import Twist
"""
Plotting to be implemented
"""
# import matplotlib.pyplot as plt

# Get current time
TIME_START = time.time()

# Configure Bodytracking Tango Device name
DEVICE_NAME = "c3/mac/eras-1"

# Log file to keep track of events
MOVES_FILE = "moves_file.log"
moves_file = open("MOVES_FILE", 'w')

# Refresh Rate for Kinect is 30 fps
REFRESH_RATE = 1.0 / 30.0

# Create a Device Proxy for the Bodytracking device
device_proxy = PyTango.DeviceProxy(DEVICE_NAME)

# Configuration Variables for the Robot
linear_speed, angular_speed = 0, 0

#Plotting commands - TBD
# plt.axis([0,20,-0.1,0.1])
# plt.ion()
# plt.show()

# Event containers for time, position, and orientation
time_events = deque()
position_events = deque()
orientation_events = deque()

#ROS Husky topic name for velocity
HUSKY_TOPIC = '/husky_velocity_controller/cmd_vel'
ROS_NODE_NAME = 'Telerobot'
HUSKY_QUEUE_SIZE = 1000

# Trigger to enable/disable the interface
TRIGGER = True

"""
Callback class attributes handle all communication and processing
"""

class Callback:

    """
    Command takes the velocity components as inputs and publishes them
    to ROS. It handles message passing and manages the Robot Velocity
    states.
    """

    def command(self, linear_velocity, angular_velocity):

        # Create  a publisher Node to the HUSKY_TOPIC
        pub = rospy.Publisher(HUSKY_TOPIC, Twist, queue_size = HUSKY_QUEUE_SIZE)

        # Construct a twist message, and set up the linear and angular
        # velocity arguments
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        # Announce the move
        rospy.loginfo("In Motion with {} m/s and {} rad/s".format
                            (linear_velocity, angular_velocity))

         # Publish the message to ROS
        pub.publish(twist)
        rospy.sleep(REFRESH_RATE)

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

                # Communication with ROS command() module
                if(linear_speed != 0 and angular_speed != 0):
                    self.command(linear_speed, angular_speed)

            # Log the Event data
            moves_file.write(str(position) + "\t" + str(orientation) + "\n")

        # Tango Error Handling - Exceptions TBD
        else:
            print(event.errors)
            moves_file.write(str(event.errors))


if __name__ == "__main__":

    # Create the callback object
    cb = Callback()

    #ROS Initialization
    try:
        # Create and Initialize a ROS node!
        rospy.init_node(ROS_NODE_NAME)
    except rospy.ROSInitException as init_exception:
        print("roscore is not initiated yet. Message", init_exception)

    while TRIGGER:
        try:
            # Subscribe to the 'moves' event from the Bodytracking interface
            moves_event = device_proxy.subscribe_event(
                                                                        'moves',
                                                                        PyTango.
                                                                        EventType.
                                                                        CHANGE_EVENT,
                                                                        cb,
                                                                         []
                                                                         )
        except PyTango.DevError as err:
            PyTango.Except.print_exception(err)


        # Wait for at least REFRESH_RATE Seconds for the next callback.
        time.sleep(REFRESH_RATE)
