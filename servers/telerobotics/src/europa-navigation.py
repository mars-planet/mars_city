#!/usr/bin/env python
from __future__ import print_function

__author__ = "Siddhant Shrivastava"
__email__ = "sidhu94@gmail.com"
__status__ = "Development"
__copyright__ = "Italian Mars Society"

"""
Implementation of the Telerobotics and EUROPA Planner Interface.
The events are subscribed to the Planner Coordinates Event and the received data
is processed and converted to Linear and Angular velocity which the Husky UGV
can understand.

NB: The ROS machine must have the following processes in three different
       terminalsrunning prior to this -

roslaunch husky_gazebo husky_empty_world.launch
roslaunch husky_viz view_robot.launch
roslaunch husky_navigation amcl_demo.launch
"""

import PyTango
import time
import datetime
from collections import deque
import rospy
from geometry_msgs.msg import Twist, Quaternion, Point
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

# Get current time
TIME_START = time.time()

# Configure Bodytracking Tango Device name
DEVICE_NAME = "c3/europa/planner"

# Log file to keep track of events
NAV_FILE = "navigation_file.log"
navigation_file = open("NAV_FILE", 'w')

# Refresh Rate for Planner is 30 times per second
REFRESH_RATE = 1.0 / 30.0

# Create a Device Proxy for the Bodytracking device
device_proxy = PyTango.DeviceProxy(DEVICE_NAME)

# Configuration Variables for the Robot
linear_speed, angular_speed = 0, 0

# Event containers for time, x_coord, and y_coord
time_events = deque()
x_coord_events = deque()
y_coord_events = deque()

#ROS Husky topic name for velocity
HUSKY_BASE_TOPIC = '/move_base'

# ROS Node name
ROS_NODE_NAME = 'telerobot-europa-navigate'
HUSKY_QUEUE_SIZE = 1000
WAITING_PERIOD = 5
HARD_DEADLINE = 60

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

    def command(self, x, y):

        # Create a publisher Node to the HUSKY_TOPIC
        pub = rospy.Publisher(HUSKY_TOPIC, Twist, queue_size = HUSKY_QUEUE_SIZE)

        # Set up an Action Client to the move_base topic
        actions = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for Action Server!")

        move_base.wait_for_server(rospy.Duration(5))

        # Establish the Goal Attributes
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set pose to the x,y attributes received from EUROPA Planner
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(0.000, 0.000, 0, 0))
        move_base.send_goal(goal)

        success = move_base.wait_for_result(rospy.Duration(HARD_DEADLINE))

        if not success:
            move_base.cancel_goal()
            rospy.loginfo("Husky didn't reach the desired pose")

        else:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Husky reached the desired pose")

        rospy.sleep(REFRESH_RATE)

    """
    push_event(event) is the callback method for the Bodytracking
    Event Subcriber. It is triggered when an event change is
    encountered. The method handles the input events and
    processes the velocity components. It makes use of High-performance
    containers type called deque to buffer the events and compute
    delta between the event attributes - x_coord, y_coord, timestamps

    It processes the inputs into a ROS Robot-friendly form. Logging is
    also supported.
    """

    def push_event(self,event):

        if not event.err:
            # Extract x_coord and y_coord values from the event attribute
            [x_coord, y_coord] =  event.attr_value.value

            # Get Time Data about the triggered event
            event_time_val = event.get_date()

            # Keep track of number of events in the deque
            num_events = len(time_events)

            # If 0 events then it's the first instance of the event pair
            if(num_events is 0):
                time_events.append(event_time_val.todatetime())
                x_coord_events.append(x_coord)
                y_coord_events.append(y_coord)

            # One event ensures we have what we need to compute deltas
            elif(num_events is 1):
                # For Python 3 Time conversion
                # timestamp = (event_time_data - datetime.datetime(1970, 1, 1))
                #                       / datetime.timedelta(seconds=1)

                # Pop previous event from deque
                time_previous= time_events.pop()

                # Convert PyTango's TimeVal data type to a datetime object
                time_current =  event_time_val.todatetime()

                # Communication with ROS command() module
                if(linear_speed != 0 and angular_speed != 0):
                    self.command(x_coord, y_coord)

            # Log the Event data
            navigation_file.write(str(time_current) + "\t" + str(x_coord) + "\t" + str(y_coord) + "\n")

        # Tango Error Handling
        else:
            print(event.errors)
            navigation_file.write(str(event.errors))


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
                                                                        'navigation',
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
