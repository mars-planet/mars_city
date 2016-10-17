#!/usr/bin/env python
from __future__ import print_function

__author__ = "Siddhant Shrivastava"
__email__ = "sidhu94@gmail.com"
__status__ = "Development"
__copyright__ = "Italian Mars Society"

"""
Implementation of Keyboard Teleoperation for Husky. Keyboard is an alternative
way of controlling the Husky movement in case Bodytracking Interface
This implementation makes use of POSIX System Calls to get keyboard input and
maps it to appropriate linear and angular velocity commands which are then
published to the Velocity Controller Topic for Husky movements.
"""

# rospy and geometry_msgs.msg responsible for ROS Interfacing with Husky
import rospy
from geometry_msgs.msg import Twist

# sys, select, termios, tty are used for System-level control of Keyboard Input
import sys
import select
import termios
import tty

DISPLAY_MESSAGE = """
Teleoperate Husky using Keyboard
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

# Configuration Variables for Husky parameters

# Set queue size for the Husky Publisher message
HUSKY_QUEUE_SIZE = 100

# Set Husky Velocity Topic name
HUSKY_VELOCITY_TOPIC = "cmd_vel"

# Set the Node name for this Teleoperation node
ROS_NODE_NAME = "teleoperate_keyboard_husky"

# Speed and Turn Configuration Variables
speed = .5
turn = 1

# Dictionary to map keyboard input characters to movements in 2-D Space
# Format (North Value, West Value)
moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

# Dictionary to map keyboard characters to Speed Change Values expressed
# as percentage changes in the corresponding moveBindings value
speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

"""
getKey returns the current input key on the keyboard.
It makes use of POSIX System calls to set file descriptors, wait for I/O
and terminal control
"""

def getKey():

    # Set the integer file descriptor to a Raw descriptor
    tty.setraw(sys.stdin.fileno())

    # Poll for I/O from the standard input or the corresponding descriptor
    select.select([sys.stdin], [], [], 0)

    # Read key value from the polled file descriptor EPOLLed by select
    key = sys.stdin.read(1)

    # Control tty to drain all queued input after obtaining key value
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

"""

"""

def get_velocity(speed, turn):
    velocity_message = "currently:\tspeed {}\tturn {} " .format(speed, turn)
    return velocity_message

if __name__ == "__main__":

    # Get control settings for standard input of the terminal
    settings = termios.tcgetattr(sys.stdin)

    # Create a publisher to the Velocity Controller topic
    pub = rospy.Publisher(HUSKY_VELOCITY_TOPIC, Twist,
                                         queue_size = HUSKY_QUEUE_SIZE)

    try:
        # Create and Initialize a ROS node!
        rospy.init_node(ROS_NODE_NAME)

    # If roscore is not initialized
    except rospy.ROSInitException as init_exception:
        print("roscore is not initiated yet. Message", init_exception)

    linear = 0
    angular = 0
    status = 0

    try:

        #Display prompt messages
        print(DISPLAY_MESSAGE)
        print(get_velocity(speed, turn))

        #Infinite loop for Teleoperation
        while(1):
            key = getKey()

            # Get linear and angular speed value from Move Bindings
            if key in moveBindings.keys():
                linear = moveBindings[key][0]
                angular = moveBindings[key][1]

            # If input key is in the velocity bindings, set Speed and turn
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(get_velocity(speed, turn))

                #Keep velocity in check
                if (status == 14):
                    print(DISPLAY_MESSAGE)
                status = (status + 1) % 15

            # If key is not found in either of the bindings, do nothing
            else:
                linear = 0
                angular = 0

                # If Ctrl+C break
                if (key == '\x03'):
                    break

            #Construct the Twist message
            twist = Twist()
            twist.linear.x = linear * speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = angular * turn
            pub.publish(twist)

    # In case of any errors
    except:
        print("Error")

    # Reset the Teleoperation configuration and bring robot to a halt
    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        # Drain and clear all the standard input from the terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
