"""A test script to run a Husky UGV for MOVE_TIME * SLEEP_RATE seconds"""

import roslib
import rospy
from geometry_msgs.msg import Twist
import subprocess

LINEAR_VELOCITY = 3  # Configuration Variable for speed along the x direction  - 3 m/s
ANGULAR_VELOCITY = 2  # Configuration Variable for angular velocity for z-axis
MOVE_TIME = 40  # Husky runs for (MOVE_TIME / 10) seconds
SLEEP_RATE = 0.1
HUSKY_TOPIC = '/husky_velocity_controller/cmd_vel'
TIMEOUT = 5

def command():

    topics = rospy.get_published_topics()

    if HUSKY_TOPIC not in topics:
        # launch the relevant ros topics
        subprocess.Popen(["roslaunch", "husky_control", "control.launch"])
        # publish to the 'cmd_vel' ROS topic
        p = rospy.Publisher(HUSKY_TOPIC, Twist, queue_size=100)
        rospy.sleep(TIMEOUT)

    # create a twist message, and set up the linear and angular
    # velocity arguments
    twist = Twist()
    twist.linear.x = LINEAR_VELOCITY
    twist.angular.z = ANGULAR_VELOCITY

    # announce move, and publish the message
    rospy.loginfo("In Motion!")

    # Publish Rate - Husky moves for MOVE_TIME * SLEEP_RATE seconds
    for i in range(MOVE_TIME):
        p.publish(twist)
        rospy.sleep(SLEEP_RATE)

    # create a new message
    twist = Twist()
    rospy.loginfo("Stopping now!")
    p.publish(twist)

if __name__ == '__main__':
    try:
        # Create and Initialize a ROS node!
        rospy.init_node('commander')
    except rospy.ROSInitException as init_exception:
        print("roscore is not initiated yet. Message: %s") % init_exception
        roscore = subprocess.Popen(['roscore'])

    try:
        command()
    except rospy.ROSInterruptException as interrupt:
        print("ROS process was interrupted")
