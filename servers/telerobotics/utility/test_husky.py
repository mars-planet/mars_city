"""A test script to run a Husky UGV for 3 seconds"""

import roslib
import rospy
from geometry_msgs.msg import Twist

LINEAR_VELOCITY = 3  # Configuration Variable for speed along the x direction  - 3 m/s
ANGULAR_VELOCITY = 2 # Configuration Variable for angular velocity for z-axis
# first thing, init a node!
rospy.init_node('commander')

def command():

	# publish to the 'cmd_vel' ROS topic
	p = rospy.Publisher('husky_velocity_controller/cmd_vel', Twist, queue_size = 100)

	# create a twist message, and set up the linear and angular
	# velocity arguments
	twist = Twist()
	twist.linear.x = LINEAR_VELOCITY
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0;
	twist.angular.z = ANGULAR_VELOCITY;

	# announce move, and publish the message
	rospy.loginfo("In Motion!")

	#publish rate
	for i in range(90):
	    p.publish(twist)
	    rospy.sleep(0.1) # 30*0.1 = 3.0 seconds

	# create a new message
	twist = Twist()

	# note: everything defaults to 0 in twist, if we don't fill it in, we stop!
	rospy.loginfo("Stopping now!")
	p.publish(twist)

if __name__ == '__main__':
	try:
	    command()
	except rospy.ROSInterruptException:
	    pass
