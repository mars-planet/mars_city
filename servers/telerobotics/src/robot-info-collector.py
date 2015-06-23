#!/usr/bin/env python
from __future__ import print_function

__author__ = "Siddhant Shrivastava"
__email__ = "sidhu94@gmail.com"
__status__ = "Development"
__copyright__ = "Italian Mars Society"

"""
A ROS node which collects all the information from the robot and applies the appropriate message filters. The node is responsible for sending the information to the Tango aggregator, which would be worried about sending information on the bus.
"""

from collections import namedtuple
import rospy, cv2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan, Imu
rom std_msgs.msg import String
from husky_msgs.msg import HuskyStatus
from threading import Thread
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from husky_msgs.msg import HuskyStatus

# Configuration variable for the ROS node which collects all robot information
NODE_NAME = "RobotCollector"

# Configuration variables for all topics to be subscribed to
IMU_TOPIC = "/imu/data"
ODOM_TOPIC = "/husky_velocity_controller/odom"
LEFT_CAM_INFO_TOPIC = "/left_camera/camera_info"
RIGHT_CAM_INFO_TOPIC = "/right_camera/camera_info"
LASER_SCAN_TOPIC ="/scan"
LEFT_CAM_IMAGE_TOPIC= "/left_camera/image_raw/compressed"
RIGHT_CAM_IMAGE_TOPIC = "/right_camera/image_raw/compressed"
POSE_TOPIC = "/initial_pose"
HUSKY_STATUS_TOPIC = "status"


# Get Data Structures for different Husky parameters
twist = Twist()
pose = Pose()
odom = Odometry()
laserData = LaserScan()
imuData = Imu()

def positionCallback(receivedPose):
    global pose
    pose = receivedPose

def laserCallback(receivedLaser):
    global laserData
    laserData = receivedLaser

def odometryCallback(receivedOdom):
    global odom
    odom = receivedOdom

def imuCallback(receivedIMU):
    global imuData
    imuData = receivedIMU

# Camera basic information callbacks
def rightCameraInfoCallback(receiveRightCamera):
    global rightCamInfo
    rightCamInfo = receiveRightCamera

def leftCameraInfoCallback(receiveLeftCamera):
    global leftCamInfo
    leftCamInfo = receiveLeftCamera

def rightCameraImageCallback(rightFrame):
    np_arr = np.fromstring(rightFrame.data, np.uint8)
    rightFrame = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

def leftCameraImageCallback(leftFrame):
    np_arr = np.fromstring(leftFrame.data, np.uint8)
    leftFrame = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

def statusCallback(receivedStatus):
    global huskyStatus
    huskyStatus = receivedStatus

def spin():
    rospy.spin()


if __name__ == '__main__':
    # Initialize client node
    rospy.init_node(NODE_NAME)

    # Subscribing to all these topics
    rospy.Subscriber(POSE_TOPIC, Pose, positionCallback)
    rospy.Subscriber(ODOM_TOPIC, PoseWithCovarianceStamped, odometryCallback)    rospy.Subscriber(IMU_TOPIC, Imu, imuCallback)
    rospy.Subscriber(LASER_SCAN_TOPIC, LaserScan, laserCallback)
    rospy.Subscriber(RIGHT_CAM_INFO_TOPIC, CameraInfo, rightCameraInfoCallback)
    rospy.Subscriber(LEFT_CAM_INFO_TOPIC, CameraInfo, leftCameraInfoCallback)
    rospy.Subscriber(RIGHT_CAM_IMAGE_TOPIC, CompressedImage, rightCameraImageCallback)
    rospy.Subscriber(LEFT_CAM_IMAGE_TOPIC, CompressedImage, leftCameraImageCallback)
    rospy.Subscriber(HUSKY_STATUS_TOPIC, HuskyStatus, statusCallback)

    # For future Arm Data collection
    transListener = tf.TransformListener()

    Thread(target = spin).start()
