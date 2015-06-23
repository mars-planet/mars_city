#!/usr/bin/env python

__author__ = "Siddhant Shrivastava"
__email__ = "sidhu94@gmail.com"
__status__ = "Development"
__copyright__ = "Italian Mars Society"

"""
Robot diagnostics feedback aggregator Tango Device server
The server subscribes to the ROS topics which publishes all robot diagnostic
 information. The data aggregated from the ROS nodes is aggregated and
sent to the Tango Bus.
"""

import rospy
from husky_msgs.msg import HuskyStatus
import PyTango
from PyTango import AttrQuality, AttrWriteType, DispLevel, DevState, AttrDataFormat, ArgType
from PyTango.server import Device, DeviceMeta, attribute, command, server_run
from PyTango.server import class_property, device_property
import time

# Toggle between Simulation mode and Real Mode
SIMULATION_MODE = True

# Configuration Variable for Diagnostics Topic
DIAGNOSTICS_TOPIC = "/status"

# Tango Device server which publishes diagnostic information on the bus
class PyRobotDiagnostics(Device):

    __metaclass__ = DeviceMeta
    POLLING_SHORT = 30  # polling period for tasks which change frequently
    POLLING_LONG = 100  # polling period for parameters which change slowly over time

    # Attribute definitions for various diagnostic messages
    battery = attribute(label = "Battery Level", dtype = float,
                        display_level = DispLevel.EXPERT,
                        access = AttrWriteType.READ,
                        unit = "%", format = "8.4f",
                        min_value=0.0, max_value=100,
                        min_alarm=10, max_alarm=100,
                        min_warning=20, max_warning=90,
                        fget="getBattery", polling_period = POLLING_LONG,
                        doc="Battery charge estimate in percentage")

    batteryCapacity = attribute(label = "Battery Capacity", dtype = 'uint16',
                        display_level = DispLevel.EXPERT,
                        access = AttrWriteType.READ,
                        unit = "Wh", polling_period = POLLING_LONG,
                        fget="getBatteryCapacity",
                        doc="Robot Battery Capacity estimate in Watt hours")

    uptime = attribute(label= "Robot Uptime", dtype = int,
                       display_level = DispLevel.OPERATOR,
                       access = AttrWriteType.READ,
                       polling_period = POLLING_LONG,
                       unit = "ms", fget = "getUptime",
                       doc = "Robot MCU Uptime in milliseconds")

    currentDraw = attribute(label="Current Draw", dtype = ('float',),
                            display_level = DispLevel.OPERATOR,
                            access = AttrWriteType.READ,
                            polling_period = POLLING_LONG,
                            unit = "A", fget = "getCurrentDraw",
                            doc = "Husky Robot Current Draw in MCU,"
                            "Left driver, and Right Driver")

    voltageComponents = attribute(label = "Voltage", dtype = ('float',),
                                  display_level = DispLevel.OPERATOR,
                                  access = AttrWriteType.READ,
                                  polling_period = POLLING_LONG,
                                  unit = "V", fget = "getVoltage",
                                  doc = "Husky Robot Component Voltage ")

    temperatures = attribute(label = "Component Temperatures",
                             dtype = ('float',), display_level = DispLevel.OPERATOR,
                             access = AttrWriteType.READ,
                             polling_period = POLLING_LONG,
                             unit = "Celsius", fget = "getComponentTemperatures",
                             doc = "Component temperatures - Left/Right Driver/Motor")

    errors = attribute(label = "ERR_STOP_Conditions", dtype = ('bool',),
                       access = AttrWriteType.READ,
                       polling_period = POLLING_SHORT,
                       fget = "getErrorStopConditions",
                       doc = " Husky Error/Stop conditions")


    # Attribute getter functions

    # Get Battery Levels in percentage
    def getBattery(self):

      if SIMULATION_MODE is True:
        CURR_TIME = time.time()
        ELAPSED_TIME = CURR_TIME - START_TIME
        BATTERY_PERCENT = BatterySim - (ELAPSED_TIME * Speedup)
      self.info_stream(" Battery Levels : %f" % BATTERY_PERCENT)
      return BATTERY_PERCENT

    # Get Battery Capacity in Wh
    def getBatteryCapacity(self):
      self.info_stream(" Battery Levels : %d" % BATTERY_CAPACITY)
      return BATTERY_CAPACITY

      # Get Robot Uptime in milliseconds
    def getUptime(self):
      self.info_stream(" Uptime : %d" % UPTIME)
      return UPTIME

      # Get Voltage levels of battery and drivers
    def getVoltage(self):
      self.info_stream(" Voltage Levels  : %f V, %f V, %f V" % (VOLTAGE_BATTERY,
                       VOLTATE_LEFT_DRIVER, VOLTAGE_RIGHT_DRIVER))
      return (VOLTAGE_BATTERY, VOLTATE_LEFT_DRIVER, VOLTAGE_RIGHT_DRIVER)

    # Get Current drawn by the Microcontroller Unit, and Drivers
    def getCurrentDraw(self):
      self.info_stream(" Current Draw : %f A, %f A, %f A" %  (CURRENT_MCU,
       CURRENT_LEFT_DRIVER, CURRENT_RIGHT_DRIVER))
      return (CURRENT_MCU, CURRENT_LEFT_DRIVER, CURRENT_RIGHT_DRIVER)

    # Get Temperatures of Drivers and Motors
    def getComponentTemperatures(self):
      self.info_stream(" Temperature of Components : %f" % (TEMP_LEFT_DRIVER,
       TEMP_RIGHT_DRIVER, TEMP_LEFT_MOTOR, TEMP_RIGHT_MOTOR))
      return (TEMP_LEFT_DRIVER, TEMP_RIGHT_DRIVER,
       TEMP_LEFT_MOTOR, TEMP_RIGHT_MOTOR)

    # Get Error and Stop conditions for the Husky robot
    def getErrorStopConditions(self):
      self.info_stream("Error/Stop Conditions : %b, %b, %b, %b, %b, %b" %
      (TIMEOUT, LOCKOUT, E_STOP, ROS_PAUSE, NO_BATTERY, CURRENT_LIMIT))
      return (TIMEOUT, LOCKOUT, E_STOP, ROS_PAUSE, NO_BATTERY, CURRENT_LIMIT)


# Callback function gets the Diagnostic messages every time the message is published
def callback(self,msg):

  # All variables for diagnostic information should be global for Tango Attributes
  global UPTIME,  FREQ, CURRENT_MCU, CURRENT_LEFT_DRIVER, CURRENT_RIGHT_DRIVER
  VOLTAGE_BATTERY, VOLTATE_LEFT_DRIVER, VOLTAGE_RIGHT_DRIVER,
  TEMP_LEFT_DRIVER, TEMP_RIGHT_DRIVER, TEMP_LEFT_MOTOR, TEMP_RIGHT_MOTOR,
  BATTERY_CAPACITY, BATTERY_PERCENT, TIMEOUT, LOCKOUT, E_STOP, ROS_PAUSE,
  NO_BATTERY, CURRENT_LIMIT

  # Unpacking the msg structure
  [UPTIME,  FREQ, CURRENT_MCU, CURRENT_LEFT_DRIVER, CURRENT_RIGHT_DRIVER,
  VOLTAGE_BATTERY, VOLTATE_LEFT_DRIVER, VOLTAGE_RIGHT_DRIVER,
  TEMP_LEFT_DRIVER, TEMP_RIGHT_DRIVER, TEMP_LEFT_MOTOR, TEMP_RIGHT_MOTOR,
  BATTERY_CAPACITY, BATTERY_PERCENT, TIMEOUT, LOCKOUT, E_STOP, ROS_PAUSE,
  NO_BATTERY, CURRENT_LIMIT] = msg

#intiates all Robot communication and subscribes to Diagnostics Messages
def  initROS():
    try:
        rospy.init_node('DiagnosticMessageCollector', anonymous = False, log_level =rospy.INFO)
        rospy.Subscriber(DIAGNOSTICS_TOPIC, HuskyStatus, callback)
        rospy.spin()
    except ROSException as err:
        print(err)


if __name__ == "__main__":

    if SIMULATION_MODE is True:
      # global variables for simulation mode
      global BatterySim, Speedup, START_TIME
      # Battery Simulator Variable which decreases linearly over time
      BatterySim = 100.0
      #Configure Speedup
      Speedup = 0.1
      # Get current time in milliseconds
      START_TIME = time.time()

    else:
      initROS()
    # Setup the Device server in the Tango Database
    server_run((PyRobotDiagnostics,))
