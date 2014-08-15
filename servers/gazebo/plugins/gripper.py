
#
import trollius
from trollius import From
import pygazebo
import pygazebo.msg.joint_cmd_pb2
# import math

import sys
import PyTango
import curses

lift_force = 0
grip_force = 0


class PyDevice(PyTango.DeviceClass):
    cmd_list = {}
    attr_rw_type = [
        [PyTango.ArgType.DevFloat,
         PyTango.AttrDataFormat.SCALAR,
         PyTango.AttrWriteType.READ_WRITE],
        {'polling period': 100}
    ]
    attr_list = {
        'lift_force': attr_rw_type,
        'grip_force': attr_rw_type,
    }

    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type("GripperDevice")


class PyTangoGripper(PyTango.Device_4Impl):
    def __init__(self, cl, name):
        self.devices = {}
        self.DELTA_FORCE = 1
        PyTango.Device_4Impl.__init__(self, cl, name)
        self.info_stream('In Gripper.__init__')
        PyTangoGripper.init_device(self)

    def __del__(self):
        print ("destructor")
        # shut down curses cleanly
        curses.nocbreak(); self.stdscr.keypad(0); curses.echo()
        curses.endwin()

    def read_lift_force(self, the_att):
        char = self.stdscr.getch()
        if char == curses.KEY_RIGHT:
            self.grip_force += self.DELTA_FORCE
        elif char == curses.KEY_LEFT:
            self.grip_force -= self.DELTA_FORCE
        elif char == curses.KEY_UP:
            self.lift_force += self.DELTA_FORCE
        elif char == curses.KEY_DOWN:
            self.grip_force -= self.DELTA_FORCE
        else:  # char == -1 if timeout on stdscr
            self.lift_force = 0
            self.grip_force = 0

        the_att.set_value(self.lift_force)

    def read_grip_force(self, the_att):
        the_att.set_value(self.grip_force)

    def write_lift_force(self, the_att):
        self.lift_force = the_att.get_write_value("""data""")

    def write_grip_force(self, the_att):
        self.grip_force = the_att.get_write_value("""data""")

    def is_lift_force_allowed(self, req_type):
        return self.get_state() in (PyTango.DevState.ON,)

    def is_grip_force_allowed(self, req_type):
        return self.get_state() in (PyTango.DevState.ON,)

    def init_device(self):
        self.info_stream('In Python init_device method')
        self.set_state(PyTango.DevState.ON)
        self.lift_force = 0
        self.grip_force = 0
        # get the curses stdscr window
        self.stdscr = curses.initscr()

        # turn off input echoing
        curses.noecho()
        # respond to keys immediately (don't wait for enter)
        curses.cbreak()
        # avoid blocking
        self.stdscr.nodelay(1)
        # map arrow keys to special values
        self.stdscr.keypad(True)


@trollius.coroutine
def publish_loop(U):
    manager = yield From(pygazebo.connect())

    lift_publisher = yield From(
        manager.advertise('/gazebo/default/trevor/front_gripper/lift_force',
                          'gazebo.msgs.JointCmd'))
    grip_publisher = yield From(
        manager.advertise('/gazebo/default/trevor/front_gripper/grip_force',
                          'gazebo.msgs.JointCmd'))

    lift_message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    lift_message.axis = 0
    lift_message.force = U.lift_force
    grip_message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    grip_message.axis = 0
    grip_message.force = U.grip_force
    lift_message.name = "trevor::front_gripper::palm_raiser"

    while True:
        yield From(lift_publisher.publish(lift_message))
        yield From(trollius.sleep(0.1))
        grip_message.name = "trevor::front_gripper::left_finger_tip"
        yield From(grip_publisher.publish(grip_message))
        yield From(trollius.sleep(0.1))
        grip_message.name = "trevor::front_gripper::right_finger_tip"
        yield From(grip_publisher.publish(grip_message))
        yield From(trollius.sleep(0.1))
        grip_message.name = "trevor::front_gripper::palm_left_finger"
        yield From(grip_publisher.publish(grip_message))
        yield From(trollius.sleep(0.1))
        grip_message.name = "trevor::front_gripper::palm_right_finger"
        yield From(grip_publisher.publish(grip_message))
        yield From(trollius.sleep(0.1))
        print("Publishing")


@trollius.coroutine
def server_loop(U):
    U.server_run()

if __name__ == '__main__':
    util = PyTango.Util(sys.argv)
    util.add_class(PyDevice, PyTangoGripper)

    U = PyTango.Util.instance()
    U.server_init()

    tasks = [
        trollius.Task(publish_loop(U)),
        trollius.Task(server_loop(U)),
    ]

    loop = trollius.get_event_loop()
    loop.run_until_complete(trollius.wait(tasks))
    loop.close()
