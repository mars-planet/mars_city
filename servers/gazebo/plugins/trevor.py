
#
import trollius
from trollius import From
import pygazebo
import pygazebo.msg.pose_pb2
from math import *

import sys
import PyTango
import curses

translate_speed = 0
rotate_speed = 0


class PyDevice(PyTango.DeviceClass):
    cmd_list = {}
    attr_rw_type = [
        [PyTango.ArgType.DevFloat,
            PyTango.AttrDataFormat.SCALAR,
            PyTango.AttrWriteType.READ_WRITE],
        {'polling period': 100}
        ]
    attr_list = {
        'translate_speed': attr_rw_type,
        'rotate_speed': attr_rw_type,
        }

    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type("GripperDevice")


class Rpy:
    def __init__(self, r=0.0, p=0.0, y=0.0):
        self.roll = r
        self.pitch = p
        self.yaw = y


class Quaternion:
    def __init__(self, a=0.0, b=0.0, c=0.0, d=0.0):
        self.x = a
        self.y = b
        self.z = c
        self.w = d


def radian_to_degree(rad):
    return rad*180/pi


def degree_to_rad(deg):
    return deg*pi/180


def quaternion_to_rpy(q):
    yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    pitch = asin(-2.0*(q.x*q.z - q.w*q.y))
    roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
    return Rpy(roll, pitch, yaw)


def rpy_to_quaternion(rpy):
    q.x = (cos(roll/2) * cos(pitch/2) * cos(yaw/2)) + (sin(roll/2) * sin(pitch/2) * sin(yaw/2))
    q.y = (sin(roll/2) * cos(pitch/2) * cos(yaw/2)) - (cos(roll/2) * sin(pitch/2) * sin(yaw/2))
    q.z = (cos(roll/2) * sin(pitch/2) * cos(yaw/2)) + (sin(roll/2) * cos(pitch/2) * sin(yaw/2))
    q.w = (cos(roll/2) * cos(pitch/2) * sin(yaw/2)) - (sin(roll/2) * sin(pitch/2) * cos(yaw/2))
    return q


class PyTrevorGazebo(PyTango.Device_4Impl):
    def __init__(self, cl, name):
        self.devices = {}
        self.DELTA_FORCE = 1
        PyTango.Device_4Impl.__init__(self, cl, name)
        self.info_stream('In Trevor.__init__')
        PyTrevorGazebo.init_device(self)

    def __del__(self):
        print ("destructor")
        # shut down curses cleanly
        curses.nocbreak(); self.stdscr.keypad(0); curses.echo()
        curses.endwin()

    def read_translate_speed(self, the_att):
        char = self.stdscr.getch()
        if char == curses.KEY_RIGHT:
            self.rotate_speed += self.DELTA_FORCE
        elif char == curses.KEY_LEFT:
            self.rotate_speed -= self.DELTA_FORCE
        elif char == curses.KEY_UP:
            self.translate_speed += self.DELTA_FORCE
        elif char == curses.KEY_DOWN:
            self.rotate_speed -= self.DELTA_FORCE
        else:  # char == -1 if timeout on stdscr
            self.translate_speed = 0
            self.rotate_speed = 0

        the_att.set_value(self.translate_speed)

    def read_rotate_speed(self, the_att):
        the_att.set_value(self.rotate_speed)

    def write_translate_speed(self, the_att):
        self.translate_speed = the_att.get_write_value("""data""")

    def write_rotate_speed(self, the_att):
        self.rotate_speed = the_att.get_write_value("""data""")

    def is_translate_speed_allowed(self, req_type):
        return self.get_state() in (PyTango.DevState.ON,)

    def is_rotate_speed_allowed(self, req_type):
        return self.get_state() in (PyTango.DevState.ON,)

    def init_device(self):
        self.info_stream('In Python init_device method')
        self.set_state(PyTango.DevState.ON)
        self.translate_speed = 0
        self.rotate_speed = 0
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

    publisher = yield From(
        manager.advertise('/gazebo/default/trevor/cmd_vel',
                          'gazebo.msgs.Pose'))

    message = pygazebo.msg.pose_pb2.Pose()
    message.position.x = U.translate_speed
    message.orientation.z = rpy_to_quaternion(
        Rpy(0, 0, (degree_to_rad(U.rotate_speed)))).z

    while True:
        yield From(publisher.publish(message))
        yield From(trollius.sleep(0.1))


@trollius.coroutine
def server_loop(U):
    U.server_run()

if __name__ == '__main__':
    util = PyTango.Util(sys.argv)
    util.add_class(PyDevice, PyTrevorGazebo)

    U = PyTango.Util.instance()
    U.server_init()

    tasks = [
        trollius.Task(publish_loop(U)),
        trollius.Task(server_loop(U)),
        ]

    loop = trollius.get_event_loop()
    loop.run_until_complete(trollius.wait(tasks))
    loop.close()
