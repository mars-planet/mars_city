#! /usr/bin/python

import trollius
from trollius import From
import PYTango

import pygazebo
import pygazebo.msg.pose_pb2
import math

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
        self.set_type("ArrowDevice")

class PySkidDrive(PyTango.Device_4Impl):
    def __init__(self, cl, name):
        self.devices = {}
        PyTango.Device_4Impl.__init__(self, cl, name)
        self.info_stream('In Arrow.__init__')
        PyArrow.init_device(self)

    def __del__(self):
        print ("destructor")
        # shut down curses cleanly
        curses.nocbreak(); self.stdscr.keypad(0); curses.echo()
        curses.endwin()

    def read_translate_speed(self, the_att):
        char = self.stdscr.getch()
        if char == curses.KEY_RIGHT:
            self.translate_speed=1
            self.rotate_speed=-1
        elif char == curses.KEY_LEFT:
            self.translate_speed=1
            self.rotate_speed=1
        elif char == curses.KEY_UP:
            self.translate_speed=1
            self.rotate_speed=0
        elif char == curses.KEY_DOWN:
            self.translate_speed=-1
            self.rotate_speed=0
        else: # char == -1 if timeout on stdscr
            self.translate_speed=0
            self.rotate_speed=0

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



if __name__ == '__main__':
    util = PyTango.Util(sys.argv)
    util.add_class(PyDevice, PySkidDrive)

    U = PyTango.Util.instance()
    U.server_init()
    U.server_run()

running = True

class Quaternion:
	def __init__(self):
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.w = 0.0


def quaternion_to_rpy(q, roll, pitch, yaw):
	yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
	pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
	roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);

def rpy_to_quaternion(q, roll, pitch, yaw):
	q.x = (cos(roll/2) * cos(pitch/2) * cos(yaw/2)) + (sin(roll/2) * sin(pitch/2) * sin(yaw/2))
	q.y = (sin(roll/2) * cos(pitch/2) * cos(yaw/2)) - (cos(roll/2) * sin(pitch/2) * sin(yaw/2))
	q.z = (cos(roll/2) * sin(pitch/2) * cos(yaw/2)) + (sin(roll/2) * cos(pitch/2) * sin(yaw/2))
	q.w = (cos(roll/2) * cos(pitch/2) * sin(yaw/2)) - (sin(roll/2) * sin(pitch/2) * cos(yaw/2))

@trollius.coroutine
def publish_loop():
    manager = yield From(pygazebo.connect())

    publisher = yield From(
        manager.advertise('/gazebo/default/trevor/model/skid_drive/vel_cmd',
                          'gazebo.msgs.Pose'))

    message = pygazebo.msg.pose_pb2.Pose()
    message.position.x = self.translate_speed

    Quaternion q
    rpy_to_quaternion(q, 0, 0, self.rotate_speed)
    message.orientation.x = q.x
    message.orientation.y = q.y
    message.orientation.z = q.z
    message.orientation.w = q.w

    while running:
        yield From(publisher.publish(message))
        yield From(trollius.sleep(0.1))

loop = trollius.get_event_loop()
loop.run_until_complete(publish_loop())
