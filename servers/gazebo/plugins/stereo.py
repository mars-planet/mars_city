#! /usr/bin/python

import trollius
from trollius import From

import pygazebo
import pygazebo.msg.int_pb2
import math

class PyDevice(PyTango.DeviceClass):
    cmd_list = {}
    attr_rw_type = [
        [PyTango.ArgType.DevFloat,
         PyTango.AttrDataFormat.SCALAR,
         PyTango.AttrWriteType.READ_WRITE],
        {'polling period': 100}
    ]
    # rw in case another client wants to change the angle
    attr_list = {
        'rotate_angle': attr_rw_type,
    }

    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type("ArrowDevice")

class PyStereo(PyTango.Device_4Impl):
    def __init__(self, cl, name):
        self.devices = {}
        self.DELTA_ANGLE = 0.035 # 2 degrees each time
        PyTango.Device_4Impl.__init__(self, cl, name)
        self.info_stream('In Arrow.__init__')
        PyArrow.init_device(self)

    def __del__(self):
        print ("destructor")
        # shut down curses cleanly
        curses.nocbreak(); self.stdscr.keypad(0); curses.echo()
        curses.endwin()

    def read_rotate_angle(self, the_att):
        char = self.stdscr.getch()
        if char == curses.KEY_RIGHT:
            self.rotate_angle += -self.DELTA_ANGLE
        elif char == curses.KEY_LEFT:
            self.rotate_angle += self.DELTA_ANGLE
        else: # char == -1 if timeout on stdscr
            pass

        while self.rotate_angle > math.pi:
            self.rotate_angle = 2*math.pi - self.rotate_angle
        while self.rotate_angle < -math.pi:
            self.rotate_angle = 2*math.pi + self.rotate_angle

        the_att.set_value(self.rotate_angle)

    def write_rotate_angle(self, the_att):
        self.rotate_angle = the_att.get_write_value("""data""")

    def is_rotate_angle_allowed(self, req_type):
        return self.get_state() in (PyTango.DevState.ON,)

    def init_device(self):
        self.info_stream('In Python init_device method')
        self.set_state(PyTango.DevState.ON)
        self.rotate_angle = 0
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
    util.add_class(PyDevice, PyStereo)

    U = PyTango.Util.instance()
    U.server_init()
    U.server_run()

running = True

@trollius.coroutine
def publish_loop():
    manager = yield From(pygazebo.connect())

    publisher = yield From(
        manager.advertise('/gazebo/default/trevor/model/skid_drive/vel_cmd',
                          'gazebo.msgs.Int'))

    message = pygazebo.msg.int_pb2.Int()
    message.data = self.rotate_angle

    while running:
        yield From(publisher.publish(message))
        yield From(trollius.sleep(0.1))

loop = trollius.get_event_loop()
loop.run_until_complete(publish_loop())
