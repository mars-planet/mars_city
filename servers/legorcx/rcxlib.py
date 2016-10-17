"""Module to send commands via IR to a LEGO RCX 2.0"""

# Written by Ezio Melotti
# Based on the rcx.cpp library by Daniel Berger

# Sending two commands in rapid succession doesn't work,
# so we need to sleep at least 0.1s between them.
# The code should be optimized to group similar commands
# (e.g. setting the same speed on two motors).
from time import sleep

# To create /dev/usb/lego0, do the following:
#   sudo modprobe legousbtower
#   sudo ln -s /dev/usb/legousbtower0 /dev/usb/lego0
#   sudo chmod 777 /dev/usb/lego0
# On Windows try to use "\\\\.\\LEGOTOWER1" (untested)

class RCX(object):
    def __init__(self, path='/dev/usb/lego0'):
        self.ir = open('/dev/usb/lego0', 'wb', 0)
        self.switchbit = 0
        self.motors_on = [0, 0, 0]
        self.motors_speeds = [0, 0, 0]
        self.motors_directions = [1, 1, 1]
        self.default_directions = [1, 1, 1]

    def set_motors(self, A, B, C):
        motors = [A, B, C]
        if motors == self.motors_on:
            return
        byte = A*1 + B*2 + C*4
        if not all(motors):
            # turn all motors
            self._send('\x21\x47')
            sleep(0.1)
        if any(motors):
            # turn on motors
            msg = '\x21' + chr(byte+0x80)
            self._send(msg)
        self.motors_on = motors

    def set_speeds(self, A, B, C):
        #valid speeds are from -7 to 7
        speeds = [A, B, C]
        absspeeds = map(abs, speeds)
        a, b, c = absspeeds
        if a != self.motors_speeds[0]:
            self._send('\x13\x01\x02' + chr(a))
            sleep(0.1)
        if b != self.motors_speeds[1]:
            self._send('\x13\x02\x02' + chr(b))
            sleep(0.1)
        if c != self.motors_speeds[2]:
            self._send('\x13\x04\x02' + chr(c))
            sleep(0.1)
        self.motors_speeds = [a, b, c]
        directions = [cmp(A, 0), cmp(B, 0), cmp(C, 0)]
        if directions != self.motors_directions:
            self.set_directions(*directions)
            print 'RCX.set_speeds: set directions'

    def set_directions(self, A, B, C):
        a = A * self.default_directions[0]
        b = B * self.default_directions[1]
        c = C * self.default_directions[2]
        directions = [a, b, c]
        if directions == self.motors_directions:
            return
        if a:
            self._send('\xe1\x81' if a > 0 else '\xe1\x01')
            sleep(0.1)
        if b:
            self._send('\xe1\x82' if b > 0 else '\xe1\x02')
            sleep(0.1)
        if c:
            self._send('\xe1\x84' if c > 0 else '\xe1\x04')
        self.motors_directions = [A, B, C]

    def _send(self, msg):
        print '  >', ' '.join('%02X' % ord(x) for x in msg)
        # maximal length of msg is 125 bytes
        s = 0
        #create message to send
        sendbuf = '\x55\xff\x00'
        fst = ord(msg[0]) | self.switchbit
        self.switchbit ^= 0x08
        msg = chr(fst) + msg[1:]
        for c in msg:
            sendbuf += c
            sendbuf += chr(ord(c)^0xff)
            s += ord(c)
        s = s % 256 # cast to unsigned char
        sendbuf += chr(s)
        sendbuf += chr(s^0xff)
        #print [hex(ord(x)) for x in sendbuf]
        return self.ir.write(sendbuf)

    def beep(self, sound=1):
        # 0: blip, 1: beep beep, 2: downward tones,
        # 3: upward tones, 4: low buzz, 5: fast upward tones
        if sound < 0 or sound > 5:
            raise ValueError('sound should be in range(6)')
        msg = '\x51' + chr(sound)
        self._send(msg)
