from __future__ import division

import sys
from collections import deque

import gevent
from emokit import emotiv

if __name__ == "__main__":
    headset = emotiv.Emotiv()
    gevent.spawn(headset.setup)
    gevent.sleep(1)

    polling = int(sys.argv[1])/1000
    size = int(128 / (1/polling))
    print size
    print '-----'
    sys.stderr.write('START\n')

    # don't move the headset while reading this
    packet = headset.dequeue()
    x, y = packet.gyroX, packet.gyroY
    xpos, ypos = 0, 0
    xspeed, yspeed = 0, 0
    xs, ys = deque(maxlen=size), deque(maxlen=size)
    try:
        while True:
            packet = headset.dequeue()
            xdiff = x-packet.gyroX
            ydiff = y-packet.gyroY
            if abs(xdiff) > 0:
                xspeed += xdiff
                xpos += int(round(xspeed))
            #xpos = max(min(xpos, xmax), 0)
            if abs(ydiff) > 0:
                yspeed -= ydiff
                ypos += int(round(yspeed))
            x, y = packet.gyroX, packet.gyroY
            xs.append(x)
            ys.append(y)
            if len(xs) >= size:
                print '%d;%d' %(sum(xs)/size,  sum(ys)/size)
                sys.stdout.flush()
                xs.clear()
                ys.clear()
    except KeyboardInterrupt:
        headset.close()
    finally:
        headset.close()
