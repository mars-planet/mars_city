from __future__ import division

import sys
from collections import deque

import gevent
from emokit import emotiv

if __name__ == "__main__":
    headset = emotiv.Emotiv()
    gevent.spawn(headset.setup)
    gevent.sleep(1)

    # Read the polling time (in milliseconds) from sys.argv and
    # convert it to seconds.  If 200 is passed, values will be
    # printed on stdout every 0.2 seconds
    polling = int(sys.argv[1])/1000

    # Packets are received at 128Hz (see emotive_protocol.asciidoc in
    # the openyou repo), i.e. 128 packets per second.
    # If the polling time is 0.2s, we are reading at 5Hz, so we store
    # in a queue 128Hz / 5Hz = ~25 packets and write on stdout the
    # average values of the 25 packets before resetting the queue.
    size = int(128 / (1/polling))
    print '-----'
    sys.stderr.write('START\n')

    # don't move the headset while reading this
    packet = headset.dequeue()
    x, y = packet.gyroX, packet.gyroY
    xs, ys = deque(maxlen=size), deque(maxlen=size)

    try:
        while True:
            packet = headset.dequeue()
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
