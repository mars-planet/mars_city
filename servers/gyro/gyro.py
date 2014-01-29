from __future__ import division, print_function

import sys
import argparse
from collections import deque

import gevent
from emokit import emotiv


def read_from_headset(polling, logfile):
    headset = emotiv.Emotiv()
    gevent.spawn(headset.setup)
    gevent.sleep(1)
    # don't move the headset while reading this
    #packet = headset.dequeue()
    #x = packet.gyroX
    #y = packet.gyroY
    x = 1692.5231211892856
    y = 1700.0293896006028

    x = 1693.33246285
    y = 1700.17013241

    x = 1693.83995781
    y = 1699.888864

    x = 1691
    y = 1700



    #print("Calibrating (don't move the headset)...")
    #PNUM = 500
    #xs, ys = 0, 0
    #print('-'*50)
    #for k in range(PNUM):
        #if k%1000 == 0:
            #print('|', end='')
            #sys.stdout.flush()
        #packet = headset.dequeue()
        #xs += packet.gyroX
        #ys += packet.gyroY
    #x0 = xs / PNUM
    #y0 = ys / PNUM
    #print('[done]')
    #print('x0:y0 = %4d:%4d' % (x0, y0))
    #x, y = x0, y0

    # Packets are received at 128Hz (see emotive_protocol.asciidoc in
    # the openyou repo), i.e. 128 packets per second.
    # If the polling time is 0.2s, we are reading at 5Hz, so we store
    # in a queue 128Hz / 5Hz = ~25 packets and write on stdout the
    # average values of the 25 packets before resetting the queue.
    size = int(128 / (1/polling))
    positions = deque(maxlen=size)

    print('-----')
    sys.stderr.write('START\n')

    # if a logfile is provide, open it
    if logfile is not None:
        f = open(logfile, 'w')

    # this controls how much you have to move your head before changing
    # direction
    THRESHOLD = 300

    xpos, ypos = 0, 0
    xspeed, yspeed = 0, 0
    #last_direction = 'STOP'
    try:
        while True:
            packet = headset.dequeue()
            #print(packet.gyroX, packet.gyroY)
            xdiff = x - packet.gyroX
            ydiff = y - packet.gyroY
            if abs(xdiff) > 0:
                xspeed += xdiff
                xpos += int(round(xspeed/10))
            if abs(ydiff) > 0:
                yspeed -= ydiff
                ypos += int(round(yspeed/10))
            positions.append((xpos, ypos))
            if len(positions) >= size:
                xavg = sum(pos[0] for pos in positions) / len(positions)
                yavg = sum(pos[1] for pos in positions) / len(positions)
                #print('  xp:yp = %5d %5d' % (xavg, yavg))
                positions.clear()
                if abs(xavg) > THRESHOLD or abs(yavg) > THRESHOLD:
                    if abs(xavg) > abs(yavg):
                        direction = 'RIGHT' if xavg > 0 else 'LEFT'
                    else:
                        direction = 'FORWARD' if yavg > 0 else 'BACKWARDS'
                else:
                    direction = 'STOP'
                #if direction != last_direction:
                    #last_direction = direction
                print(direction)
                sys.stdout.flush()

                #print '%4d %4d' % (xspeed, yspeed)
                print('  x:y = %4d %4d' % (xpos, ypos))
            x, y = packet.gyroX, packet.gyroY
            if logfile is not None:
                f.write('%s %s\n' % (packet.gyroX, packet.gyroY))
            #headset.packets.empty()
    except KeyboardInterrupt:
        headset.close()
    finally:
        headset.close()
        if logfile is not None:
            f.close()

if __name__ == "__main__":
    desc = 'Read gyroscope data from the headset and print directions on stdout.'
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument("polling", type=int,
                        help="set the time interval (in ms) between the outputs")
    parser.add_argument("--log", metavar="FILENAME", default=None,
                        dest="logfile", help="log the data on a file")
    args = parser.parse_args()

    # Convert the polling time from milliseconds to seconds.
    polling = args.polling / 1000
    read_from_headset(polling, args.logfile)
