from __future__ import division

import sys
import json
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
    packets = deque(maxlen=size)

    print '-----'
    sys.stderr.write('START\n')

    # Read the packets in loop and add them to the queue.
    # Once enough packets are collected, calculate the average of the values,
    # serialize the result with json, print it on stdout, and reset the queue
    try:
        while True:
            packet = headset.dequeue()
            # We only need the sensors data + the battery.
            # The rawdata, counter, and sync are not needed;
            # the other values are redundant
            data = dict(packet.sensors)
            del data['Unknown']  # we don't need this either
            data['battery'] = packet.battery
            packets.append(data)
            packets_num = len(packets)
            if packets_num >= size:
                avgdata = {}
                for key in data:
                    if key == 'battery':
                        # return the battery level for the last packet
                        avgdata[key] = packets[-1][key]
                    else:
                        # calculate the average of sensors qualities and values
                        # for all the packets
                        qsum, vsum = 0, 0
                        for p in packets:
                            sensor_data = p[key]
                            qsum += sensor_data['quality']
                            vsum += sensor_data['value']
                        avgdata[key] = dict(quality=qsum/packets_num,
                                            value=vsum/packets_num)
                print json.dumps(avgdata)
                sys.stdout.flush()
                packets.clear()
    except KeyboardInterrupt:
        headset.close()
    finally:
        headset.close()
