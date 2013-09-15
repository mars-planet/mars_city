from __future__ import division
from __future__ import print_function

import sys
import json
import time
import argparse
from collections import deque

import gevent
from emokit import emotiv


def read_from_headset(polling, logfile):
    headset = emotiv.Emotiv()
    gevent.spawn(headset.setup)
    gevent.sleep(1)

    # Packets are received at 128Hz (see emotive_protocol.asciidoc in
    # the openyou repo), i.e. 128 packets per second.
    # If the polling time is 0.2s, we are reading at 5Hz, so we store
    # in a queue 128Hz / 5Hz = ~25 packets and write on stdout the
    # average values of the 25 packets before resetting the queue.
    size = int(128 / (1/polling))
    packets = deque(maxlen=size)

    print('-----')
    sys.stderr.write('START\n')

    # if a logfile is provide, open it
    if logfile is not None:
        f = open(logfile, 'w')

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
                json_data = json.dumps(avgdata)
                print(json_data)
                if logfile is not None:
                    f.write(json_data + '\n')
                sys.stdout.flush()
                packets.clear()
    except KeyboardInterrupt:
        headset.close()
    finally:
        headset.close()
        if logfile is not None:
            f.close()


def read_from_file(polling, simfile):
    # simulation mode
    print('-----')
    sys.stderr.write('START\n')
    with open(simfile) as f:
        for line in f:
            print(line, end='')
            time.sleep(polling)


if __name__ == "__main__":
    desc = ('Read data from the headset or logfile, serialize them as JSON, '
            'and print them on stdout.')
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument("polling", type=int,
                        help="set the time interval (in ms) between the outputs")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--log", metavar="FILENAME", default=None,
                       dest="logfile", help="log the data on a file")
    group.add_argument("--sim", metavar="FILENAME", default=None,
                       dest="simfile", help="read data from a log file")
    args = parser.parse_args()

    # Convert the polling time from milliseconds to seconds.
    polling = args.polling / 1000

    if args.simfile is not None:
        read_from_file(polling, args.simfile)
    else:
        read_from_headset(polling, args.logfile)
