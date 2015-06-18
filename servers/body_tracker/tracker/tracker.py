import sys
import argparse
import os
import threading
import time
import json
import PyTango
import pygame
import math

from pykinect import nui
from pykinect.nui import JointId
from pykinect.nui.structs import TransformSmoothParameters
from pykinect.nui import SkeletonTrackingState
from urllib import urlopen

from tango_tracker import PyTracker

class Tracker:
    KINECTEVENT = pygame.USEREVENT

    SMOOTH_PARAMS_SMOOTHING = 0.7
    SMOOTH_PARAMS_CORRECTION = 0.4
    SMOOTH_PARAMS_PREDICTION = 0.7
    SMOOTH_PARAMS_JITTER_RADIUS = 0.1
    SMOOTH_PARAMS_MAX_DEV_RADIUS = 0.1
    SMOOTH_PARAMS = TransformSmoothParameters(SMOOTH_PARAMS_SMOOTHING,
                                              SMOOTH_PARAMS_CORRECTION,
                                              SMOOTH_PARAMS_PREDICTION,
                                              SMOOTH_PARAMS_JITTER_RADIUS,
                                              SMOOTH_PARAMS_MAX_DEV_RADIUS)

    device_name = None
    kinect = None

    skeleton = None
    skeleton_lock = threading.Lock()

    def update_skeleton(self, new_skeleton):
        self.skeleton_lock.acquire()
        self.skeleton = new_skeleton
        self.skeleton_lock.release()

    def get_skeleton(self):
        return self.skeleton

    def get_skeleton_lock(self):
        return self.skeleton_lock

    def post_frame(self, frame):
        """Get skeleton events from the Kinect device and post them into the PyGame
        event queue."""
        try:
            pygame.event.post(
                pygame.event.Event(self.KINECTEVENT, skeleton_frame=frame)
            )
        except:
            # event queue full
            pass

    def save_skeletal_data(self, skeleton_frame):
        """Save skeletal data got from Kinect"""
        for index, skeleton_info in enumerate(skeleton_frame.SkeletonData):
            if skeleton_info.eTrackingState == SkeletonTrackingState.TRACKED:
                self.update_skeleton(skeleton_info.SkeletonPositions)
                # manage only the first tracked skeleton
                return

    def log_skeletal_data(self, log_file_name):
        # save to json file
        file = open(log_file_name, 'a')
        file.write(self.skeleton_to_json() + '\n')
        file.close()

    def start_tracker(self, log_file_name=None):
        pygame.init()

        if self.kinect is None:
            self.kinect = nui.Runtime()
            self.kinect.skeleton_engine.enabled = True

        self.kinect.skeleton_frame_ready += self.post_frame

        while True:
            event = pygame.event.wait()

            if event.type == pygame.QUIT:
                break
            elif event.type == self.KINECTEVENT:
                self.kinect._nui.NuiTransformSmooth(event.skeleton_frame,
                                                    self.SMOOTH_PARAMS)
                self.save_skeletal_data(event.skeleton_frame)

                if log_file_name is not None and self.skeleton is not None:
                    self.log_skeletal_data(log_file_name)

    def skeleton_to_json(self):
        """Convert skeleton to json"""
        tmp = [None] * JointId.count
        for joint_type, j in enumerate(self.skeleton):
            tmp[joint_type] = {'x': j.x, 'y': j.y, 'z': j.z, 'w': j.w}
        return json.dumps(tmp)

    def json_to_skeleton(self, json_data):
        """Convert json to a skeleton"""
        data = json.loads(json_data)
        tmp = [None] * JointId.count
        for joint_type, j in enumerate(data):
            tmp[joint_type] = nui.Vector(j['x'], j['y'], j['z'], j['w'])
        return tmp

    def start_emulate_tracker(self, sim_file_name):
        """Start a fake tracker, that reads from a file some\
        joints instead of getting them from an actual Kinect device"""
        # Kinect works at 30fps
        polling = 1.0/30.0
        with open(sim_file_name) as f:
            while True:
                line = f.readline()
                if not line:
                    f.seek(0)
                else:
                    self.update_skeleton(self.json_to_skeleton(line))
                    time.sleep(polling)

    def set_tracker_in_device(self):
        util = PyTango.Util.instance()
        device = util.get_device_by_name("c3/mac/" + self.device_name)
        device.set_tracker(self)

    def start_tango(self):
        PyTango.server.run((PyTracker,),
                           post_init_callback=self.set_tracker_in_device,
                           args=['tracker', self.device_name])

    def __init__(self, device_name, kinect=None, log=None, sim=None):
        self.device_name = device_name
        self.kinect = kinect

        if log is None:
            if sim is None:
                # start the tracker thread
                track_thr = threading.Thread(target=self.start_tracker)
            else:
                # read from file skeletal data
                track_thr = threading.Thread(target=self.start_emulate_tracker,
                                             args=[sim])
            track_thr.daemon = True
            track_thr.start()

            # start Tango
            self.start_tango()
        else:
            self.start_tracker(log)


if __name__ == '__main__':
    # argument management
    parser = argparse.ArgumentParser(description='Read skeletal data '
                                     'from the Kinect or logfile, and '
                                     'publish them on Tango bus or save '
                                     'on a JSON file.')

    parser.add_argument('device',
                        choices=['eras-1', 'eras-2',
                                 'eras-3', 'eras-4'],
                        help='the device where this data will be published '
                        '(eras-X)')

    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument("--log", help="log the data on a file",
                       metavar="FILENAME", required=False)
    group.add_argument("--sim", help="read data from a log file",
                       metavar="FILENAME", required=False)

    # parse arguments
    try:
        args = parser.parse_args()
    except:
        pass
    else:
        stop = False
        if args.log is not None and os.path.isfile(args.log):
            s = None
            while s != 'y' and s != 'Y' and s != 'n' and s != 'N':
                s = raw_input('File exists. '
                              'Do you want to overwrite it? (y/n) ')
            if s == 'n' or s == 'N':
                stop = True
            else:
                f = open(args.log, 'w')
                f.truncate()
                f.close()
        if args.sim is not None and not os.path.isfile(args.sim):
            print 'error: File does not exist'
            stop = True

        if not stop:
            Tracker(args.device, None, args.log, args.sim)
