import sys
import argparse
import os
import threading
import time
import json
import PyTango
import pygame

from pykinect import nui
from pykinect.nui import JointId
from pykinect.nui.structs import TransformSmoothParameters
from pykinect.nui import SkeletonTrackingState
from urllib import urlopen


class PyDevice(PyTango.DeviceClass):
    cmd_list = {}
    attr_type = [
        [PyTango.ArgType.DevFloat,
            PyTango.AttrDataFormat.SPECTRUM,
            PyTango.AttrWriteType.READ, 3],
        {'polling period': 30}
    ]
    attr_list = {
        'skeleton_head': attr_type,
        'skeleton_neck': attr_type,
        'skeleton_left_shoulder': attr_type,
        'skeleton_right_shoulder': attr_type,
        'skeleton_left_elbow': attr_type,
        'skeleton_right_elbow': attr_type,
        'skeleton_left_hand': attr_type,
        'skeleton_right_hand': attr_type,
        'skeleton_torso': attr_type,
        'skeleton_left_hip': attr_type,
        'skeleton_right_hip': attr_type,
        'skeleton_left_knee': attr_type,
        'skeleton_right_knee': attr_type,
        'skeleton_left_foot': attr_type,
        'skeleton_right_foot': attr_type
    }

    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type("TrackerDevice")


class PyTracker(PyTango.Device_4Impl):
    __tracker = None

    def set_tracker(self, tracker):
        self.__tracker = tracker

    def acquire_skeleton_lock(self):
        skel_is_none = self.__tracker.get_skeleton() is None
        lock_acquired = self.__tracker.get_skeleton_lock().acquire(False)
        if skel_is_none or not lock_acquired:
            return False
        else:
            return True

    def release_skeleton_lock(self):
        self.__tracker.get_skeleton_lock().release()

    def __init__(self, cls, name):
        self.devices = {}
        PyTango.Device_4Impl.__init__(self, cls, name)
        self.info_stream('In 1Tracker.__init__')
        PyTracker.init_device(self)

    def read_skeleton_head(self, the_att):
        # sync access to skeleton
        if not self.acquire_skeleton_lock():
            the_att.set_value(self.skeleton_head)
            return

        try:
            skeleton = self.__tracker.get_skeleton()

            self.skeleton_head = (skeleton[JointId.Head].x,
                                  skeleton[JointId.Head].y,
                                  skeleton[JointId.Head].z)
            self.skeleton_neck = (skeleton[JointId.ShoulderCenter].x,
                                  (
                                    skeleton[JointId.ShoulderLeft].y * 0.3 +
                                    skeleton[JointId.ShoulderRight].y * 0.3 +
                                    skeleton[JointId.ShoulderCenter].y * 0.4
                                  ),
                                  skeleton[JointId.ShoulderCenter].z)
            self.skeleton_left_shoulder = (skeleton[JointId.ShoulderLeft].x,
                                           skeleton[JointId.ShoulderLeft].y,
                                           skeleton[JointId.ShoulderLeft].z)
            self.skeleton_right_shoulder = (skeleton[JointId.ShoulderRight].x,
                                            skeleton[JointId.ShoulderRight].y,
                                            skeleton[JointId.ShoulderRight].z)
            self.skeleton_left_elbow = (skeleton[JointId.ElbowLeft].x,
                                        skeleton[JointId.ElbowLeft].y,
                                        skeleton[JointId.ElbowLeft].z)
            self.skeleton_right_elbow = (skeleton[JointId.ElbowRight].x,
                                         skeleton[JointId.ElbowRight].y,
                                         skeleton[JointId.ElbowRight].z)
            self.skeleton_left_hand = (skeleton[JointId.HandLeft].x,
                                       skeleton[JointId.HandLeft].y,
                                       skeleton[JointId.HandLeft].z)
            self.skeleton_right_hand = (skeleton[JointId.HandRight].x,
                                        skeleton[JointId.HandRight].y,
                                        skeleton[JointId.HandRight].z)
            self.skeleton_torso = (skeleton[JointId.Spine].x,
                                   skeleton[JointId.Spine].y,
                                   skeleton[JointId.Spine].z)
            self.skeleton_left_hip = (skeleton[JointId.HipLeft].x,
                                      skeleton[JointId.HipLeft].y,
                                      skeleton[JointId.HipLeft].z)
            self.skeleton_right_hip = (skeleton[JointId.HipRight].x,
                                       skeleton[JointId.HipRight].y,
                                       skeleton[JointId.HipRight].z)
            self.skeleton_left_knee = (skeleton[JointId.KneeLeft].x,
                                       skeleton[JointId.KneeLeft].y,
                                       skeleton[JointId.KneeLeft].z)
            self.skeleton_right_knee = (skeleton[JointId.KneeRight].x,
                                        skeleton[JointId.KneeRight].y,
                                        skeleton[JointId.KneeRight].z)
            self.skeleton_left_foot = (skeleton[JointId.FootLeft].x,
                                       skeleton[JointId.FootLeft].y,
                                       skeleton[JointId.FootLeft].z)
            self.skeleton_right_foot = (skeleton[JointId.FootRight].x,
                                        skeleton[JointId.FootRight].y,
                                        skeleton[JointId.FootRight].z)

            # TODO: add skeleton height
            # TODO: user step estimation (and add command)
        finally:
            self.release_skeleton_lock()

        the_att.set_value(self.skeleton_head)

    def read_skeleton_neck(self, the_att):
        the_att.set_value(self.skeleton_neck)

    def read_skeleton_left_shoulder(self, the_att):
        the_att.set_value(self.skeleton_left_shoulder)

    def read_skeleton_right_shoulder(self, the_att):
        the_att.set_value(self.skeleton_right_shoulder)

    def read_skeleton_left_elbow(self, the_att):
        the_att.set_value(self.skeleton_left_elbow)

    def read_skeleton_right_elbow(self, the_att):
        the_att.set_value(self.skeleton_right_elbow)

    def read_skeleton_left_hand(self, the_att):
        the_att.set_value(self.skeleton_left_hand)

    def read_skeleton_right_hand(self, the_att):
        the_att.set_value(self.skeleton_right_hand)

    def read_skeleton_torso(self, the_att):
        the_att.set_value(self.skeleton_torso)

    def read_skeleton_left_hip(self, the_att):
        the_att.set_value(self.skeleton_left_hip)

    def read_skeleton_right_hip(self, the_att):
        the_att.set_value(self.skeleton_right_hip)

    def read_skeleton_left_knee(self, the_att):
        the_att.set_value(self.skeleton_left_knee)

    def read_skeleton_right_knee(self, the_att):
        the_att.set_value(self.skeleton_right_knee)

    def read_skeleton_left_foot(self, the_att):
        the_att.set_value(self.skeleton_left_foot)

    def read_skeleton_right_foot(self, the_att):
        the_att.set_value(self.skeleton_right_foot)

    def init_device(self):
        self.info_stream('In Python init_device method')
        self.set_state(PyTango.DevState.ON)
        self.skeleton_head = (0, 0, 0)
        self.skeleton_neck = (0, 0, 0)
        self.skeleton_left_shoulder = (0, 0, 0)
        self.skeleton_right_shoulder = (0, 0, 0)
        self.skeleton_left_elbow = (0, 0, 0)
        self.skeleton_right_elbow = (0, 0, 0)
        self.skeleton_left_hand = (0, 0, 0)
        self.skeleton_right_hand = (0, 0, 0)
        self.skeleton_torso = (0, 0, 0)
        self.skeleton_left_hip = (0, 0, 0)
        self.skeleton_right_hip = (0, 0, 0)
        self.skeleton_left_knee = (0, 0, 0)
        self.skeleton_right_knee = (0, 0, 0)
        self.skeleton_left_foot = (0, 0, 0)
        self.skeleton_right_foot = (0, 0, 0)


class Tracker:
    __KINECTEVENT = pygame.USEREVENT

    __SMOOTH_PARAMS_SMOOTHING = 0.7
    __SMOOTH_PARAMS_CORRECTION = 0.4
    __SMOOTH_PARAMS_PREDICTION = 0.7
    __SMOOTH_PARAMS_JITTER_RADIUS = 0.1
    __SMOOTH_PARAMS_MAX_DEV_RADIUS = 0.1
    __SMOOTH_PARAMS = TransformSmoothParameters(__SMOOTH_PARAMS_SMOOTHING,
                                                __SMOOTH_PARAMS_CORRECTION,
                                                __SMOOTH_PARAMS_PREDICTION,
                                                __SMOOTH_PARAMS_JITTER_RADIUS,
                                                __SMOOTH_PARAMS_MAX_DEV_RADIUS)

    __device_name = None
    __kinect = None

    __skeleton = None
    __skeleton_lock = threading.Lock()

    def update_skeleton(self, new_skeleton):
        self.__skeleton_lock.acquire()
        self.__skeleton = new_skeleton
        self.__skeleton_lock.release()

    def get_skeleton(self):
        return self.__skeleton

    def get_skeleton_lock(self):
        return self.__skeleton_lock

    def post_frame(self, frame):
        """Get skeleton events from the Kinect device and post them into the PyGame
        event queue."""
        try:
            pygame.event.post(
                pygame.event.Event(self.__KINECTEVENT, skeleton_frame=frame)
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

        if self.__kinect is None:
            self.__kinect = nui.Runtime()
            self.__kinect.skeleton_engine.enabled = True

        self.__kinect.skeleton_frame_ready += self.post_frame

        while True:
            event = pygame.event.wait()

            if event.type == pygame.QUIT:
                break
            elif event.type == self.__KINECTEVENT:
                self.__kinect._nui.NuiTransformSmooth(event.skeleton_frame,
                                                      self.__SMOOTH_PARAMS)
                self.save_skeletal_data(event.skeleton_frame)

                if log_file_name is not None and self.__skeleton is not None:
                    self.log_skeletal_data(log_file_name)

    def skeleton_to_json(self):
        """Convert skeleton to json"""
        tmp = [None] * JointId.count
        for joint_type, j in enumerate(self.__skeleton):
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

    def start_tango(self):
        util = PyTango.Util(['tracker', self.__device_name])
        util.add_class(PyDevice, PyTracker)
        U = PyTango.Util.instance()
        U.server_init()

        device = U.get_device_by_name("c3/mac/" + self.__device_name)
        device.set_tracker(self)

        U.server_run()

    def __init__(self, device_name, kinect=None, log=None, sim=None):
        self.__device_name = device_name
        self.__kinect = kinect

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
