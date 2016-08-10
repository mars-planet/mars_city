import sys
import argparse
import os
import threading
import PyTango

import copy
from PyTango.server import Device, DeviceMeta, attribute, command
from PyTango.server import class_property, device_property

from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import ctypes
import _ctypes
import pygame

import numpy

# Check dlls before of all

KINECT_FPS = 30
if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

# colors for drawing different bodies
SKELETON_COLORS = [pygame.color.THECOLORS["red"],
                   pygame.color.THECOLORS["blue"],
                   pygame.color.THECOLORS["green"],
                   pygame.color.THECOLORS["orange"],
                   pygame.color.THECOLORS["purple"],
                   pygame.color.THECOLORS["yellow"],
                   pygame.color.THECOLORS["violet"]]


class PyTracker(Device):
    __metaclass__ = DeviceMeta

    POLLING = 30

    # the Tracker object, from which get skeletal data
    skletonobj = None

    def init_device(self):
        Device.init_device(self)
        self.info_stream('In Python init_device method')
        self.set_state(PyTango.DevState.ON)
        self.skleton = PyTango.IMAGE

    def set_skleton(self, skletonobj):
        self.skletonobj = skletonobj

    skleton = attribute(label="skleton",
                        dtype=[[numpy.float64,
                                PyTango.IMAGE,
                                PyTango.READ, 100, 100], ],
                        polling_period=POLLING,
                        max_dim_x=1024,
                        max_dim_y=1024)

    def aquire_skelton_status(self):

        if self.skletonobj is None:
            return False
        else:
            return True

    def read_skleton(self):
        if self.aquire_skelton_status():
            try:
                bodies = self.skeltonobj.get_bodies()
                for i in range(0, self._kinect.max_body_count):
                    body = bodies.bodies[i]
                    if not body.is_tracked:
                        continue

                    joints = body.joints
                    # print joints
                    # convert joint coordinates to color space
                    joint_points = self.skletonobj._kinect.body_joints_to_color_space(joints)
                    self.coord_array = self.skletonobj.save_body_coodrinates(joints, joint_points)
                    self.push_change_event("skleton", self.coord_array)

            except:
                print("error reading skleton")


class Tracker:

    device_name = None

    def get_bodies(self):
        if self._bodies is not None:
            return self._bodies

    def set_tracker_in_device(self):
        util = PyTango.Util.instance()
        device = util.get_device_by_name("C3/unity/" + self.device_name)
        device.set_skleton(self)

    def start_tango(self):
        PyTango.server.run((PyTracker,),
                           post_init_callback=self.set_tracker_in_device,
                           args=['tango_pygame', self.device_name])

    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        """
        #self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck)
        joints = body.joints
        #print joints
        # convert joint coordinates to color space
        joint_points = self._kinect.body_joints_to_color_space(joints)
        """
        joint0State = joints[joint0].TrackingState
        joint1State = joints[joint1].TrackingState

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked):
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        try:
            # print start, "    ", end
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except:  # need to catch it due to possible invalid positions (with inf)
            pass

    def skleton2numpyarray(self, skleton, strlength):
        names = ['id', 'data']
        len1 = "S" + str(strlength) # string length accepted
        formats = ['S9', len1]
        dtype = dict(names=names, formats=formats)
        array = numpy.array(skleton.items(), dtype=dtype)
        return array

    def get_coordinates(self, joints, jointPoints, color, start, end):
        final_coordinates = []
        joint0State = joints[start].TrackingState
        joint1State = joints[end].TrackingState

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked):
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good
        startcordinates = (jointPoints[start].x, jointPoints[start].y)
        endcoordinates = (jointPoints[end].x, jointPoints[end].y)

        try:
            # print start, "    ", end
            final_coordinates.append(startcordinates)
            final_coordinates.append(endcoordinates)
            return final_coordinates
        except:  # need to catch it due to possible invalid positions (with inf)
            pass

    def save_body_coodrinates(self, joints, jointPoints):
        # Torso
        torso = []
        torso.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck))
        torso.append(
            self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder))
        torso.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_SpineShoulder,
                                          PyKinectV2.JointType_SpineMid))
        torso.append(
            self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase))
        torso.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_SpineShoulder,
                                          PyKinectV2.JointType_ShoulderRight))
        torso.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_SpineShoulder,
                                          PyKinectV2.JointType_ShoulderLeft))
        torso.append(
            self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight))
        torso.append(
            self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft))

        # Right Arm
        right_arm = []
        right_arm.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_ShoulderRight,
                                              PyKinectV2.JointType_ElbowRight))
        right_arm.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_ElbowRight,
                                              PyKinectV2.JointType_WristRight))
        right_arm.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_WristRight,
                                              PyKinectV2.JointType_HandRight))
        right_arm.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_HandRight,
                                              PyKinectV2.JointType_HandTipRight))
        right_arm.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_WristRight,
                                              PyKinectV2.JointType_ThumbRight))

        # Left Arm
        left_arm = []
        left_arm.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_ShoulderLeft,
                                             PyKinectV2.JointType_ElbowLeft))
        left_arm.append(
            self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_ElbowLeft,
                                 PyKinectV2.JointType_WristLeft))
        left_arm.append(
            self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft))
        left_arm.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_HandLeft,
                                             PyKinectV2.JointType_HandTipLeft))
        left_arm.append(
            self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_WristLeft,
                                 PyKinectV2.JointType_ThumbLeft))

        # Right Leg
        right_leg = []
        right_leg.append(
            self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight))
        right_leg.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_KneeRight,
                                              PyKinectV2.JointType_AnkleRight))
        right_leg.append(self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_AnkleRight,
                                              PyKinectV2.JointType_FootRight))

        # Left Leg
        left_leg = []
        left_leg.append(
            self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft))
        left_leg.append(
            self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft))
        left_leg.append(
            self.get_coordinates(joints, jointPoints, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft))


        skeleton = {"torso": str(torso), "right arm": str(right_arm), "left arm": str(left_arm),
                    "right leg": str(right_leg),
                    "left leg": str(left_leg)}
        strlength = len(torso) + len(right_arm) + len(left_arm) + len(right_leg) + len(left_leg)
        skeletonnd = self.skleton2numpyarray(skeleton, strlength)
        return skeletonnd

    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_SpineMid)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_ShoulderRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_ShoulderLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft)

        # Right Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight,
                            PyKinectV2.JointType_ElbowRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight,
                            PyKinectV2.JointType_WristRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight,
                            PyKinectV2.JointType_HandTipRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight,
                            PyKinectV2.JointType_ThumbRight)

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft,
                            PyKinectV2.JointType_ElbowLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft)

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight)

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft)

    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def __init__(self, device_name):
        self.device_name = device_name
        pygame.init()

        self._clock = pygame.time.Clock()
        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1),
                                               pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)
        pygame.display.set_caption("Kinect for Windows v2 Body Game")
        self._done = False
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(
            PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface(
            (self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data
        self._bodies = None
        self.start_tango()

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get():  # User did something
                if event.type == pygame.QUIT:  # If user clicked close
                    self._done = True  # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE:  # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'],
                                                           pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

            # --- Game logic should go here

            # --- Getting frames and drawing
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame():
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None:
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked:
                        continue

                    joints = body.joints
                    # print joints
                    # convert joint coordinates to color space
                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i])
                    # self.save_body_coodrinates(joints,joint_points)

            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size)
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height))
            self._screen.blit(surface_to_draw, (0, 0))
            surface_to_draw = None
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(60)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Read skeletal data '
                                                 'from the Kinect or logfile, and '
                                                 'publish them on Tango bus or save '
                                                 'on a JSON file.')

    parser.add_argument('device',
                        choices=['eras1'],
                        help='the device where this data will be published '
                             '(eras-X)')

    # # parse arguments
    try:
        args = parser.parse_args()
    except:
        pass
    else:
        print args.device, "hello"
        t = Tracker(args.device)
        t.run()
