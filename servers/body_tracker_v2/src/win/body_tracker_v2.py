from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys
import json
import numpy as np
import cv2
import pprint
import time

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


class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()

        # Total distance travelled
        self._steps_moved = 0

        # Previous step distance. A temporary value
        self._previous_step = 0

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Keep track of time
        self._start_time = time.time()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((
                                            self._infoObject.current_w >> 1,
                                            self._infoObject.current_h >> 1),
                                            pygame.HWSURFACE |
                                            pygame.DOUBLEBUF |
                                            pygame.RESIZABLE, 32)

        pygame.display.set_caption("Step tracker")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(
                                            PyKinectV2.FrameSourceTypes_Color |
                                            PyKinectV2.FrameSourceTypes_Body |
                                            PyKinectV2.FrameSourceTypes_Depth)

        # back buffer surface for getting Kinect color frames, 32bit color,
        # width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface(
                                (self._kinect.color_frame_desc.Width,
                                 self._kinect.color_frame_desc.Height), 0, 32)
        # here we will store skeleton data
        self._bodies = None

    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
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
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except:  # need to catch it due to possible invalid positions(with inf)
            pass

    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_Head,
                            PyKinectV2.JointType_Neck)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_Neck,
                            PyKinectV2.JointType_SpineShoulder)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_SpineMid)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_SpineMid,
                            PyKinectV2.JointType_SpineBase)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_ShoulderRight)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_ShoulderLeft)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_SpineBase,
                            PyKinectV2.JointType_HipRight)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_SpineBase,
                            PyKinectV2.JointType_HipLeft)

        # Right Arm
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_ShoulderRight,
                            PyKinectV2.JointType_ElbowRight)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_ElbowRight,
                            PyKinectV2.JointType_WristRight)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_WristRight,
                            PyKinectV2.JointType_HandRight)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_HandRight,
                            PyKinectV2.JointType_HandTipRight)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_WristRight,
                            PyKinectV2.JointType_ThumbRight)

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_ShoulderLeft,
                            PyKinectV2.JointType_ElbowLeft)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_ElbowLeft,
                            PyKinectV2.JointType_WristLeft)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_WristLeft,
                            PyKinectV2.JointType_HandLeft)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_HandLeft,
                            PyKinectV2.JointType_HandTipLeft)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_WristLeft,
                            PyKinectV2.JointType_ThumbLeft)

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_HipRight,
                            PyKinectV2.JointType_KneeRight)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_KneeRight,
                            PyKinectV2.JointType_AnkleRight)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_AnkleRight,
                            PyKinectV2.JointType_FootRight)

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_HipLeft,
                            PyKinectV2.JointType_KneeLeft)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_KneeLeft,
                            PyKinectV2.JointType_AnkleLeft)
        self.draw_body_bone(joints, jointPoints, color,
                            PyKinectV2.JointType_AnkleLeft,
                            PyKinectV2.JointType_FootLeft)

    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get():  # User did something
                if event.type == pygame.QUIT:  # If user clicked close
                    # Print the final no of steps taken
                    print self._steps_moved / float(1000)
                    # Flag that we are done so we exit this loop
                    self._done = True

                elif event.type == pygame.VIDEORESIZE:  # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'],
                                                           pygame.HWSURFACE |
                                                           pygame.DOUBLEBUF |
                                                           pygame.RESIZABLE,
                                                           32)

            # --- Getting frames and drawing
            # --- Woohoo! We've got a color frame! Let's fill out back buffer
            # surface with frame's data
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            """if self._kinect.has_new_depth_frame():
                dframe = self._kinect.get_last_depth_frame()
                #self.draw_color_frame(frame, self._frame_surface)
                pp=pprint.PrettyPrinter()
                #temp=np.reshape(dframe,(512,424))
                #pp.pprint(temp)
                print np.prod(dframe.shape)
                #print numpy.prod(frame.shape)
                #cv2.cvtColor(numpy.reshape(frame, (1080,1920,4)),
                                cv2.COLOR_BGR2GRAY)
                #cv2.imshow('frame',
                            cv2.cvtColor(np.reshape(frame,(1080,1920,4)),
                            cv2.COLOR_BGR2GRAY))
                #if cv2.waitKey(1) & 0xFF == ord('q'):
                #   break
                dframe = None"""

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame():
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None:
                # Use self._kinect.max_body_count for multi body tracking
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked:
                        continue

                    joints = body.joints
                    # convert joint coordinates to depth space

                    joint_points = self._kinect.body_joints_to_depth_space(joints)
                    # to align with color space with body joints
                    # joint_points = self._kinect.body_joints_to_color_space(joints)

                    # Here we will be tracking using the left ankle.
                    # We can change this to any of the 25 joints.
                    AnkleLeft_x = int(joint_points[PyKinectV2.JointType_AnkleLeft].x)
                    AnkleLeft_y = int(joint_points[PyKinectV2.JointType_AnkleLeft].y)

                    # print temp_x, temp_y
                    if not ((self._start_time + 1) > time.time()):
                        depth_frame = np.reshape(self._kinect.get_last_depth_frame(), (512, 424))
                        if not AnkleLeft_x < 512:
                            AnkleLeft_x = AnkleLeft_x % 512
                        if not AnkleLeft_y < 424:
                            AnkleLeft_y = AnkleLeft_y % 424
                        if self._previous_step == 0:
                            self._previous_step = depth_frame[AnkleLeft_x][AnkleLeft_y]
                        else:
                            temp = abs(int(depth_frame[AnkleLeft_x][AnkleLeft_y]) - int(self._previous_step))
                            if temp <= 500:
                                temp = temp
                            elif temp <= 1000:
                                temp = temp / 2
                            elif temp <= 1500:
                                temp = temp / 3
                            elif temp <= 2000:
                                temp = temp / 4
                            else:
                                temp = 0
                            self._steps_moved = self._steps_moved + temp

                            '''print "previous step", self._previous_step
                            print depth_frame[AnkleLeft_x][AnkleLeft_y]
                            print "diff is", abs(int(depth_frame[AnkleLeft_x][AnkleLeft_y]) - int(self._previous_step))'''

                            self._previous_step = depth_frame[AnkleLeft_x][AnkleLeft_y]

                        self._start_time = time.time()
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i])

            # --- copy back buffer surface pixels to the screen, resize it if
            # needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size)
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface,
                                                     (self._screen.get_width(),
                                                      target_height))
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


__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime()
game.run()
