# Check dlls before of all
import init_libs

import sys
import itertools
import pygame
from pygame.locals import *
from pykinect import nui
import time
import threading
import thread
import pygame.color

from pykinect import nui
from pykinect.nui import JointId
from pykinect.nui import SkeletonTrackingState
from pykinect.nui import RuntimeOptions
from pykinect.nui import ImageResolution
from pykinect.nui import Runtime
from pygame.color import THECOLORS
from pgu import gui

from tracker import Tracker

# windows size
SCREEN_SIZE = (860, 506)

# depth resolution used
DEPTH_RESOLUTION = 640, 480

# resolution used for display depth and skeleton images
PREVIEW_RESOLUTION = 320, 240

# time to wait after moving tilt:
TILT_WAITING = 1.35

# variable used to manage Kinect events in PyGame
KINECTEVENT = pygame.USEREVENT

# definition of limbs to be drawn
LEFT_ARM = (JointId.ShoulderCenter,
            JointId.ShoulderLeft,
            JointId.ElbowLeft,
            JointId.WristLeft,
            JointId.HandLeft)
RIGHT_ARM = (JointId.ShoulderCenter,
             JointId.ShoulderRight,
             JointId.ElbowRight,
             JointId.WristRight,
             JointId.HandRight)
LEFT_LEG = (JointId.HipCenter,
            JointId.HipLeft,
            JointId.KneeLeft,
            JointId.AnkleLeft,
            JointId.FootLeft)
RIGHT_LEG = (JointId.HipCenter,
             JointId.HipRight,
             JointId.KneeRight,
             JointId.AnkleRight,
             JointId.FootRight)
SPINE = (JointId.HipCenter,
         JointId.Spine,
         JointId.ShoulderCenter,
         JointId.Head)

# engine used to convert skeletal joints from 3D space to depth map space
skeleton_to_depth_image = nui.SkeletonEngine.skeleton_to_depth_image

# Kinects
kinects = [None, None, None, None]
# Kinect Cameras (useful to control motorized tilts)
kinect_cam = [None, None, None, None]

# Temp surface for loading depth data in (16-bit)
tmp_s = [pygame.Surface(DEPTH_RESOLUTION, 0, 16),
         pygame.Surface(DEPTH_RESOLUTION, 0, 16),
         pygame.Surface(DEPTH_RESOLUTION, 0, 16),
         pygame.Surface(DEPTH_RESOLUTION, 0, 16)]

# Temp surface for loading depth data in (8-bit)
tmp_s_8 = [pygame.Surface(DEPTH_RESOLUTION, 0, 8),
           pygame.Surface(DEPTH_RESOLUTION, 0, 8),
           pygame.Surface(DEPTH_RESOLUTION, 0, 8),
           pygame.Surface(DEPTH_RESOLUTION, 0, 8)]

# Surfaces to be used for drawing update_depth method
img_device_srf = [None, None, None, None]

# Surfaces to be used for drawing update_skel method
img_skel_srf = [None, None, None, None]

# Images that will contain depth maps in the GUI
img_device = [None, None, None, None]

# Image that will display skeleton (overlay on img_device)
img_device_skeleton = [None, None, None, None]

# flags to specify if depth maps and skeletons must be shown
display_depth = [True, True, True, True]
display_skeleton = [False, False, False, False]

# fonts
fontBig = None
fontSub = None

# container for widgets used in the GUI
widgets = [{}, {}, {}, {}]


def toggle_display_depth(index):
    display_depth[index] = not display_depth[index]

    if not display_depth[index]:
        img_device_srf[index].fill(THECOLORS["black"])


def toggle_display_skeleton(index):
    display_skeleton[index] = not display_skeleton[index]
    time.sleep(0.1)
    clean_skeleton(index)


def draw_skeleton_data(screen, pSkelton, positions, width=4):
    start = pSkelton.SkeletonPositions[positions[0]]

    for position in itertools.islice(positions, 1, None):
        next = pSkelton.SkeletonPositions[position.value]

        curstart = skeleton_to_depth_image(start,
                                           PREVIEW_RESOLUTION[0],
                                           PREVIEW_RESOLUTION[1])
        curend = skeleton_to_depth_image(next,
                                         PREVIEW_RESOLUTION[0],
                                         PREVIEW_RESOLUTION[1])

        pygame.draw.line(screen, THECOLORS["red"], curstart, curend, width)

        start = next


def clean_skeleton(index):
    img_skel_srf[index].fill(THECOLORS["black"])


def draw_skeleton(screen, skeletons, index):
    # clean the screen
    clean_skeleton(index)

    for index, skeleton_info in enumerate(skeletons):
        # test if the current skeleton is tracked or not
        if skeleton_info.eTrackingState == SkeletonTrackingState.TRACKED:
            # drawing the limbs
            draw_skeleton_data(screen, skeleton_info, SPINE)
            draw_skeleton_data(screen, skeleton_info, LEFT_ARM)
            draw_skeleton_data(screen, skeleton_info, RIGHT_ARM)
            draw_skeleton_data(screen, skeleton_info, LEFT_LEG)
            draw_skeleton_data(screen, skeleton_info, RIGHT_LEG)

            return


def change_tilt_thread(slider, cam):
    """change the tilt angle of Kinect"""

    # disable the widget
    slider.disabled = True

    # move the tilt
    cam.set_elevation_angle(-slider.value)

    # wait before moving tilt again
    time.sleep(TILT_WAITING)

    # re-enable the widget
    slider.disabled = False


def change_tilt(index):
    t = threading.Thread(target=change_tilt_thread,
                         args=[widgets[index]['device_slider'],
                               kinect_cam[index]])
    t.daemon = True
    t.start()


def update_depth(frame, index):
    """update the image"""
    if display_depth[index]:
        frame.image.copy_bits(tmp_s[index]._pixels_address)
        arr2d = (pygame.surfarray.pixels2d(tmp_s[index]) >> 7) & 255
        pygame.surfarray.blit_array(tmp_s_8[index], arr2d)

        pygame.transform.scale(tmp_s_8[index],
                               PREVIEW_RESOLUTION,
                               img_device_srf[index])


def update_skel(frame, index):
    if display_skeleton[index]:
        draw_skeleton(img_skel_srf[index], frame.SkeletonData, index)


def start_tango(index):
    device_name = "eras-" + str(widgets[index]['radio_device_group'].value)
    kinect = kinects[index]

    tracker_thread = threading.Thread(target=Tracker,
                                      args=(device_name, kinect, None, None))
    tracker_thread.daemon = True
    tracker_thread.start()

    disable_kinect_manager_gui_after_start_tango(index)


def disable_kinect_manager_gui_after_start_tango(index):
    widgets[index]['button_start_tango'].disabled = True
    widgets[index]['radio_device_group'].disabled = True


def activate_button_start_tango(index):
    widgets[index]['button_start_tango'].disabled = False


def draw_gui_kinect(title, index, lo, x, y):
    title = gui.Label(title, font=fontBig)
    lo.add(title, x, y)

    w = widgets[index]

    radio_device_table = gui.Table()
    w['radio_device_group'] = gui.Group()
    w['radio_device_eras1'] = gui.Radio(w['radio_device_group'], 1)
    w['radio_device_eras2'] = gui.Radio(w['radio_device_group'], 2)
    w['radio_device_eras3'] = gui.Radio(w['radio_device_group'], 3)
    w['radio_device_eras4'] = gui.Radio(w['radio_device_group'], 4)
    w['radio_device_group'].connect(gui.CHANGE,
                                    activate_button_start_tango,
                                    index)
    w['radio_device_eras1_lbl'] = gui.Label(" eras-1 ", font=fontSub)
    w['radio_device_eras2_lbl'] = gui.Label(" eras-2 ", font=fontSub)
    w['radio_device_eras3_lbl'] = gui.Label(" eras-3 ", font=fontSub)
    w['radio_device_eras4_lbl'] = gui.Label(" eras-4 ", font=fontSub)
    radio_device_table.tr()
    radio_device_table.td(w['radio_device_eras1'])
    radio_device_table.td(w['radio_device_eras1_lbl'])
    radio_device_table.tr()
    radio_device_table.td(w['radio_device_eras2'])
    radio_device_table.td(w['radio_device_eras2_lbl'])
    radio_device_table.tr()
    radio_device_table.td(w['radio_device_eras3'])
    radio_device_table.td(w['radio_device_eras3_lbl'])
    radio_device_table.tr()
    radio_device_table.td(w['radio_device_eras4'])
    radio_device_table.td(w['radio_device_eras4_lbl'])
    lo.add(radio_device_table, x, y+22)

    w['button_start_tango'] = gui.Button("Start", font=fontSub)
    w['button_start_tango'].connect(gui.CLICK, start_tango, index)
    lo.add(w['button_start_tango'], x, y+95)

    table_display_depth_skeleton = gui.Table()
    w['cb_display_depth'] = gui.Switch(display_depth[index])
    w['cb_display_depth'].connect(gui.CHANGE, toggle_display_depth, index)
    w['label_display_depth'] = gui.Label("depth", align=-1, font=fontSub)
    table_display_depth_skeleton.tr()
    table_display_depth_skeleton.td(w['cb_display_depth'])
    table_display_depth_skeleton.td(w['label_display_depth'])
    w['cb_display_skeleton'] = gui.Switch(display_skeleton[index])
    w['cb_display_skeleton'].connect(gui.CHANGE,
                                     toggle_display_skeleton,
                                     index)
    w['label_display_skeleton'] = gui.Label("skeleton", align=-1, font=fontSub)
    table_display_depth_skeleton.tr()
    table_display_depth_skeleton.td(w['cb_display_skeleton'])
    table_display_depth_skeleton.td(w['label_display_skeleton'])
    lo.add(table_display_depth_skeleton, x, y+120)

    w['device_slider'] = gui.VSlider(value=0, min=-27, max=27, size=32,
                                     width=16, height=220)
    w['device_slider'].connect(pygame.locals.MOUSEBUTTONUP, change_tilt, index)
    lo.add(w['device_slider'], x+75, y+20)

    lo.add(img_device[index], x+95, y)
    lo.add(img_device_skeleton[index], x+95, y)

    toggle_kinect_manager_gui(index)


def toggle_kinect_manager_gui(index):
    for name, widget in widgets[index].iteritems():
        widget.disabled = not widget.disabled

    widgets[index]['button_start_tango'].disabled = True

    radio_group = widgets[index]['radio_device_group']
    if not radio_group.disabled and radio_group.value is not None:
        widgets[index]['button_start_tango'].disabled = False


def main():
    # First of all, check if there is a sensor plugged in
    if nui._NuiGetSensorCount() < 1:
        sys.stderr.write("Error: no Kinect sensors available\n")
        return 1

    # Initialize Everything
    pygame.init()
    pygame.font.init()
    global fontBig, fontSub
    fontBig = pygame.font.SysFont("", 22, True)
    fontSub = pygame.font.SysFont("", 20)

    # initialization of images and their surfaces
    for index in range(4):
        img_device_srf[index] = pygame.Surface(PREVIEW_RESOLUTION, 0, 8)
        img_skel_srf[index] = pygame.Surface(PREVIEW_RESOLUTION)
        img_skel_srf[index].set_colorkey(THECOLORS["black"])
        img_device[index] = gui.Image(img_device_srf[index])
        img_device_skeleton[index] = gui.Image(img_skel_srf[index])

    for img in img_device_srf:
        img.set_palette(tuple([(i, i, i) for i in range(256)]))

    screen = pygame.display.set_mode(SCREEN_SIZE)
    pygame.display.set_caption('Kinect tracker manager')

    # create GUI object
    this_gui = gui.App()

    # layout using document
    lo = gui.Container(width=SCREEN_SIZE[0], height=SCREEN_SIZE[1])

    # draw the GUI
    draw_gui_kinect("KINECT A", 0, lo, 5, 5)
    draw_gui_kinect("KINECT B", 1, lo, 435, 5)
    draw_gui_kinect("KINECT C", 2, lo, 5, 260)
    draw_gui_kinect("KINECT D", 3, lo, 435, 260)

    # add layout to the GUI
    this_gui.init(lo)

    global kinects
    global kinect_cam
    for i in range(len(kinects)):
        # init all Kinects
        try:
            kinects[i] = Runtime(RuntimeOptions.uses_depth_and_player_index |
                                 RuntimeOptions.uses_skeletal_tracking,
                                 i)

            kinects[i].depth_stream.open(nui.ImageStreamType.depth, 2,
                                          nui.ImageResolution.resolution_640x480,
                                          nui.ImageType.depth_and_player_index)
            kinects[i].depth_frame_ready += lambda f, i=i: update_depth(f, i)


            kinects[i].skeleton_engine.enabled = True
            kinects[i].skeleton_frame_ready += lambda f, i=i: update_skel(f, i)

            kinect_cam[i] = nui.Camera(kinects[i])

            # set the right tilt angle
            widgets[i]['device_slider'].value = -kinect_cam[i].elevation_angle

            toggle_kinect_manager_gui(i)
        except:
            kinects[i] = None

    # Main Loop
    while True:

        # Handle Input Events
        for event in pygame.event.get():
            if event.type == QUIT:
                return
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                return

            # pass event to gui
            this_gui.event(event)

        # clear background
        screen.fill((250, 250, 250))

        # Draw GUI
        try:
            this_gui.paint(screen)
        except pygame.error as e:
            pass

        pygame.display.flip()

if __name__ == '__main__':
    main()
