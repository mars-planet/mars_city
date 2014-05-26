import cv2
from .managers import WindowManager, CaptureManager
from .depth import DepthTracker
from .object_tracker import ObjectTrackerManager


class Cameo(object):

    def __init__(self, left_channel=0, right_channel=1):

        self.window_manager = WindowManager('Debug Window', self.onKeypress)

        # Capture Video Streams for the left and right cameras
        self.left_capture_manager = CaptureManager(
            cv2.VideoCapture(left_channel), True)

        self.right_capture_manager = CaptureManager(
            cv2.VideoCapture(right_channel), True)

        self.object_tracker_manager = ObjectTrackerManager(
            self.left_capture_manager)

    def start(self, device):
        """ Run `start` from Tango """
        self.device = device

        # Start Video Stream Loop
        self.run()

    def run(self):
        """Run the main loop."""
        self.window_manager.createWindow()
        while self.window_manager.isWindowCreated:

            self.left_capture_manager.enterFrame()
            left_frame = self.left_capture_manager.frame

            self.right_capture_manager.enterFrame()
            right_frame = self.right_capture_manager.frame

            # Compute disparity
            disparity_frame=DepthTracker.computeDisparity(left_frame,right_frame, ndisparities=16, SADWindowSize=25);

            # Display disparity map
            x=0
            y=0
            w=100
            h=100
            cv2.rectangle(disparity_frame,(x,y),(x+w,y+h),(0,255,0))
            self.window_manager.show(disparity_frame)

            self.left_capture_manager.exitFrame()
            self.right_capture_manager.exitFrame()
            self.window_manager.process_events()

    def onKeypress(self, keycode):
        """Handle a keypress.

        space  -> Take a screenshot.
        tab    -> Start/stop recording a screencast.
        escape -> Quit.

        """

        if keycode == 32:  # space
            self._captureManager.writeImage('screenshot.png')
        elif keycode == 9:  # tab
            if not self._captureManager.isWritingVideo:
                self._captureManager.startWritingVideo(
                    'screencast.avi')
            else:
                self._captureManager.stopWritingVideo()
        elif keycode == 27:  # escape
            self.window_manager.destroyWindow()
