import cv2
from managers import WindowManager, CaptureManager
from depth import DepthTracker
import Queue

class Cameo(object):
    
    def __init__(self):


        self._windowManager = WindowManager('Preview Window',self.onKeypress)

        # Capture Video Streams for the left and right cameras
        self._leftCaptureManager = CaptureManager(
            cv2.VideoCapture(0), self._windowManager, True, channel = 1)

        self._rightCaptureManager = CaptureManager(
            cv2.VideoCapture(1), self._windowManager, True, channel = 2)


    def start(self, device):
        """ Run `start` from Tango """

        #Start Video Stream Loop
        self.run()

    def run(self):
        """Run the main loop."""
        self._windowManager.createWindow()
        while self._windowManager.isWindowCreated:

            self._leftCaptureManager.enterFrame()
            left_frame = self._leftCaptureManager.frame

            self._rightCaptureManager.enterFrame()
            right_frame = self._rightCaptureManager.frame

            # depth_frame=DepthTracker.computeDepthField(left_frame,right_frame);  uncomment once we get two actual webcams (minoru)  
                    
            self._leftCaptureManager.exitFrame()
            self._rightCaptureManager.exitFrame()
            self._windowManager.processEvents()
    
    def onKeypress(self, keycode):
        """Handle a keypress.
        
        space  -> Take a screenshot.
        tab    -> Start/stop recording a screencast.
        escape -> Quit.
        
        """

        if keycode == 32: # space
            self._captureManager.writeImage('screenshot.png')
        elif keycode == 9: # tab
            if not self._captureManager.isWritingVideo:
                self._captureManager.startWritingVideo(
                    'screencast.avi')
            else:
                self._captureManager.stopWritingVideo()
        elif keycode == 27: # escape
            self._windowManager.destroyWindow()