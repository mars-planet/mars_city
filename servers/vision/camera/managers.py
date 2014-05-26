import cv2
import numpy
import time


class CaptureManager(object):

    def __init__(self, capture, previewWindowManager=None,
                 shouldMirrorPreview=False, channel=0):
        
        self._frame = None
        self._channel = channel

        self.capture = capture
        self.entered_frame = False
        self.image_filename = None
        self.video_filename = None
        self.video_encoding = None
        self.video_writer = None

        self.start_time = None
        self.frame_elapsed = long(0)
        self._fpsEstimate = None

    @property
    def channel(self):
        return self._channel

    @channel.setter
    def channel(self, value):
        if self._channel != value:
            self._channel = value
            self._frame = None

    @property
    def frame(self):
        if self.entered_frame and self._frame is None:
            _, self._frame = self.capture.retrieve(channel=self.channel)
        return self._frame

    @property
    def isWritingImage(self):
        return self.image_filename is not None

    @property
    def isWritingVideo(self):
        return self.video_filename is not None

    def enterFrame(self):
        """Capture the next frame, if any."""

        # But first, check that any previous frame was exited.
        assert not self.entered_frame, \
            'previous enterFrame() had no matching exitFrame()'

        if self.capture is not None:
            self.entered_frame = self.capture.grab()

    def exitFrame(self):
        """Draw to the window. Write to files. Release the frame."""

        # Check whether any grabbed frame is retrievable.
        # The getter may retrieve and cache the frame.
        if self.frame is None:
            self.entered_frame = False
            return

        # Update the FPS estimate and related variables.
        if self.frame_elapsed == 0:
            self.start_time = time.time()
        else:
            timeElapsed = time.time() - self.start_time
            self._fpsEstimate = self.frame_elapsed / timeElapsed
        self.frame_elapsed += 1

        # Write to the image file, if any.
        if self.isWritingImage:
            cv2.imwrite(self.image_filename, self._frame)
            self.image_filename = None

        # Write to the video file, if any.
        self._writeVideoFrame()

        # Release the frame.
        self._frame = None
        self.entered_frame = False

    def writeImage(self, filename):
        """Write the next exited frame to an image file."""
        self.image_filename = filename

    def startWritingVideo(
            self, filename,
            encoding=cv2.cv.CV_FOURCC('I', '4', '2', '0')):
        """Start writing exited frames to a video file."""
        self.video_filename = filename
        self.video_encoding = encoding

    def stopWritingVideo(self):
        """Stop writing exited frames to a video file."""
        self.video_filename = None
        self.video_encoding = None
        self.video_writer = None

    def _writeVideoFrame(self):

        if not self.isWritingVideo:
            return

        if self.video_writer is None:
            fps = self.capture.get(cv2.cv.CV_CAP_PROP_FPS)
            if fps <= 0.0:
                # The capture's FPS is unknown so use an estimate.
                if self.frame_elapsed < 20:
                    # Wait until more frames elapse so that the
                    # estimate is more stable.
                    return
                else:
                    fps = self._fpsEstimate
            size = (int(self.capture.get(
                        cv2.cv.CV_CAP_PROP_FRAME_WIDTH)),
                    int(self.capture.get(
                        cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)))
            self.video_writer = cv2.VideoWriter(
                self.video_filename, self.video_encoding,
                fps, size)

        self.video_writer.write(self._frame)


class WindowManager(object):

    def __init__(self, windowName, keypressCallback=None):
        self.keypressCallback = keypressCallback

        self._windowName = windowName
        self._isWindowCreated = False

    @property
    def isWindowCreated(self):
        return self._isWindowCreated

    def createWindow(self):
        cv2.namedWindow(self._windowName)
        self._isWindowCreated = True

    def show(self, frame):
        cv2.imshow(self._windowName, frame)

    def destroyWindow(self):
        cv2.destroyWindow(self._windowName)
        self._isWindowCreated = False

    def processEvents(self):
        keycode = cv2.waitKey(1)
        if self.keypressCallback is not None and keycode != -1:
            # Discard any non-ASCII info encoded by GTK.
            keycode &= 0xFF
            self.keypressCallback(keycode)
