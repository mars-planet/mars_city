import cv2
import numpy
import time


class CaptureManager(object):

    def __init__(self, capture, channel=0):
        
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
    def is_writing_image(self):
        return self.image_filename is not None

    @property
    def is_writing_video(self):
        return self.video_filename is not None

    def enter_frame(self):
        """Capture the next frame, if any."""

        # But first, check that any previous frame was exited.
        assert not self.entered_frame, \
            'previous enter_frame() had no matching exit_frame()'

        if self.capture is not None:
            self.entered_frame = self.capture.grab()

    def exit_frame(self):
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
        if self.is_writing_image:
            cv2.imwrite(self.image_filename, self._frame)
            self.image_filename = None

        # Write to the video file, if any.
        self._write_video_frame()

        # Release the frame.
        self._frame = None
        self.entered_frame = False

    def write_image(self, filename):
        """Write the next exited frame to an image file."""
        self.image_filename = filename

    def start_writing_video(
            self, filename,
            encoding=cv2.cv.CV_FOURCC('I', '4', '2', '0')):
        """Start writing exited frames to a video file."""
        self.video_filename = filename
        self.video_encoding = encoding

    def stop_writing_video(self):
        """Stop writing exited frames to a video file."""
        self.video_filename = None
        self.video_encoding = None
        self.video_writer = None

    def _write_video_frame(self):

        if not self.is_writing_video:
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

    def __init__(self, windowName, keypress_callback=None):
        self.keypress_callback = keypress_callback

        self._windowName = windowName
        self._is_window_created = False

    @property
    def is_window_created(self):
        return self._is_window_created

    def create_window(self):
        cv2.namedWindow(self._windowName)
        self._is_window_created = True

    def show(self, frame):
        cv2.imshow(self._windowName, frame)

    def destroy_window(self):
        cv2.destroyWindow(self._windowName)
        self._is_window_created = False

    def process_events(self):
        keycode = cv2.waitKey(1)
        if self.keypress_callback is not None and keycode != -1:
            # Discard any non-ASCII info encoded by GTK.
            keycode &= 0xFF
            self.keypress_callback(keycode)
