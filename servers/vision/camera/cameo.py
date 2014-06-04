import cv2
from .managers import WindowManager, CaptureManager
from .depth import DepthTrackerManager
from .object_tracker import ObjectTrackerManager
from .object_recognizer import ObjectRecognizerManager


class Cameo(object):

    def __init__(self, left_channel=0, right_channel=1):

        self.window_manager = WindowManager('Debug Window',
                                            self.on_key_press)

        # Capture Video Streams for the left and right cameras
        self.left_capture_manager = CaptureManager(
            cv2.VideoCapture(left_channel), mirrored=True)
        self.right_capture_manager = CaptureManager(
            cv2.VideoCapture(right_channel), mirrored=True)

        self.object_tracker_manager = ObjectTrackerManager(
            self.left_capture_manager)
        self.depth_tracker_manager = DepthTrackerManager()

        self.object_recognition_manager = ObjectRecognizerManager()

    def start(self, device):
        """Run `start` from Tango"""
        self.device = device

        # Start Video Stream Loop
        self.run()

    def run(self):
        """Run the main loop."""
        self.window_manager.create_window()
        while self.window_manager.is_window_created:

            self.left_capture_manager.enter_frame()
            left_frame = self.left_capture_manager.frame

            self.right_capture_manager.enter_frame()
            right_frame = self.right_capture_manager.frame

            # Compute disparity
            self.depth_tracker_manager.compute_disparity(
                left_frame, right_frame,
                ndisparities=16, SADWindowSize=25)
            disparity_frame = self.depth_tracker_manager.disparity_map

            # Find nearby objects on screen
            objects, blobs = self.depth_tracker_manager.objects_in_proximity(
                min_member_size=400, return_images=True)

            blob_centroids = map(lambda p: p.centroid(), blobs)

            for n, image in enumerate(objects):
                centroid = blob_centroids[n]

                self.object_recognition_manager.add_object(image)

                self.window_manager.draw_circle(
                    left_frame, x=int(
                        centroid[0]), y=int(
                        centroid[1]), radius=10)
                self.window_manager.draw_text(
                    left_frame, "Object", x=int(
                        centroid[0]), y=int(
                        centroid[1]))

            if self.object_recognition_manager.amount_of_images > 4:
                self.object_recognition_manager.recognize_object(image)

            # Display left frame
            self.window_manager.show(left_frame)

            self.left_capture_manager.exit_frame()
            self.right_capture_manager.exit_frame()
            self.window_manager.process_events()

    def on_key_press(self, keycode):
        """Handle a keypress.

        space  -> Take a screenshot.
        tab    -> Start/stop recording a screencast.
        escape -> Quit.

        """

        if keycode == 32:  # space
            self.right_capture_manager.writeImage('screenshot.png')
        elif keycode == 9:  # tab
            if not self.right_capture_manager.is_writing_video:
                self.right_capture_manager.start_writing_video(
                    'screencast.avi')
            else:
                self.right_capture_manager.stop_writing_video()
        elif keycode == 27:  # escape
            self.window_manager.destroy_window()
