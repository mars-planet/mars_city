import numpy as np
import cv2


class ObjectTrackerManager(object):

    def __init__(self, capture_manager):

        self.capture_manager = capture_manager
        self.frame = capture_manager.frame

        self.selection = None
        self.tracking_state = False

    def set_object(self, x0, y0, x1, y1):
        """Set the image region of the object which should be tracked"""

        self.selection = (x0, y0, x1, y1)
        self.tracking_state = True

    def compute_camshift(self, frame):
        """Compute the Camera Shift for the Current Selection on the
        Given Frame (if any)"""

        self.frame = self.capture_manager.frame

        vis = self.frame.copy()
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv, np.array(
                (0., 60., 32.)), np.array(
                (180., 255., 255.)))

        if self.selection:
            x0, y0, x1, y1 = self.selection
            self.track_window = (x0, y0, x1 - x0, y1 - y0)
            hsv_roi = hsv[y0:y1, x0:x1]
            mask_roi = mask[y0:y1, x0:x1]

            # Calculate Object Histogram
            hist = cv2.calcHist([hsv_roi], [0],
                                mask_roi, [16], [0, 180])
            cv2.normalize(hist, hist, 0, 255,
                          cv2.NORM_MINMAX)
            self.hist = hist.reshape(-1)

            vis_roi = vis[y0:y1, x0:x1]
            cv2.bitwise_not(vis_roi, vis_roi)
            vis[mask == 0] = 0

        if self.tracking_state:
            self.selection = None
            prob = cv2.calcBackProject([hsv], [0],
                                       self.hist, [0, 180], 1)
            prob &= mask
            term_crit = (
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                10,
                1)
            track_box, self.track_window = cv2.CamShift(
                prob, self.track_window, term_crit)

            return track_box
