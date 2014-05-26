import cv2
import numpy as np
from skimage import measure

class DepthTrackerManager(object):

    def __init__(self):
        self._disparity_map = None

    @property
    def disparity_map(self):
        rgb_disparity_frame = cv2.cvtColor(
            self._disparity_map,
            cv2.COLOR_GRAY2RGB)
        return rgb_disparity_frame

    def objects_in_proximity(self, distance):
        """Returns an array of object locations
        on the disparity map relative to the specified distance
        """

        if self._disparity_map is None:
            raise ValueError("Must compute the disparity map first!")

        # Find the contours on the disparity map to find nearby objects
        contours = measure.find_contours(self._disparity_map, level=0.1,
            fully_connected='high', positive_orientation='high')

        # Filter out the small objects, keeping only the close ones
        contours = filter(lambda x: len(x)>distance, contours)

        return contours

    def compute_disparity(self, left_image, right_image,
                          ndisparities=16, SADWindowSize=25):
        """Compute the disparity image, given the left and right image."""

        stereo = cv2.StereoBM(
            cv2.STEREO_BM_BASIC_PRESET,
            ndisparities=ndisparities,
            SADWindowSize=SADWindowSize)

        gray_left = cv2.cvtColor(left_image,
                                 cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_image,
                                  cv2.COLOR_BGR2GRAY)

        # Compute stereo disparity image
        disparity_map = (
            stereo.compute(gray_left, gray_right).astype(np.float32) - 32
        ) / 25.0

        self._disparity_map = disparity_map
