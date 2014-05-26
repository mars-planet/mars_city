import cv2
import numpy as np


class DepthTrackerManager(object):

    def __init__(self):
        self._disparity_map = None

    @property
    def disparity_map(self):
        rgb_disparity_frame = cv2.cvtColor(self._disparity_map,cv2.COLOR_GRAY2RGB)
        return rgb_disparity_frame

    def objects_in_proximity(self,distance=None):
        """ Returns an array of object locations on the disparity map"""
        
        if not distance: raise("Must specify a distance to retrieve objects from!")
        if self._disparity_map==None: raise("Must compute the disparity map first! [ compute_disparity ] ")

        contours,hierarchy = cv2.findContours(self._disparity_map,2,1)
        return contours

    def compute_disparity(self, left_image, right_image, ndisparities=16, SADWindowSize=25):
        """Compute the depth image, given the left and right image."""

        stereo = cv2.StereoBM(
            cv2.STEREO_BM_BASIC_PRESET,
            ndisparities=ndisparities,
            SADWindowSize=SADWindowSize)

        gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        # Compute stereo disparity image
        disparity_map=(stereo.compute(gray_left,gray_right).astype(np.float32) - 32) / 25.0
        self._disparity_map=disparity_map