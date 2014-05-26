import cv2
import numpy as np


class DepthTracker(object):

    @staticmethod
    def compute_disparity(imgL, imgR, ndisparities=16, SADWindowSize=25):
        """Compute the depth image, given the left and right image."""

        stereo = cv2.StereoBM(
            cv2.STEREO_BM_BASIC_PRESET,
            ndisparities=ndisparities,
            SADWindowSize=SADWindowSize)

        gray_left = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

        # Compute stereo disparity image
        gray_disparity_frame=(stereo.compute(gray_left,gray_right).astype(np.float32) - 32) / 25.0

        # Convert the disparity image to RGB
        frame=cv2.cvtColor(gray_disparity_frame,cv2.COLOR_GRAY2RGB)
        
        return frame
