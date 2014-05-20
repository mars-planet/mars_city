import cv2

class DepthTracker(object):

    @staticmethod
    def computeDepthField(imgL, imgR):
        """Compute the depth image, given the left and right image."""
        window_size = 3
        min_disp = 16
        num_disp = 112-min_disp
        stereo = cv2.StereoSGBM(minDisparity = min_disp,
            numDisparities = num_disp,
            SADWindowSize = window_size,
            uniquenessRatio = 10,
            speckleWindowSize = 100,
            speckleRange = 32,
            disp12MaxDiff = 1,
            P1 = 8*3*window_size**2,
            P2 = 32*3*window_size**2,
            fullDP = False
        )

        return stereo.compute(imgL, imgR).astype(np.float32) / 16.0