import cv2
import numpy as np
from sklearn.cluster import MiniBatchKMeans
import SimpleCV


class DepthTrackerManager(object):

    def __init__(self):
        self._disparity_map = None
        self._left_image = None
        self._right_image = None

    @property
    def disparity_map(self):
        rgb_disparity_frame = cv2.cvtColor(
            self._disparity_map,
            cv2.COLOR_GRAY2RGB)
        return rgb_disparity_frame

    def compute_stereo_distance(self, metric="euclidean"):
        """Compute the distance between the left and right image frames.
        Possible values for the metric keyword are: `euclidean` and `manhattan`.
        """

        if self._disparity_map is None:
            raise ValueError("Must compute the disparity map first!")

        difference_image = self._left_image - self._right_image

        # Calculate distance between the two images
        if metric == "euclidean":
            distance = np.linalg.norm(difference_image)
        elif metric == "manhattan":
            distance = sum(abs(difference_image))
        else:
            raise ValueError("Supplied distance metric is not supported!")

        return distance

    def compute_histogram(self, bins=10):
        """Returns the histogram of the disparity map. This gives us a general
        idea of the current amount of near objects on screen."""

        if self._disparity_map is None:
            raise ValueError("Must compute the disparity map first!")

        # Compute the histogram
        histogram = np.histogram2d(
            x=self._disparity_map[:, 1], y=self._disparity_map[:, 0],
            bins=bins)

        return histogram

    def objects_in_proximity(self, min_member_size, n_clusters=10,
                             return_images=False):
        """Returns an array of object locations
        on the disparity map relative to the specified distance

        Set the return_images to True if you want images of the actual objects
        """

        if self._disparity_map is None:
            raise ValueError("Must compute the disparity map first!")

        # Find the contours on the disparity map to find nearby objects
        image = SimpleCV.Image(self._disparity_map)
        image.show()

        blobs = image.binarize(200).findBlobs()
        blobs = [c for c in blobs if c.area() > min_member_size]

        # Does the user want the cropped images?
        if return_images:

            contour_polygons = map(lambda x: x.contour(), blobs)
            cropped_images = []
            for polygon in contour_polygons:

                # This is the image which we will crop the polygon from
                base_image = self._left_image

                # Create the mask
                mask = np.zeros(base_image.shape, dtype=np.uint8)
                roi_corners = np.array([polygon], dtype=np.int32)
                cv2.fillPoly(mask, roi_corners, (255, 255, 255))

                # Apply the mask
                masked_image = cv2.bitwise_and(base_image, mask)
                cropped_images.append(masked_image)

            return (cropped_images,blobs)

        return blobs

    def compute_disparity(self, left_image, right_image,
                          ndisparities=16, SADWindowSize=25):
        """Compute the disparity image, given the left and right image."""

        stereo = cv2.StereoBM(
            cv2.STEREO_BM_BASIC_PRESET,
            ndisparities=ndisparities,
            SADWindowSize=SADWindowSize)

        self._left_image = left_image
        # Convert the given images to grayscale
        gray_left = cv2.cvtColor(left_image,
                                 cv2.COLOR_BGR2GRAY)

        self._right_image = right_image
        gray_right = cv2.cvtColor(right_image,
                                  cv2.COLOR_BGR2GRAY)

        # Compute stereo disparity image
        disparity_map = (
            stereo.compute(gray_left, gray_right).astype(np.float32) - 32
        ) / 25.0

        self._disparity_map = disparity_map
