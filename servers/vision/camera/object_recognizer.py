import numpy as np
import cv2
from sklearn.cluster import DBSCAN
from sklearn.semi_supervised import LabelSpreading

# TODO: save the clustering predictions so that we do not always have to `fit`
# TODO: Change this to an actual database [Redis array store?]
IMAGE_STORE = []


class ObjectRecognizerManager(object):

    def _pre_process_new_image(self, image):
        """Find a histogram of oriented gradients vector representation of
        a new image"""

        # Resize the given image to 32x32
        image = cv2.resize(image, (32, 32))

        gx = cv2.Sobel(img, cv2.CV_32F, 1, 0)
        gy = cv2.Sobel(img, cv2.CV_32F, 0, 1)
        mag, ang = cv2.cartToPolar(gx, gy)

        # Quantizing binvalues in (0...16)
        bins = np.int32(bin_n * ang / (2 * np.pi))

        # Divide to 4 sub-squares
        bin_cells = bins[
            :10, :10], bins[
            10:, :10], bins[
            :10, 10:], bins[
                10:, 10:]
        mag_cells = mag[:10, :10], mag[10:, :10], mag[:10, 10:], mag[10:, 10:]
        hists = [np.bincount(b.ravel(), m.ravel(),
                             bin_n) for b, m in zip(bin_cells, mag_cells)]
        hist = np.hstack(hists)
        return hist

    def summarize(self, strategy="DBSCAN"):
        """Storing millions of images of the same object is not efficent.
        This function will reduce the number of datapoints, trying keeping only
        the ones which are most significant."""

        if strategy == "DBSCAN":
            # Use DBSCAN to eliminate everything except for the core points

            dbscan = DBSCAN()
            dbscan.fit(IMAGE_STORE)
            core_sample_indices = dbscan.core_sample_indices_

            # Retain only the core points found by DBSCAN
            IMAGE_STORE = IMAGE_STORE[core_sample_indices]
        else:
            raise ValueError("Summarization method not available!")

    def add_object(self, image_object):
        """Add a given image of an object to our database"""

        # Vectorize the given image
        image_vector = self._pre_process_new_image(image_object)

        # Store image in database
        IMAGE_STORE.append(image_vector)

    def recognize_object(self, image_object, sample_labelled_vectors=None):
        """Returns the cluster association of the given object image. If the
        sample_vectors keyword parameter is specified, the most appropriate
        label will be returned.
        """

        image_vector = self._pre_process_new_image(image_object)

        # Has the user specified any semi-supervision?
        if sample_labelled_vectors is not None:

            # Create the semi-labeled data
            labels = -np.ones(len(IMAGE_STORE))
            for labelled_vector in enumerate(sample_labelled_vectors):
                labels[labelled_vector["index"]] = labelled_vector["label"]

            # Construct a semi-supervised model using nearest neighbors
            label_spread = label_propagation.LabelSpreading(
                kernel='knn',
                alpha=1.0)
            label_spread.fit(IMAGE_STORE, labels)

            # Use our semi-supervised model to predict the image class
            predicted_label = label_spread.predict(image_vector)

            return predicted_label

        # Initialize and learn the currently stored images
        dbscan = DBSCAN()
        dbscan.fit(IMAGE_STORE)
        cluster_association = dbscan.predict(image_vector)

        return cluster_association

    def find_objects(self, image, sample_labelled_vectors=None):
        """Find the known objects in a specified image. This function returns
        a dictionary containing the locations of known images in the following
        format:

            {"rock":[polygon1, polygon2, polygon3], "plant": [polygon4]}

        """

        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        example_points = self.find_example_points()

        # The found objects on screen
        found_objects = {}

        # Template match an examplar data point from each cluster
        for cluster_association, example_image in example_points:
            template = example_image
            found_objects[cluster_association] = []

            res = cv2.matchTemplate(image_gray, template, cv2.TM_CCOEFF_NORMED)
            threshold = 0.8
            loc = np.where(res >= threshold)
            for pt in zip(*loc[::-1]):
                found_objects[cluster_association].append(pt)

        # TODO: assoicate each cluster to sample vectors

        return found_objects

    def find_example_points(self):
        """Finds examplar data points for each cluster"""

        # Train DBSCAN
        dbscan = DBSCAN()
        dbscan.fit(IMAGE_STORE)

        labels = dbscan.labels_
        unique, indices = np.unique(labels, return_index=True)

        # Remove the 'noise' example data point
        index = np.where(unique == -1)
        unique = np.delete(unique, index)
        indices = np.delete(indices, index)

        return np.vstack(unique, IMAGE_STORE[indices])
