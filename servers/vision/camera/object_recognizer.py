import numpy as np
import cv2
from sklearn.cluster import MiniBatchKMeans, DBSCAN
from sklearn.semi_supervised import LabelSpreading
from sklearn.decomposition import PCA
import pylab as pl


class ObjectRecognizerManager(object):

    def __init__(self):

        # TODO: Change this to an actual database [Redis array store?]
        self.IMAGE_STORE = []

        # Parameters
        self.dbscan_params = {'eps': 2000}

    @property
    def amount_of_images(self):
        return len(self.IMAGE_STORE)

    def estimate_objects_memorized(self):
        """A rough estimate of the number of objects which we have learned"""
        
        # Use the elbow method to estimate K (number of clusters)
        hog_vectors = PCA(n_components=2).fit_transform(np.vstack(self.IMAGE_STORE))
        # Wait until cluster inertia decreases linearly.
        # Meaning, Let's wait for 2nd derivative = ~0
        def f(x):
            kmeans = MiniBatchKMeans(n_clusters=x)
            kmeans.fit(hog_vectors)
            print kmeans.inertia_
            return kmeans.inertia_

        def g(x):
            return f(x) - f(x+1)

        def optimize(func, initial_x=0, max_iter = 10):
            
            cur_x=initial_x
            cur_iter = 0
            past_estimate = None
            while True:

                estimate = func(x=cur_x)

                if past_estimate:
                    diff_estimate = past_estimate - estimate
                    print diff_estimate
                past_estimate = estimate

                cur_x=-1
                cur_iter+=1
                if cur_iter>max_iter:
                    break
            return cur_x

        # Start at standard approximation
        n_images = self.amount_of_images
        initial_estimate = int(np.sqrt(n_images/2))

        # Start optimizing
        k_clusters = optimize(g, initial_x=initial_estimate, max_iter=10)
        return k_clusters

    def _pre_process_new_image(self, image, bin_n=100):
        """Find a histogram of oriented gradients vector representation of
        a new image"""

        gx = cv2.Sobel(image, cv2.CV_32F, 1, 0)
        gy = cv2.Sobel(image, cv2.CV_32F, 0, 1)
        mag, ang = cv2.cartToPolar(gx, gy)

        # Quantizing binvalues in (0...16)
        bins = np.int32(bin_n * ang / (2 * np.pi))

        # Divide to 4 sub-squares
        bin_cells = (bins[:10, :10],
                     bins[10:, :10],
                     bins[:10, 10:],
                     bins[10:, 10:])
        mag_cells = mag[:10, :10], mag[10:, :10], mag[:10, 10:], mag[10:, 10:]
        hists = [np.bincount(b.ravel(), m.ravel(), bin_n)
                 for b, m in zip(bin_cells, mag_cells)]
        hist = np.hstack(hists)
        return hist

    def summarize(self, strategy="dbscan", n_clusters=None):
        """Storing millions of images of the same object is not efficent.
        This function will reduce the number of datapoints, trying keeping only
        the ones which are most significant."""
        pca = PCA(n_components=2)
        pca.fit(np.vstack(self.IMAGE_STORE))
        X_reduced = pca.transform(np.vstack(self.IMAGE_STORE))
        if strategy == "dbscan":
            
            dbscan = DBSCAN(**self.dbscan_params).fit(X_reduced)

            
            core_points = dbscan.components_
            unique = np.unique(dbscan.labels_)

            # Now we shall obtain the medoids of each cluster
            kmeans = MiniBatchKMeans(n_clusters=len(unique)).fit(core_points)
            cluster_centers = kmeans.cluster_centers_
            medoids = []
            for centroid in cluster_centers:
                dist=np.abs(core_points-centroid)

                # Point with the minimum distance from centroid
                min_dist_index = np.argmin(dist, axis=1)
                medoids.append(core_points[min_dist_index])


            self.IMAGE_STORE = medoids

            d = np.vstack(self.IMAGE_STORE)
            pl.clf()
            pl.scatter(d[:, 0], d[:, 1])
            pl.savefig("visualize_objects.png")
            
        else:
            raise ValueError("Summarization method not available!")

    def add_object(self, image_object):
        """Add a given image of an object to our database"""

        # Vectorize the given image
        image_vector = self._pre_process_new_image(image_object)

        # Store image in database
        self.IMAGE_STORE.append(image_vector)

    def recognize_object(self, image_object, sample_labelled_vectors=None):
        """Returns the cluster association of the given object image. If the
        sample_vectors keyword parameter is specified, the most appropriate
        label will be returned.
        """

        image_vector = self._pre_process_new_image(image_object)

        # Has the user specified any semi-supervision?
        if sample_labelled_vectors is not None:

            # Create the semi-labeled data
            labels = -np.ones(len(self.IMAGE_STORE))
            for labelled_vector in enumerate(sample_labelled_vectors):
                labels[labelled_vector["index"]] = labelled_vector["label"]

            # Construct a semi-supervised model using nearest neighbors
            label_spread = label_propagation.LabelSpreading(
                kernel='knn',
                alpha=1.0)
            label_spread.fit(self.IMAGE_STORE, labels)

            # Use our semi-supervised model to predict the image class
            predicted_label = label_spread.predict(image_vector)

            return predicted_label

        # Initialize and learn the currently stored images

        pca = PCA(n_components=2)
        pca.fit(np.vstack(self.IMAGE_STORE))
        X_train = pca.transform(np.vstack(self.IMAGE_STORE))
        X_predict = pca.transform(image_vector)

        dbscan = DBSCAN(**self.dbscan_params)
        cluster_association = dbscan.fit_predict(X_train, image_vector)

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

        # TODO: associate each cluster to sample vectors

        return found_objects

    def find_example_points(self):
        """Finds examplar data points for each cluster"""

        # Train DBSCAN
        dbscan = DBSCAN()
        dbscan.fit(self.IMAGE_STORE)

        labels = dbscan.labels_
        unique, indices = np.unique(labels, return_index=True)

        # Remove the 'noise' example data point
        index = np.where(unique == -1)
        unique = np.delete(unique, index)
        indices = np.delete(indices, index)

        return np.vstack(unique, self.IMAGE_STORE[indices])
