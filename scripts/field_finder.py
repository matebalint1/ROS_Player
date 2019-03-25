import numpy as np
import matplotlib as plt

from skimage.feature import peak_local_max
from scipy import ndimage as ndi
import matplotlib.pyplot as plt

class FieldFinder:

    def __init__(self, lidar_obj):
        self.lidar_obj = lidar_obj




    def get_hough_transform(self, points):
        # Calculates hough transform of the input points.
        # Points are a numpy array.
        # Returns 2d array in hough space.

        # Rho and Theta ranges
        thetas = np.deg2rad(np.arange(-90.0, 90.0))  # determines  angle resolution
        diag_len = 6  # m  # max_dist
        rhos = np.linspace(-diag_len, diag_len, num=120)  # determines radius resolution

        # Cache some resuable values
        cos_t = np.cos(thetas)
        sin_t = np.sin(thetas)
        num_thetas = len(thetas)

        # Hough accumulator array of theta vs rho
        accumulator = np.zeros((len(rhos), num_thetas), dtype=np.uint64)

        points_np = np.array(points)

        # Vote in the hough accumulator
        t_idx = np.arange(num_thetas)
        for i in range(len(points_np)):
            x = points_np[i, 0]
            y = points_np[i, 1]

            step_size_in_rho = 2.0 * diag_len / float(len(rhos))
            rho = np.rint((cos_t * x + sin_t * y) / step_size_in_rho).astype(int) + len(rhos) / 2
            rho = np.maximum(np.minimum(rho, len(rhos) - 1), 0)
            accumulator[rho, t_idx] += 1

        # self.find_2_paralell_lines_from_hough_transform(accumulator, thetas, rhos)
        return accumulator, thetas, rhos


    def find_2_paralell_lines_from_hough_transform(self, accumulator, thetas, rhos):
        # Find first peak:
        idx = np.argmax(accumulator)
        p1_iy = idx / accumulator.shape[1]
        p1_ix = idx % accumulator.shape[1]

        hough_max_coordinates = peak_local_max(accumulator, min_distance=10, threshold_abs=2)
        hough_max_values = accumulator[hough_max_coordinates[:, 0], hough_max_coordinates[:, 1]]

        print(hough_max_coordinates)
        print(hough_max_values)

        rho = rhos[idx / accumulator.shape[1]]
        theta = thetas[idx % accumulator.shape[1]]
        print("r={0:.2f}, theta={1:.0f}".format(rho, np.rad2deg(theta)))

        accumulator[hough_max_coordinates[:, 0], hough_max_coordinates[:, 1]] = 50
        plt.matshow(accumulator)
        plt.show()