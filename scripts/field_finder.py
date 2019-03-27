#!/usr/bin/env python
import numpy as np
import matplotlib as plt

from skimage.feature import peak_local_max
from scipy import ndimage as ndi
import matplotlib.pyplot as plt

class FieldFinder:

    def __init__(self, lidar_obj):
        self.lidar_obj = lidar_obj

    def find_field(self, points):
        accumulator, thetas, rhos = self._get_hough_transform(points)
        paralell_lines = self._find_2_paralell_lines_from_hough_transform(accumulator, thetas, rhos)

        if len(paralell_lines) != 0:
            #print(paralell_lines)
            print("Distance between two most prominent paralel lines %f" % abs(paralell_lines[0]-paralell_lines[3]))

        return paralell_lines


    def _get_hough_transform(self, points):
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

        return accumulator, thetas, rhos


    def _find_2_paralell_lines_from_hough_transform(self, accumulator, thetas, rhos):
        # Find first peak:
        idx = np.argmax(accumulator)
        p1_iy = idx / accumulator.shape[1]
        p1_ix = idx % accumulator.shape[1]

        # Find peaks of the transform
        hough_max_coordinates = peak_local_max(accumulator, min_distance=10, threshold_abs=2)
        hough_max_values = accumulator[hough_max_coordinates[:, 0], hough_max_coordinates[:, 1]]

        # Add intensity to coordinate array
        hough_max_coordinates_and_intensity = np.c_[hough_max_coordinates, hough_max_values]

        # Combine nearby points
        hough_max_coordinates_combined = self._combine_close_points(hough_max_coordinates_and_intensity)


        if len(hough_max_coordinates_combined) == 0:
            return np.array([])# if only on main line found -> there are no paralell lines

        # Sort combined points based on their angle value
        hough_max_coordinates_combined = hough_max_coordinates_combined[hough_max_coordinates_combined[:, -2].argsort()]
        paralell_lines = self._get_points_with_similar_angle(hough_max_coordinates_combined)

        if len(paralell_lines) > 0:
            #print(paralell_lines)
            # Sort found paralell lines on their combined intensity, decending order
            intesities = paralell_lines[:,2] + paralell_lines[:,5]
            paralell_lines = paralell_lines[(intesities).argsort()]


            return  paralell_lines[-1]

        #print(hough_max_coordinates)
        #print("after")
        #print(hough_max_coordinates_combined)
        #print(hough_max_values)

        #rho = rhos[idx / accumulator.shape[1]]
        #theta = thetas[idx % accumulator.shape[1]]
        #print("r={0:.2f}, theta={1:.0f}".format(rho, np.rad2deg(theta)))

        #i = hough_max_coordinates_combined.astype(int)
        #accumulator[i[:, 0], i[:, 1]] = 50
        #plt.matshow(accumulator)
        #plt.show()

        return np.array([])


    def _get_points_with_similar_angle(self, points, max_angle_difference = 2, min_distance = 5):
        # points: [[p1x, p1y, p1int], ...], sorted on p1y value (angle)
        # min_distance: minimum distance between paralell lines in index units
        # Returns: [[p1x, p1y, p1int, p2x, p2y, p2int], ...]

        list_of_paralell_lines = []
        i = 0
        while i < len(points) - 1:
            angle_diff = abs(points[i,1] - points[i + 1,1])
            distance = abs(points[i,0] - points[i + 1,0])

            if angle_diff <= max_angle_difference and distance > min_distance:
                list_of_paralell_lines.append([points[i,0], points[i,1],points[i,2],
                                               points[i + 1,0], points[i + 1,1], points[i + 1,2]])
            i += 1

        return np.array(list_of_paralell_lines)




    def _combine_close_points(self, points):
        # Combines points that are closer than treshold and creates a new list of those.
        # Input numpy array [[x,y, intensity],...]

        combine_objects_closer_than = 3  # index units
        points_combined = []
        point_list = points.tolist() # convert to python list

        while len(point_list) > 1:
            close_objects_index = []
            close_objects_index.append(0)

            p1 = point_list[0]
            i_second = 1

            # Find close objects
            while i_second < len(point_list):
                p2 = point_list[i_second]
                distance_between_points = np.sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

                if distance_between_points <= combine_objects_closer_than:
                    close_objects_index.append(i_second)

                i_second += 1

            # Calculate average position of found nearby objects
            avg_x = 0
            avg_y = 0
            intensity_sum = 0

            for i in close_objects_index:
                # print("i:%d  x%f y%f" % (i, float(list_of_obj[i][0]),float(list_of_obj[i][1])))
                weight = 1 / float(len(close_objects_index))
                # print(weight)
                avg_x += weight * point_list[i][0]
                avg_y += weight * point_list[i][1]
                intensity_sum +=  point_list[i][2]

            # Add found object to the new object array
            points_combined.append([avg_x, avg_y, intensity_sum])

            # Pop used objects from the orginal array
            close_objects_index.sort(reverse=True)
            for i in close_objects_index:
                point_list.pop(i)


        return np.array(points_combined)