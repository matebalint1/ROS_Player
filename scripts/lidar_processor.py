#!/usr/bin/env python
import numpy as np
from scipy.spatial import distance


class LidarProcessor:

    def __init__(self, min_r = 0.1, max_r = 5.9, max_distance_between_two_neighbor_points_in_an_object = 0.07):#0.07 good
        self._min_r = min_r # m
        self._max_r = max_r # m
        self._max_distance_between_two_neighbor_points_in_an_object = max_distance_between_two_neighbor_points_in_an_object

        self._max_object_size = 0.08 # m used for filtering too big objects
        self._min_object_size = 0.04 # m sed for filtering too small objects (-1 == not in use)
        self._size_of_scan_array = 100 # number of scans to be stored in array
        self._crop_angle = 90  # deg (total)

        self.scan_array = [] # contains objects from previous scans


    def add_scan(self, raw_scan, dx = 0, dy = 0, dyaw = 0, yaw_now = 0):

        found_objects = self._get_objects_from_laser_scan(raw_scan)
        filtered_objects = self._filter_objects(found_objects)

        total_odom = self._combine_scan_data_percise(filtered_objects, dx, dy, dyaw, yaw_now)
        return  filtered_objects, total_odom


    def get_scan_array(self):
        return self.scan_array

    def get_sum_of_scan_array(self):
        # for test purposes, return 1d version of all points
        return sum(self.scan_array, [])


    def _get_objects_from_laser_scan(self, raw_scan):
        # This method uses radial approach for finding objects from the scan
        # Returns an array that contains information about first and last point of a object and distance:
        # list_of_obj = [[p1_r, p1_angel, p2_r, p2_angle], ...]

        list_of_obj = []

        angle_increment = raw_scan.angle_increment  # Important, defines rotation direction of the lidar!
        scan_data = np.array(raw_scan.ranges)

        crop_angle_one_side = self._crop_angle/2 * 3.1415 / 180  # rad, used to crop the scan
        start_index = int(round(crop_angle_one_side / abs(angle_increment)))
        end_index = scan_data.size - start_index

        i = start_index
        start_index_of_object = start_index
        while i < end_index - 1:

            p1_r = scan_data[i]
            p1_angle = i * angle_increment

            if p1_r > self._max_r or p1_r < self._min_r:
                # if r is infinity or too close -> skip
                # reset param:
                i += 1
                start_index_of_object = i
                continue

            p2_r = scan_data[i + 1]
            p2_angle = (i + 1) * angle_increment

            distance_between_points = -1
            if p2_r < self._max_r and p2_r > self._min_r:
                # point 2 is not inf of zero
                distance_between_points = np.sqrt(
                    p1_r * p1_r + p2_r * p2_r - 2 * p1_r * p2_r * np.cos(p1_angle - p2_angle))

            if distance_between_points > self._max_distance_between_two_neighbor_points_in_an_object \
                    or distance_between_points == -1 \
                    or i == end_index - 2:
                # if distance_between_points is -1 the distance to the other will be large -> this point will be at the
                # end of a object.

                p_start_r = scan_data[start_index_of_object]
                p_start_angle = start_index_of_object * angle_increment

                list_of_obj.append([p_start_r, p_start_angle, p1_r, p1_angle])

                start_index_of_object = i + 1

            i += 1

        list_of_obj_cartesian = []

        for obj in list_of_obj:
            p1_x = obj[0] * np.cos(obj[1])
            p1_y = obj[0] * np.sin(obj[1])

            p2_x = obj[2] * np.cos(obj[3])
            p2_y = obj[2] * np.sin(obj[3])

            list_of_obj_cartesian.append([-p1_x, -p1_y, -p2_x, -p2_y])

        return list_of_obj_cartesian


    def _filter_objects(self, obj_array):
        # Removes too big or too small objects from array
        # Array format [[x1,y1,x2,y2], ...]

        filtered_obj = []

        for obj in obj_array:
            p1_x = obj[0]
            p1_y = obj[1]
            p2_x = obj[2]
            p2_y = obj[3]

            distance = np.sqrt((p1_x - p2_x) * (p1_x - p2_x) + (p1_y - p2_y) * (p1_y - p2_y))

            if distance <= self._max_object_size and distance >= self._min_object_size:
                avg_x = (p1_x + p2_x) / 2
                avg_y = (p1_y + p2_y) / 2

                filtered_obj.append([avg_x, avg_y])

        return filtered_obj



    def _combine_scan_data(self, new_scan, dx, dy, dyaw, yaw_now):
        # new_scan: 2d scan
        # scan_data_arr: list containing a arrays of scans (2D)
        # delta_pose: dx,dy,dyaw, in odom frame

        # Calculate dx, dy in base_link frame
        dyaw_frames = -yaw_now + 3.1415/2 # add 90 deg
        dy_base = -(dx * np.cos(dyaw_frames) - dy * np.sin(dyaw_frames))
        dx_base = -(dx * np.sin(dyaw_frames) + dy * np.cos(dyaw_frames))
        #print("base_link dx:%f, dy:%f" % (dx_base,dy_base))

        # Remove first item from scan array if len >=3
        if len(self.scan_array) >= self._size_of_scan_array:
            del self.scan_array[0]

        # Rotate old points
        for scan in self.scan_array:
            for point in scan:
                # Rotate point around origin
                x = point[0] * np.cos(-dyaw) - point[1] * np.sin(-dyaw)
                y = point[0] * np.sin(-dyaw) + point[1] * np.cos(-dyaw)

                # Linear translation
                x += dx_base
                y += dy_base

                # Save translated values
                point[0] = x
                point[1] = y

        # Combine old and new data
        self.scan_array.append(new_scan)


    def _combine_scan_data_percise(self, new_scan, dx, dy, dyaw, yaw_now):
        # Matches point structures to minimise odometry error
        # new_scan: 2d scan
        # scan_data_arr: list containing a arrays of scans (2D)
        # delta_pose: dx,dy,dyaw, in odom frame
        # Return list of matched translation: [dx, dy, dyaw]

        # Calculate dx, dy in base_link frame
        dyaw_odom_to_robot = -yaw_now + 3.1415/2 # add 90 deg
        dy_base = -(dx * np.cos(dyaw_odom_to_robot) - dy * np.sin(dyaw_odom_to_robot))
        dx_base = -(dx * np.sin(dyaw_odom_to_robot) + dy * np.cos(dyaw_odom_to_robot))
        #print("base_link dx:%f, dy:%f" % (dx_base,dy_base))

        # Remove first item from scan array if it is longer than desired
        if len(self.scan_array) >= self._size_of_scan_array:
            del self.scan_array[0]

        #"""
        # Rotate all old points according to the odometry information
        for scan in self.scan_array:
            for point in scan:
                # Rotate point around origin
                x = point[0] * np.cos(-dyaw) - point[1] * np.sin(-dyaw)
                y = point[0] * np.sin(-dyaw) + point[1] * np.cos(-dyaw)

                # Linear translation
                x += dx_base
                y += dy_base

                # Save translated values
                point[0] = x
                point[1] = y
        #"""

        # Match new and last scan to minimise odometry error
        x_diff_avg = 0
        y_diff_avg = 0
        angle_diff = 0
        """
        if len(self.scan_array) > 0:
            x_diff_avg, y_diff_avg, angle_diff = self._match_points(new_scan, self.scan_array[len(self.scan_array) - 1])
            #print([x_diff_avg, y_diff_avg, angle_diff])

            # Rotate all points according to the new odometry data
            for scan in self.scan_array:
                for point in scan:
                    # Rotate point around origin
                    x = point[0] * np.cos(angle_diff) - point[1] * np.sin(angle_diff)
                    y = point[0] * np.sin(angle_diff) + point[1] * np.cos(angle_diff)

                    # Linear translation
                    x += x_diff_avg
                    y += y_diff_avg

                    # Save translated values
                    point[0] = x
                    point[1] = y

        """
        # Combine old and new data
        if len(new_scan) > 0:
            self.scan_array.append(new_scan)

        return [[dx_base, dy_base, -dyaw], [x_diff_avg, y_diff_avg, angle_diff]]


    def _closest_point(self, point, points):
        closest_index = distance.cdist([point], points).argmin()
        dist = np.linalg.norm(point-points[closest_index])
        return closest_index, dist


    def _match_points(self, point_array1, point_array2):
        # Finds a translation to match point as close as possible to each other
        # Returns translation: dx, dy, dyaw and translated point_array2 around origo

        if(len(point_array1) == 0 or len(point_array1) == 0):
            return 0,0,0

        p1a = np.array(point_array1, dtype=float)
        p2a = np.array(point_array2, dtype=float)

        # Find corresponding points:
        matching_points = []
        i = 0
        while i < p1a.shape[0] and i < p2a.shape[0]:

            i2, dist = self._closest_point(p1a[i], p2a)
            if dist <= 0.5:
                matching_points.append([p1a[i, 0],
                                        p1a[i, 1],
                                        p2a[i2, 0],
                                        p2a[i2, 1]])
            i += 1

        matching_points_np = np.array(matching_points, dtype=float)

        if matching_points_np.shape[0] == 0:
            # no matching point found
            return 0,0,0

        # Calculate rotation difference:
        angle_diff = np.arctan2(matching_points_np[:,1], matching_points_np[:,0]) - \
                     np.arctan2(matching_points_np[:,3], matching_points_np[:,2])

        angle_diff = np.sort(angle_diff)
        #print(angle_diff)
        crop_index = int(round(angle_diff.size * 0.2))  # crop data 20 % of largest and smallest values
        yaw_diff = np.average(angle_diff[crop_index : angle_diff.size - crop_index])
        #print("mean %f median %f" % (np.mean(angle_diff), np.median(angle_diff)))

        # Rotate p2 in matching_points_np, according to the angle_diff variable
        p2_x = matching_points_np[:, 2] * np.cos(yaw_diff) - matching_points_np[:, 3] * np.sin(yaw_diff)
        p2_y = matching_points_np[:, 2] * np.sin(yaw_diff) + matching_points_np[:, 3] * np.cos(yaw_diff)

        matching_points_np[:, 2] = p2_x
        matching_points_np[:, 3] = p2_y

        # Find linear x and y translation for matching points and sort
        x_diff = np.sort(matching_points_np[:, 0] - matching_points_np[:, 2])
        y_diff = np.sort(matching_points_np[:, 1] - matching_points_np[:, 3])

        crop_index = int(round(x_diff.size * 0.2)) # crop data 20 % of largest and smallest values
        x_diff_avg = np.average(x_diff[crop_index : x_diff.size - crop_index])
        y_diff_avg = np.average(y_diff[crop_index : x_diff.size - crop_index])

        # Move matching points according to the translation
        matching_points_np[:, 2] += x_diff_avg
        matching_points_np[:, 3] += y_diff_avg

        return x_diff_avg, y_diff_avg, yaw_diff#, matching_points_np[:,2:3]



