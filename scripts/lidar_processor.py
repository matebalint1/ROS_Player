import numpy as np

class LidarProcessor:

    def __init__(self, min_r, max_r, max_distance_between_two_neighbor_points_in_an_object):
        self._min_r = min_r # m
        self._max_r = max_r # m
        self._max_distance_between_two_neighbor_points_in_an_object = max_distance_between_two_neighbor_points_in_an_object

        self._max_object_size = 0.12 # m
        self._size_of_scan_array = 3 # scans
        self._crop_angle = 90  # deg (total)

        self.scan_array = [] # contains objects from previous scans


    def add_scan(self, raw_scan):

        found_objects = self._get_objects_from_laser_scan(raw_scan)
        filtered_objects = self._filter_objects_larger_than_from_list(found_objects, self._max_object_size)

        self._combine_scan_data(filtered_objects, 0, 0, 0)

        return  filtered_objects


    def get_scan_array(self):
        return self.scan_array


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


    def _combine_scan_data(self, new_scan, dx, dy, dyaw):
        # new_scan: 2d scan
        # scan_data_arr: list containing a arrays of scans (2D)
        # delta_pose: list of dx,dy,dyaw

        # Remove first item from scan array if len >=3
        if len(self.scan_array) >= self._size_of_scan_array:
            del self.scan_array[0]

        # Rotate old points
        for scan in self.scan_array:
            for point in scan:
                x = point[0]
                y = point[1]

                # Rotate point around origin
                x = x * np.cos(dyaw) - y * np.sin(dyaw)
                y = x * np.sin(dyaw) + y * np.cos(dyaw)

                # Linear translation
                x += dx
                y += dy

                # Save translated values
                point[0] = x
                point[1] = y

        # Combine old and new data
        self.scan_array.append(new_scan)



    def _filter_objects_larger_than_from_list(self, obj_array, max_size):
        filtered_obj = []

        for obj in obj_array:
            p1_x = obj[0]
            p1_y = obj[1]
            p2_x = obj[2]
            p2_y = obj[3]

            distance = np.sqrt((p1_x - p2_x) * (p1_x - p2_x) + (p1_y - p2_y) * (p1_y - p2_y))

            if distance <= max_size:
                avg_x = (p1_x + p2_x) / 2
                avg_y = (p1_y + p2_y) / 2

                filtered_obj.append([avg_x, avg_y])

        return filtered_obj


    #def get_hough_transform
    #def