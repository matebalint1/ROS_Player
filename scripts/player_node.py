#!/usr/bin/env python
import sys


import rospy
import copy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge, CvBridgeError

# You don't have to use a class, but it keeps things together and means that you
# don't use global variables
class PlayNode:

    def __init__(self, window_name="Camera Input"):
        """
        :param message_slop: Messages with a header.stamp within message_slop
        seconds of each other will be considered to be synchronisable
        """
        rospy.init_node("robot_node")
        rospy.loginfo("Initialised PlayNode")

        self.bridge = CvBridge()
        self.image_window = window_name
        self.image = None
        self.laser = None

        self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        self.image_sub = rospy.Subscriber("front_camera/image_raw", Image, self.camera_cb)
        self.laser_sub = rospy.Subscriber("front_laser/scan", LaserScan, self.laser_cb)

    def laser_cb(self, msg):
        """This function is called every time a message is received on the
        front_laser/scan topic

        We just keep the self.laser member up to date
        """
        #rospy.loginfo("Received new scan")
        self.laser = msg

    def camera_cb(self, msg):
        """This function is called every time a message is received on the
        front_camera/image_raw topic

        We convert the image to an opencv object and update the self.image member
        """
        #rospy.loginfo("Received new image")

        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        self.image = cv2.flip(image, -1)



    def show_img(self):
        """Need to do this separately because otherwise it can freeze
        """
        if self.image is not None:
            cv2.imshow(self.image_window, self.image)
            cv2.waitKey(1)
        else:
            rospy.loginfo("No image to show yet")


    def set_velocities(self, linear, angular):
        """Use this to set linear and angular velocities
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.velocity_pub.publish(msg)


"""
def get_objects_visible_in_laser_scan(scan_data, min_r, max_r):
    # This function returns a list of objects (point that are close to each other)
    # in the lidar scan, each cell contains an array containing distance and direction in rad.

    # scan data is a numpy array
    # min_r and max_r define in meters the max distance of the objects

    list_of_objects = []

    # defines max diameter of an object (this function does not detect walls)
    max_object_diameter = 0.15 # m
    max_distance_between_points = 0.15 # m
    min_number_of_points_in_one_object = 4

    n = scan_data.size
    i = 0
    angle_increment_between_scans = 2*3.1415/n
    print(scan_data)
    print(n)
    while i < n - 1:

        r_p1 = (scan_data[i])
        if r_p1 < max_r and r_p1 > min_r:
            max_number_of_subscans = int(round(max_object_diameter / \
                                               r_p1 / \
                                               angle_increment_between_scans))# rad
            print("max nuber of sub scans %d" % max_number_of_subscans)
            group_i = 1
            while group_i <= max_number_of_subscans and i + group_i < n:

                r_p2 = scan_data[i + group_i]

                if r_p2 < max_r and r_p2 > min_r:
                    #print("r_p2 %f" % r_p2)
                    # Calculate angle of each point
                    angle_p1 = i * angle_increment_between_scans
                    angle_p2 = (i + group_i) * angle_increment_between_scans

                    distance_between_points = np.sqrt(r_p1*r_p1 + r_p2*r_p2 - \
                                                      2*r_p1*r_p2*np.cos(angle_p1 - angle_p2))
                    print("distance of p1:%d p2:%d : %f"%(i,i+group_i,distance_between_points))
                    if distance_between_points <= max_distance_between_points:
                        group_i += 1
                    else:
                        print("got something at index: %d**********************************" % group_i)
                        if group_i >= min_number_of_points_in_one_object:
                            # add found object to list
                            avg_angle = (angle_p1 + angle_p2) / 2
                            avg_distance = (r_p1 + r_p2) / 2
                            list_of_objects.append([avg_distance, avg_angle])
                        # go to next point in main loop
                        break
                else:
                    break


        i += 1


    return list_of_objects
"""

def get_objects_visible_in_laser_scan(scan_data, min_r, max_r):
    min_number_of_measurements_in_2x2_to_be_accepted = 3

    pixel_size = 0.05 # m
    map_size = 2*5 # m
    resolution_of_map = int(map_size / pixel_size)

    # Middle of the map is where the robot is
    map = np.zeros([resolution_of_map, resolution_of_map]) # 2D array

    n = scan_data.size
    angle_increment_between_scans = 2*3.1415/n
    i = 0

    # Put each measurement to the map
    while i < n:
        r = scan_data[i]

        if r < max_r and r > min_r:
            angle = i * angle_increment_between_scans
            x = r * np.cos(angle)
            y = r * np.sin(angle)

            x_i = int(round((x + map_size/2 ) / pixel_size)) # origin is in the middle of the map
            y_i = int(round((y + map_size/2 ) / pixel_size))

            if x_i < resolution_of_map and y_i < resolution_of_map:
                map[y_i, x_i] += 1 # add one to map
                #print("added to map x:%f   y:%f"% (x,y))

        i += 1

    # Find all 2x2 cells in the map that contain enough measurements
    list_of_obj = []
    list_of_obj_radial = []
    y_i = 0
    while y_i < resolution_of_map - 1:
        x_i = 0
        while x_i < resolution_of_map - 1:
            number_of_measurements_2x2 = map[y_i,x_i] + \
                                        map[y_i + 1,x_i] + \
                                        map[y_i + 1, x_i + 1] + \
                                        map[y_i, x_i + 1]

            if number_of_measurements_2x2 >= min_number_of_measurements_in_2x2_to_be_accepted:
                x = -((x_i + 0.5) * pixel_size - map_size/2)
                y = -((y_i + 0.5) * pixel_size - map_size/2)
                list_of_obj.append([x, y])

                r = np.sqrt(x*x + y*y)
                angle = np.arctan2(y, x) * 180/3.1415
                list_of_obj_radial.append([r, angle])


            x_i += 1
        y_i += 1


    # Combine double measurements, e.g. objects that are close to each other (closer than 0.1 m)
    i_first = 0
    while True:
        i_second = i_first + 1

        while i_second < len(list_of_obj):

            p1 = list_of_obj[i_first]
            p2 = list_of_obj[i_second]

            distance_between_points = np.sqrt((p1[0]-p2[0]) * (p1[0]-p2[0]) + (p1[1]-p2[1]) * (p1[1]-p2[1]))
            #print(distance_between_points)
            if distance_between_points <= 0.1:
                avg_x = (p1[0] + p2[0]) / 2
                avg_y = (p1[1] + p2[1]) / 2

                list_of_obj[i_first] = [avg_x, avg_y]
                list_of_obj.pop(i_second)
            else:
                i_second += 1



        i_first += 1
        if i_first >= len(list_of_obj) - 1:
            break


    #for pos in list_of_obj:
        #print(pos)

    #print("Orginaly %d, now:%d" % (len(list_of_obj_radial), len(list_of_obj)))

    return list_of_obj


def collision_avoidance(laser_scan_objects, default_speed_forward):
    robot_v_x = default_speed_forward
    robot_v_y = 0

    for o in laser_scan_objects:
        x = o[0]
        y = o[1]
        r = np.sqrt(x*x + y*y)

        proximity_factor_of_obstacle = 1 / (r)
        robot_v_x -= proximity_factor_of_obstacle * x / r
        robot_v_y -= proximity_factor_of_obstacle * y / r

    robot_v_angle = -0.2*np.arctan2(robot_v_y, robot_v_x)
    robot_v_total = 0.4*np.sqrt(robot_v_y*robot_v_y + robot_v_x*robot_v_x)

    print("ROBOT speed linear:%f, yaw:%f" % (robot_v_total, robot_v_angle))
    play_node.set_velocities(robot_v_total, robot_v_angle)

def simple_collision_avoidance(range_measurements):
    range_n = len(range_measurements)  # number of measurements in the scan
    angle_increment = abs(cur_laser.angle_increment)  # get increment between scans in array
    angele_to_keep_clear = 3.14 / 2 / 2  # rad, defines the angle of the scan than is kept clear of obstacles

    i_start = int(range_n / 2 - angele_to_keep_clear / angle_increment)  # index
    i_end = int(range_n / 2 + angele_to_keep_clear / angle_increment)  # index

    # Convert measuremts to numpy array and sort it from smalest to largest
    sub_measurements = np.sort(np.array(range_measurements[i_start:i_end]))
    # print(sub_measurements)

    # Calculate the average distance of the 4 closest scans
    sum_of_ranges = 0
    for i in range(4):
        if sub_measurements[i] < 6:
            sum_of_ranges += sub_measurements[i]
        else:
            sub_measurements += 6

    avg_distance = sum_of_ranges / 4

    print(avg_distance)

    if avg_distance < 0.5:
        play_node.set_velocities(-0.1, 0.3)
    else:
        # Calculate speed depending of the distance to the closest objects:
        speed = min(avg_distance, 2) / 2 * 0.9 + 0.1

        play_node.set_velocities(speed, 0)



if __name__ == '__main__':
    play_node = PlayNode()

    # You can keep the following in main, or put it into a PlayNode.run() function.

    # 10 Hz loop
    loop_rate = rospy.Rate(100)

    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        # We check that we have both laser and image data. Because these are
        # constantly being updated, we should always have the most recent
        # messages received


        if play_node.laser and play_node.image.any():
            # Now, we can use the laser and image data to do something. Because
            # the objects in the PlayNode are constantly updated, we need to
            # make a deep copy so that the data doesn't change while we're doing
            # computations on it. This isn't very efficient, but we do it just
            # for demonstration purposes
            cur_laser = copy.deepcopy(play_node.laser)
            cur_image = copy.deepcopy(play_node.image)

            # Do something useful with laser and image here


            range_measurements = cur_laser.ranges

            #simple_collision_avoidance(range_measurements)
            #print(cur_laser)

            list_of_obj = get_objects_visible_in_laser_scan(np.array(range_measurements), 0.1, 5.9)
            collision_avoidance(list_of_obj, 1)
            #print(list_of_obj)

            #play_node.show_img()

        loop_rate.sleep()
