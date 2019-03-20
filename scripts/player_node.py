#!/usr/bin/env python
import sys


import rospy
import copy
import numpy as np
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan, Image, PointCloud2
import sensor_msgs.point_cloud2
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

from visualization_msgs.msg import MarkerArray, Marker

import time
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
        self.odom = None

        str_prefix = "robot1/"
        self.velocity_pub = rospy.Publisher(str_prefix + "cmd_vel", Twist, queue_size=1000)
        self.marker_pub = rospy.Publisher(str_prefix + "visualization_msgs/visualization_marker", Marker, queue_size=1000)
        #self.marker_pub = rospy.Publisher(str_prefix + "visualization_msgs/visualization_marker_array", MarkerArray, queue_size=1000)

        self.point_cloud_pub = rospy.Publisher(str_prefix + "visualization_msgs/object_cloud", PointCloud2, queue_size=1000)

        self.image_sub = rospy.Subscriber(str_prefix + "front_camera/image_raw", Image, self.camera_cb)
        self.laser_sub = rospy.Subscriber(str_prefix + "front_laser/scan", LaserScan, self.laser_cb)
        self.odom_sub = rospy.Subscriber(str_prefix + "odom", Odometry, self.odom_cb)


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

    def odom_cb(self, msg):
        # Odometry data
        #print("got odometry")
        #print(msg)
        self.odom = msg


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
        msg.linear.x = min(linear, 0.5)
        msg.angular.z = min(angular, 0.5)
        self.velocity_pub.publish(msg)

    def publish_marker_to_rviz(self, x, y, id, action):
        # To use this method use the following code:

        # Remove old markers
        # for id in range(100):
        # play_node.publish_marker_to_rviz(0, 0, id, 0)
        # Add new markers
        # i = 0
        # for o in list_of_obj:
        #    play_node.publish_marker_to_rviz(o[0], o[1], i, 1)
        #    i += 1

        marker = Marker()
        marker.id = id
        marker.header.frame_id = "robot1/base_link"
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = x
        marker.pose.position.y = -y
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 1
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.6

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.type = Marker.CYLINDER

        if action == 0:
            marker.action = marker.DELETE
        else:
            marker.action = marker.ADD

        self.marker_pub.publish(marker)

    """
    def publish_marker_array_to_rviz(self, array):

        max_number_of_markers = 100
        marker_array = MarkerArray()

        id = 0
        for a in array:
            marker = Marker()
            marker.id = id
            marker.header.frame_id = "robot1/base_link"
            #marker.header.stamp = rospy.Time.now()
            marker.pose.position.x = a[0]
            marker.pose.position.y = -a[1]
            marker.pose.position.z = 0

            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 1
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 0

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.6

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.type = Marker.CYLINDER
            marker.action = marker.ADD

            marker_array.markers.append(marker)

            id += 1
            id = min(id, max_number_of_markers)

        # Remove old markers
        while id <= max_number_of_markers:
            marker = Marker()
            marker.action = marker.DELETE
            marker_array.markers.append(marker)
            id += 1


        self.marker_pub.publish(marker_array)
    """

    def publish_point_cloud(self, points):
        # points are in a 2d list: [[x,y], ... ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "robot1/base_link"

        # Add z coordinate to points
        cloud_points = []
        for p in points:
            cloud_points.append([p[0],-p[1],0])

        scaled_polygon_pcl = sensor_msgs.point_cloud2.create_cloud_xyz32(header, cloud_points)

        self.point_cloud_pub.publish(scaled_polygon_pcl)




def get_delta_pose(odometry_data):

    #print(odometry_data)

    x_now = odometry_data.pose.pose.position.x
    y_now = odometry_data.pose.pose.position.y

    qx = odometry_data.pose.pose.orientation.x
    qy = odometry_data.pose.pose.orientation.y
    qz = odometry_data.pose.pose.orientation.z
    qw = odometry_data.pose.pose.orientation.w

    siny_cosp = +2.0 * (qw * qz + qx * qy)
    cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_now = np.arctan2(siny_cosp, cosy_cosp)

    delta_x = x_now - last_pose_of_robot[0]
    delta_y = y_now - last_pose_of_robot[1]
    delta_yaw = yaw_now - last_pose_of_robot[2]

    last_pose_of_robot[0] = x_now
    last_pose_of_robot[1] = y_now
    last_pose_of_robot[2] = yaw_now

    return last_pose_of_robot, delta_x, delta_y, delta_yaw


def raw_lidar_to_2D_array(scan_data, min_r, max_r):

    # The following limits are used to mask out the back side of the robot.
    scan_angle_lower_limit = 50 * 3.1415/180 # rad
    scan_angle_upper_limit = (360 -50) * 3.1415 / 180  # rad

    scan_2D = []

    n = scan_data.size
    angle_increment_between_scans = 2*3.1415/n

    i = 0
    while i < n:
        r = scan_data[i]
        angle = i * angle_increment_between_scans

        if r < max_r and r > min_r and angle <= scan_angle_upper_limit and angle >= scan_angle_lower_limit:
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            scan_2D.append([x,y])
        i += 1

    return scan_2D



def combine_scan_data(new_scan, scan_data_arr, dx, dy, dyaw , min_r, max_r):
    # new_scan: 2d scan
    # scan_data_arr: list containing a arrays of scans (2D)
    # delta_pose: list of dx,dy,dyaw

    # Remove first item from scan array if len >=3
    if len(scan_data_arr) >= 1:
        del scan_data_arr[0]

    # Rotate old points
    for scan in scan_data_arr:
        for point in scan:
            x = point[0]
            y = point[1]

            # Rotate point around origin
            x = x * np.cos(dyaw) - y * np.sin(dyaw)
            y = x * np.sin(dyaw) + y * np.cos(dyaw)

            # Linear translation
            x += dx
            y += dy

    # Combine old and new data
    scan_data_arr.append(raw_lidar_to_2D_array(np.array(new_scan.ranges), min_r, max_r))

    return scan_data_arr




def get_objects_visible_in_laser_scan(scan_data_array):

    min_number_of_measurements_in_2x2_to_be_accepted = 1
    pixel_size = 0.05 # m
    map_size = 2*6 # m
    resolution_of_map = int(map_size / pixel_size)

    # Middle of the map is where the robot is
    map = np.zeros([resolution_of_map, resolution_of_map]) # 2D array

    # Put individual scans to the map grid
    for scan in scan_data_array:
        for point in scan:
            x = point[0]
            y = point[1]

            x_i = int(round((x + map_size/2 ) / pixel_size)) # origin is in the middle of the map
            y_i = int(round((y + map_size/2 ) / pixel_size))

            if x_i < resolution_of_map and y_i < resolution_of_map:
                map[y_i, x_i] += 1 # add one to map
                #print("added to map x:%f   y:%f"% (x,y))



    # Find all 2x2 cells in the map that contain enough measurements
    list_of_obj = []
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

            x_i += 1
        y_i += 1

    """
    # Combine double measurements, e.g. objects that are close to each other
    combine_objects_closer_than = 0.1 # m
    i_first = 0
    while True:
        i_second = i_first + 1

        while i_second < len(list_of_obj):

            p1 = list_of_obj[i_first]
            p2 = list_of_obj[i_second]

            distance_between_points = np.sqrt((p1[0]-p2[0]) * (p1[0]-p2[0]) + (p1[1]-p2[1]) * (p1[1]-p2[1]))
            #print(distance_between_points)
            if distance_between_points <= combine_objects_closer_than:
                avg_x = (p1[0] + p2[0]) / 2
                avg_y = (p1[1] + p2[1]) / 2

                list_of_obj[i_first] = [avg_x, avg_y]
                list_of_obj.pop(i_second)
            else:
                i_second += 1

        i_first += 1
        if i_first >= len(list_of_obj) - 1:
            break
    """

    combine_objects_closer_than = 0.3  # m
    new_list_of_obj = []

    while len(list_of_obj) > 1:
        close_objects_index = []
        close_objects_index.append(0)

        p1 = list_of_obj[0]
        i_second = 1

        # Find close objects
        while i_second < len(list_of_obj):
            p2 = list_of_obj[i_second]
            distance_between_points = np.sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

            if distance_between_points <= combine_objects_closer_than:
                close_objects_index.append(i_second)

            i_second += 1

        # Calculate average position of found nearby objects
        avg_x = 0
        avg_y = 0
        for i in close_objects_index:
            #print("i:%d  x%f y%f" % (i, float(list_of_obj[i][0]),float(list_of_obj[i][1])))
            weight = 1/float(len(close_objects_index))
            #print(weight)
            avg_x +=  weight * list_of_obj[i][0]
            avg_y +=  weight * list_of_obj[i][1]


        # Add found object to the new object array
        new_list_of_obj.append([avg_x,avg_y])


        # Pop used objects from the orginal array
        close_objects_index.sort(reverse=True)
        for i in close_objects_index:
            list_of_obj.pop(i)

    #print("Orginaly %d, now:%d" % (len(list_of_obj_radial), len(list_of_obj)))

    return new_list_of_obj


def collision_avoidance(laser_scan_objects, default_speed_forward):
    # Not working properly, needs development
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



def simple_collision_avoidance_2(obj_array):
    # This function user already recognized objects for the laser scan

    # Sector to keep empty in front of the robot
    sector_angle = 90 # deg
    angle_limit = sector_angle / 2 * 3.1415/180 # rad

    closest_obj = 6 # m (max range of lidar)

    for obj in obj_array:
        x = obj[0]
        y = obj[1]
        r = np.sqrt(x*x+y*y)
        angle = np.arctan2(y,x)

        if angle <= angle_limit and angle >= -angle_limit and r < closest_obj:
            closest_obj = r

    print("Closest object in the front %f m" % closest_obj)
    if closest_obj < 0.7:
        play_node.set_velocities(-0.1, 0.3)
    else:
        # Calculate speed depending of the distance to the closest objects:
        speed = min(closest_obj, 1) / 1 * 0.4
        play_node.set_velocities(speed, 0)



def simple_collision_avoidance(range_measurements):
    # This function uses raw laser scan measurements

    range_n = len(range_measurements)  # number of measurements in the scan
    angle_increment = abs(cur_laser.angle_increment)  # get increment between scans in array
    angele_to_keep_clear = 3.14 / 2 / 2  # rad, defines the angle of the scan than is kept clear of obstacles

    i_start = int(range_n / 2 - angele_to_keep_clear / angle_increment)  # index
    i_end = int(range_n / 2 + angele_to_keep_clear / angle_increment)  # index

    # Convert measuremts to numpy array and sort it from smallest to largest
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

    #print(avg_distance)

    if avg_distance < 0.5:
        play_node.set_velocities(-0.1, 0.3)
    else:
        # Calculate speed depending of the distance to the closest objects:
        speed = min(avg_distance, 1) / 1 * 0.4 + 0.1
        play_node.set_velocities(speed, 0)



if __name__ == '__main__':
    play_node = PlayNode()

    last_pose_of_robot = [0,0,0] # x,y,yaw
    scan_data_array_main = [] # For combining multiple laser scans

    # You can keep the following in main, or put it into a PlayNode.run() function.

    # 10 Hz loop
    loop_rate = rospy.Rate(100)

    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        # We check that we have both laser and image data. Because these are
        # constantly being updated, we should always have the most recent
        # messages received


        if play_node.laser and play_node.odom:# and play_node.image.any():
            ##start_time = time.time() # for measuring speed

            # Now, we can use the laser and image data to do something. Because
            # the objects in the PlayNode are constantly updated, we need to
            # make a deep copy so that the data doesn't change while we're doing
            # computations on it. This isn't very efficient, but we do it just
            # for demonstration purposes
            cur_laser = copy.deepcopy(play_node.laser)
            # cur_image = copy.deepcopy(play_node.image)


            odometry_data = copy.deepcopy(play_node.odom)
            #range_measurements = cur_laser.ranges
            #simple_collision_avoidance(range_measurements)

            # Pose change:
            last_pose_of_robot, delta_x, delta_y, delta_yaw = get_delta_pose(odometry_data)
            #print([delta_x, delta_y, delta_yaw])

            # Combine old and new laser scans using delta pose
            scan_data_array_main = combine_scan_data(cur_laser,\
                                                     scan_data_array_main,\
                                                     delta_x, delta_y, delta_yaw,\
                                                     0.1, 5.9)

            # Find obstacles from the combined point cloud
            list_of_obj = get_objects_visible_in_laser_scan(scan_data_array_main)

            simple_collision_avoidance_2(list_of_obj)
            play_node.publish_point_cloud(list_of_obj)

            #collision_avoidance(list_of_obj, 1)
            #print(len(list_of_obj))

            #play_node.show_img()
            ###print("Time diff: %f s" % (time.time() - start_time))

        loop_rate.sleep()
