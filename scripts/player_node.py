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

import lidar_processor
import robot_pose
#import  field_finder

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
        self.marker_pub = rospy.Publisher(str_prefix + "visualization_msgs/visualization_marker_array", MarkerArray, queue_size=1000)

        self.point_cloud_pub = rospy.Publisher(str_prefix + "visualization_msgs/object_cloud", PointCloud2, queue_size=1000)
        self.point_cloud_pub2 = rospy.Publisher(str_prefix + "visualization_msgs/object_cloud2", PointCloud2, queue_size=1000)

        self.image_sub = rospy.Subscriber(str_prefix + "front_camera/image_raw", Image, self.camera_cb)
        self.laser_sub = rospy.Subscriber(str_prefix + "front_laser/scan", LaserScan, self.laser_cb)
        #self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laser_cb)
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
            cloud_points.append([p[0],p[1],0])

        scaled_polygon_pcl = sensor_msgs.point_cloud2.create_cloud_xyz32(header, cloud_points)

        self.point_cloud_pub.publish(scaled_polygon_pcl)


    def publish_point_cloud2(self, points):
        # points are in a 2d list: [[x,y], ... ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "robot1/base_link"

        # Add z coordinate to points
        cloud_points = []
        for p in points:
            cloud_points.append([p[0],p[1],0])

        scaled_polygon_pcl = sensor_msgs.point_cloud2.create_cloud_xyz32(header, cloud_points)

        self.point_cloud_pub2.publish(scaled_polygon_pcl)




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

    return last_pose_of_robot, delta_x, delta_y, delta_yaw, yaw_now


def simple_collision_avoidance_2(obj_array):
    # This function user already recognized objects for the laser scan

    # Sector to keep empty in front of the robot
    sector_angle = 60 # deg
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
        play_node.set_velocities(-0.1, 0.2)
    else:
        # Calculate speed depending of the distance to the closest objects:
        speed = min(closest_obj, 1) / 1 * 0.2
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
    # Remove infinity
    sub_measurements[sub_measurements == np.inf] = 6
    #print(sub_measurements)

    # Calculate the average distance of the 4 closest scans
    avg_distance = np.average(sub_measurements[0:3])#sum_of_ranges / 4.0

    speed_forward = 0
    speed_rotational = 0

    if avg_distance < 0.65:
        speed_forward = 0.05
        speed_rotational = 0.2
    else:
        # Calculate speed depending of the distance to the closest objects:
        speed_forward = min(avg_distance, 1) / 1 * 0.4 + 0.1


    print("Distance to obstacle %f m, setting speeds %f m/s  %f rad/s" %
          (avg_distance, speed_forward, speed_rotational))

    if avg_distance < 0.05:
        return # do not do anything
    play_node.set_velocities(speed_forward, speed_rotational)



if __name__ == '__main__':

    play_node = PlayNode()
    lidar = lidar_processor.LidarProcessor()
    pose = robot_pose.RobotPose()
    finder = field_finder.FieldFinder(lidar) # used for finding the field

    last_pose_of_robot = [0,0,0] # x,y,yaw
    scan_data_array_main = [] # For combining multiple laser scans

    # You can keep the following in main, or put it into a PlayNode.run() function.


    # 10 Hz loop
    loop_rate = rospy.Rate(10)
    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        # We check that we have both laser and image data. Because these are
        # constantly being updated, we should always have the most recent
        # messages received


        if play_node.laser and play_node.odom:# and play_node.image.any():
            #start_time = time.time() # for measuring speed

            # Now, we can use the laser and image data to do something. Because
            # the objects in the PlayNode are constantly updated, we need to
            # make a deep copy so that the data doesn't change while we're doing
            # computations on it. This isn't very efficient, but we do it just
            # for demonstration purposes

            cur_laser = copy.deepcopy(play_node.laser)
            # cur_image = copy.deepcopy(play_node.image)
            odometry_data = copy.deepcopy(play_node.odom)

            #odom_time = float(odometry_data.header.stamp.secs) + float(odometry_data.header.stamp.nsecs )/ 1000000000.0
            #laser_time = float(cur_laser.header.stamp.secs) + float(cur_laser.header.stamp.nsecs) / 1000000000.0
            #print("Laser t:%f, odom t:%f" % (laser_time, odom_time))
            #print("Time diff %f s" % (laser_time - odom_time))
            #print(cur_laser.header.stamp.nsecs/ 1000000000.0)

            # Pose change:
            last_pose_of_robot, delta_x, delta_y, delta_yaw, yaw_now = get_delta_pose(odometry_data)
            #print([delta_x, delta_y, delta_yaw])

            # Process lidar
            list_of_obj, total_delta_odom = lidar.add_scan(cur_laser, delta_x, delta_y, delta_yaw, yaw_now)
            list_of_all_scans = lidar.get_sum_of_scan_array()

            play_node.publish_point_cloud(list_of_obj)
            play_node.publish_point_cloud2(list_of_all_scans)

            # Find field of lidar data if possible
            #field = finder.find_field(np.array(list_of_obj))

            # Update odometry
            pose.update_odom_pose(total_delta_odom[0][0],
                                  total_delta_odom[0][1],
                                  total_delta_odom[0][2])
            pose.update_odom_pose(total_delta_odom[1][0],
                                  total_delta_odom[1][1],
                                  total_delta_odom[1][2])

            #print(pose.get_pose_on_map())


            # Movement of the robot
            simple_collision_avoidance(cur_laser.ranges)
            #play_node.set_velocities(-0.1, 0.2)
            #play_node.set_velocities(1, 0)
            #simple_collision_avoidance_2(list_of_obj)

            #play_node.show_img()
            #print("Time diff: %f s" % (time.time() - start_time))

        loop_rate.sleep()

    play_node.set_velocities(0, 0) # make sure robot stops after stopping program
