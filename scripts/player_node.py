#!/usr/bin/env python
import rospy
import copy
import numpy as np


from sensor_msgs.msg import LaserScan, Image, PointCloud2
from geometry_msgs.msg import Twist, Point

import cv2
from cv_bridge import CvBridge, CvBridgeError


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

        str_prefix = "robot1/"
        self.velocity_pub = rospy.Publisher(str_prefix + "cmd_vel", Twist, queue_size=1000)

        self.image_sub = rospy.Subscriber(str_prefix + "front_camera/image_raw", Image, self.camera_cb)

        self.laser_sub = rospy.Subscriber(str_prefix + "front_laser/scan", LaserScan, self.laser_cb)



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


def edge(image_scan):
    #Noise removal for the purpose of differentiation and finally edge_detection


    #Capture livestream video content from camera 0
    cap = cv2.VideoCapture(0)

    while True:

        # Take each frame
        frame = cap.read()

        # Convert to HSV for simpler calculations
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Calcution of Sobelx
        sobelx = cv2.Sobel(frame,cv2.CV_64F,1,0,ksize=5)

        # Calculation of Sobely
        sobely = cv2.Sobel(frame,cv2.CV_64F,0,1,ksize=5)

        # Calculation of Laplacian
        laplacian = cv2.Laplacian(frame,cv2.CV_64F)

        cv2.imshow('sobelx',sobelx)
        cv2.imshow('sobely',sobely)
        cv2.imshow('laplacian',laplacian)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()

    #release the frame
    cap.release()
	



def simple_collision_avoidance(range_measurements):
    # This function uses raw laser scan measurements
    # and uses them to calculate the closest obstacles in
    # front of the robot. If an obstacle is closer than allowed
    # the robot will rotate untill there is enough space in front of it. 

    range_n = len(range_measurements)  # number of measurements in the scan
    angle_increment = abs(cur_laser.angle_increment)  # get increment between scans in array
    angele_to_keep_clear = 3.14 / 2 / 2  # rad, defines the angle of the scan than is kept clear of obstacles (one sided)

    i_start = int(range_n / 2 - angele_to_keep_clear / angle_increment)  # index
    i_end = int(range_n / 2 + angele_to_keep_clear / angle_increment)  # index

    # Convert measuremts to numpy array and sort it from smallest to largest
    sub_measurements = np.sort(np.array(range_measurements[i_start:i_end]))

    # Remove infinity
    sub_measurements[sub_measurements == np.inf] = 6

    # Calculate the average distance of the 4 closest scans
    avg_distance = np.average(sub_measurements[0:3])

    speed_forward = 0
    speed_rotational = 0

    # Determine speed for robot
    if avg_distance < 0.65:
        speed_forward = 0
        speed_rotational = 0.2
    else:
        # Calculate speed depending of the distance to the closest objects:
        speed_forward = min(avg_distance, 1) / 1 * 0.2 + 0.1


    print("Distance to obstacle %f m, setting speeds %f m/s  %f rad/s" %
          (avg_distance, speed_forward, speed_rotational))

    if avg_distance < 0.05:
        return # do not do anything
    play_node.set_velocities(speed_forward, speed_rotational)



if __name__ == '__main__':

    play_node = PlayNode()


    last_pose_of_robot = [0,0,0] # x,y,yaw
    scan_data_array_main = [] # For combining multiple laser scans


    # 10 Hz loop
    loop_rate = rospy.Rate(10)
    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        # We check that we have both laser and image data. Because these are
        # constantly being updated, we should always have the most recent
        # messages received


        if play_node.laser:# and play_node.image.any():
            #start_time = time.time() # for measuring speed



            cur_laser = copy.deepcopy(play_node.laser)
            # cur_image = copy.deepcopy(play_node.image)


            # Movement of the robot
            simple_collision_avoidance(cur_laser.ranges)
            #play_node.set_velocities(10, 0)
            #play_node.set_velocities(1, 0)

            #play_node.show_img()
            #print("Time diff: %f s" % (time.time() - start_time))

        loop_rate.sleep()

    play_node.set_velocities(0, 0) # make sure robot stops after stopping program
