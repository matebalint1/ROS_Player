#!/usr/bin/env python
import sys

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import message_filters

import cv2
from cv_bridge import CvBridge, CvBridgeError

class PlayNode:

    
    def __init__(self, window_name="Camera Input", message_slop=0.1, synchroniser_queuesize=20):
        """
        :param message_slop: Messages with a header.stamp within message_slop
        seconds of each other will be considered to be synchronisable
        """
        rospy.init_node("robot_node")
        rospy.loginfo("Initialised PlayNode")
        
        self.bridge = CvBridge()
        self.image_window = window_name
        self.image = None

        self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        self.image_sub = message_filters.Subscriber("front_camera/image_raw", Image)
        self.laser_sub = message_filters.Subscriber("front_laser/scan", LaserScan)

        self.time_sync = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.laser_sub],
                                                                     synchroniser_queuesize,
                                                                     message_slop)
        self.time_sync.registerCallback(self.perception_cb)

    def perception_cb(self, img_msg, laser_msg):
        rospy.loginfo("Received new image ({}) and scan ({})".format(img_msg.header.stamp, laser_msg.header.stamp))
        
        try:
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        self.image = cv2.flip(image, -1)

        # DO SOMETHING WITH image AND laser_msg HERE... OR NOT...

    def show_img(self):
        """Python doesn't like waitKey in the perception_cb
        """
        if self.image is not None:
            cv2.imshow(self.image_window, self.image)
            cv2.waitKey(1)
        else:
            rospy.loginfo("No image to show yet")
        
    def set_velocities(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.velocity_pub.publish(msg)

if __name__ == '__main__':

    play_node = PlayNode()
       
    loop_rate = rospy.Rate(10)

    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():

        # 10 Hz loop
        play_node.show_img()
        loop_rate.sleep()

