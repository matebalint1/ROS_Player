#include "ros/ros.h"
#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#define IMAGE_WINDOW "Camera Input"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> approx_sync;
ros::Publisher velocity_pub;

void perception_callback(const sensor_msgs::Image::ConstPtr& msg_img, const sensor_msgs::LaserScan::ConstPtr& msg_laser)
{
    ROS_INFO("New image and scan available");
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    cv::flip(image, image, -1);
    cv::imshow(IMAGE_WINDOW, image);
    cv::waitKey(10);

    /* DO SOMETHING WITH image AND msg_laser HERE... OR NOT...*/
}

void set_velocities(float lin_vel, float ang_vel)
{
    geometry_msgs::Twist msg;
    msg.linear.x = lin_vel;
    msg.angular.z = ang_vel;

    velocity_pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_node");
  ros::NodeHandle n;

  velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::topic::waitForMessage<sensor_msgs::Image>("front_camera/image_raw");
  ros::topic::waitForMessage<sensor_msgs::LaserScan>("front_laser/scan");
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "front_camera/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(n, "front_laser/scan", 1);

  message_filters::Synchronizer<approx_sync> sync(approx_sync(20), image_sub, laser_sub);
  sync.registerCallback(boost::bind(&perception_callback, _1, _2));

  cv::namedWindow(IMAGE_WINDOW);

  ros::Rate r(10);
  while(ros::ok())
  {
      /* THIS LOOP RUNS at 10 Hz */

      /* DO STUFF HERE... OR NOT... */

//      set_velocities(0.2, 0.0);

      ros::spinOnce();
      r.sleep();
  }

  return 0;
}
