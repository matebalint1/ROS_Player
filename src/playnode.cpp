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

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filterh>

// Sement Added by Reza//
// Aims at Re-Exporting in-game Objects
#include <pcl/sample_consensus/mlesac.h>
typedef pcl::PointCould<PointT> CouldPTr;




const std::string IMAGE_WINDOW = "Camera image";
ros::Publisher velocity_pub;
cv::Mat image;
sensor_msgs::LaserScan laser_msg;
bool got_image;
bool got_laser;

void image_callback(const sensor_msgs::Image::ConstPtr& msg_img)
{
  ROS_INFO("Got new image");
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
  // Store the latest image received - this variable always has the most up to
  // date image.
  image = cv_ptr->image;
  got_image = true;
}


void feature_extract(CloudPtr& raw_Cloud, CloudPTr& output_Cloud)
{

}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg_laser){
  ROS_INFO("Got new laser scan");
  // Store the laser message we get. The laser_msg variable always holds the
  // latest laser message.
  laser_msg = *msg_laser;
  got_laser = true;
}

void set_velocities(float lin_vel, float ang_vel)
{
  geometry_msgs::Twist msg;
  msg.linear.x = lin_vel;
  msg.angular.z = ang_vel;

  velocity_pub.publish(msg);
}

void show_image() {
  cv::imshow(IMAGE_WINDOW, image);
  cv::waitKey(10);
}


int main(int argc, char **argv){
  // The node name is robot_node
  ros::init(argc, argv, "robot_node");
  // The nodehandle actually starts the node
  ros::NodeHandle n;

  // Publish messages to move around
  velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ROS_INFO("Waiting for camera image");
  ros::topic::waitForMessage<sensor_msgs::Image>("robot1/front_camera/image_raw");
  ROS_INFO("Waiting for laser scan message");
  ros::topic::waitForMessage<sensor_msgs::LaserScan>("robot1/front_laser/scan");

  // Images need a special subscriber
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub = it.subscribe("robot1/front_camera/image_raw", 1, image_callback);

  // basic subscriber for the laser
  ros::Subscriber laser_sub = n.subscribe("robot1/front_laser/scan", 1, laser_callback);

  cv::namedWindow(IMAGE_WINDOW);

  // 10Hz loop
  ros::Rate r(10);
  while(ros::ok()){
    if (got_laser && got_image) {
      // Copy the laser and image. These change whenever the callbacks run, and
      // we don't want the data changing in the middle of processing.
      cv::Mat cur_img = image;
      //sensor_msgs::LaserScan cur_laser = laser_msg;



      show_image();
    }


    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
