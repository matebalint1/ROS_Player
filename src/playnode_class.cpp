#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include <image_transport/image_transport.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/foreach.hpp>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPrt;

const std::string IMAGE_WINDOW = "Camera image";

class PlayNode {
  public:

  PlayNode(int argc, char **argv){
    ros::init(argc, argv, "robot_node");
    n = std::make_unique<ros::NodeHandle>();
    
    tfBuffer = new tf2_ros::Buffer(ros::Duration(100));
    tf_listener = new tf2_ros::TransformListener(*tfBuffer);

    velocity_pub = n->advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1000);
    kinect_pub = n->advertise<PointCloud> ("player/kinect_processed", 1);

    //ROS_INFO("Waiting for camera image");
    //ros::topic::waitForMessage<sensor_msgs::Image>("robot1/front_camera/image_raw");
    //ROS_INFO("Waiting for laser scan message");
    //ros::topic::waitForMessage<sensor_msgs::LaserScan>("robot1/front_laser/scan");
    ROS_INFO("Waiting for /robot1/kinect/depth_registered/points");
    ros::topic::waitForMessage<PointCloud>("robot1/kinect/depth_registered/points");

    it = std::make_unique<image_transport::ImageTransport>(*n);

    // If not in an object, the fourth parameter here is not necessary, but we
    // need it here to ensure the callback goes to the right place, i.e. the
    // image_callback function of this object. We also have to get a reference
    // to the class member callback function as opposed to just providing the
    // name of the function as we usually do
    // Note: the first slash is important here! If you do not add the preceding slash, the node will instead subscribe
    //image_sub = it->subscribe("robot1//front_camera/image_raw", 1, &PlayNode::image_callback, this);
    //laser_sub = n->subscribe("robot1//front_laser/scan", 1, &PlayNode::laser_callback, this);
    
    kinect_sub = n->subscribe("robot1/kinect/depth_registered/points", 1, &PlayNode::kinect_callback, this);
  }

  void image_callback(const sensor_msgs::Image::ConstPtr& msg){
    ROS_INFO("Got new laser scan");
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image = cv_ptr->image;
    got_image = true;
  }

  void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    ROS_INFO("Got new image");
    laser_msg = *msg;
    got_laser = true;
  }

  void kinect_callback(const PointCloud::ConstPtr& msg){
    ROS_INFO("Got new kinect");
    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    //BOOST_FOREACH (const pcl::PointXYZRGB& pt, msg->points)
    //  printf ("\t(%f, %f, %d)\n", pt.x, pt.y, pt.r);
    kinect_msg = *msg;
    got_kinect = true;
  }

  void show_image() const {
    cv::imshow(IMAGE_WINDOW, image);
    cv::waitKey(10);
  }
  
  void set_velocities(float lin_vel, float ang_vel) const {
    geometry_msgs::Twist msg;
    msg.linear.x = lin_vel;
    msg.angular.z = ang_vel;

    velocity_pub.publish(msg);
  }

  void pub_pointcloud(PointCloud& cloud) {
    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "robot1/base_link";

    msg->height = cloud.height;
    msg->width = cloud.width;

    /*
    pcl::PointXYZRGB p (1.0, 2.0, 3.0);
    uint8_t r = 255, g = 255, b = 0;    // Example: Red color
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    p.rgb = *reinterpret_cast<float*>(&rgb);
    msg->points.push_back (p);
    */
    msg->points = cloud.points;

    kinect_pub.publish(msg);
  }

  void process_messages() {
    // Copy the laser and image. These change whenever the callbacks run, and
    // we don't want the data changing in the middle of processing.
    //cv::Mat cur_img = image;
    //sensor_msgs::LaserScan cur_laser = laser_msg;
 
    PointCloudPrt cur_kinect_in (new PointCloud(kinect_msg)); // make copy
    PointCloudPrt cur_kinect (new PointCloud);

     tf::Transform transform;//(/*tf::Quaternion(0,0,0,0)*/);
    geometry_msgs::TransformStamped transformStamped;
    try{
       transformStamped = tfBuffer->lookupTransform("robot1/base_link",(*cur_kinect_in).header.frame_id,  ros::Time(0));
      
      tf::Vector3 vector(transformStamped.transform.translation.x,
                         transformStamped.transform.translation.y,
                         transformStamped.transform.translation.z);

      tf::Quaternion rotation(transformStamped.transform.rotation.x,
                              transformStamped.transform.rotation.y,
                              transformStamped.transform.rotation.z,
                              transformStamped.transform.rotation.w);
      
      transform.setOrigin(vector);
      transform.setRotation(rotation);

    }catch (tf2::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    pcl_ros::transformPointCloud(*cur_kinect_in, *cur_kinect, transform);

    PointCloudPrt cloud_filtered (new PointCloud);
    PointCloudPrt cloud_pass(new PointCloud);

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
    pcl::PassThrough<pcl::PointXYZRGB> pass_through_filter;

    //pass through filter
    pass_through_filter.setInputCloud (cur_kinect);
    pass_through_filter.setFilterFieldName ("z");
    pass_through_filter.setFilterLimits (0.0, 2.0);
    pass_through_filter.filter (*cloud_pass);

    //voxel grid filter
    voxel_grid_filter.setInputCloud(cloud_pass);
    voxel_grid_filter.setLeafSize(0.02, 0.02, 0.02);
    voxel_grid_filter.filter(*cloud_filtered);
    
    pub_pointcloud(*cloud_filtered);
  }

  bool got_messages() const {
    return got_kinect; //got_image && got_laser;
  }

  private:

  bool got_image;
  bool got_laser;
  bool got_kinect;
 
  std::unique_ptr<ros::NodeHandle> n;
  std::unique_ptr<image_transport::ImageTransport> it;
  
  ros::Publisher velocity_pub;
  ros::Publisher kinect_pub;

  image_transport::Subscriber image_sub;
  ros::Subscriber laser_sub;
  ros::Subscriber kinect_sub;

  tf2_ros::Buffer* tfBuffer;
  tf2_ros::TransformListener* tf_listener;
  
  cv::Mat image;
  sensor_msgs::LaserScan laser_msg;
  PointCloud kinect_msg;
};

int main(int argc, char **argv){
  PlayNode playNode(argc, argv);
  //cv::namedWindow(IMAGE_WINDOW);
  
  // 10 Hz loop
  ros::Rate r(10);
  while(ros::ok())
  {

    //ROS_INFO("%d", playNode.got_messages());
    
    if (playNode.got_messages()) {
      playNode.process_messages();
      //playNode.show_image();
    }

    // Need to run spinOnce to get the callback functions to run. This is important! 
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
