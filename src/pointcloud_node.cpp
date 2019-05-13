#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>

#include "pointcloud_processor.hpp"

#include "cv_bridge/cv_bridge.h"        // do not move up
#include "opencv2/highgui/highgui.hpp"  // do not move up
#include "opencv2/opencv.hpp"


#inlude        // do not move up

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPrt;

class PlayNode {
   public:
    PlayNode(int argc, char **argv) {
        ros::init(argc, argv, "pointcloud_node");
        n = std::make_unique<ros::NodeHandle>();

        tfBuffer = new tf2_ros::Buffer(ros::Duration(100));
        tf_listener = new tf2_ros::TransformListener(*tfBuffer);

        velocity_pub =
            n->advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1000);
        kinect_pub =
            n->advertise<PointCloud>("pointcloud_node/detected_objects", 1);

        // ROS_INFO("Waiting for camera image");
        // ros::topic::waitForMessage<sensor_msgs::Image>("robot1/front_camera/image_raw");
        // ROS_INFO("Waiting for laser scan message");
        // ros::topic::waitForMessage<sensor_msgs::LaserScan>("robot1/front_laser/scan");
        ROS_INFO("Waiting for /robot1/kinect/depth_registered/points");
        ros::topic::waitForMessage<PointCloud>(
            "robot1/kinect/depth_registered/points");

        it = std::make_unique<image_transport::ImageTransport>(*n);

        // If not in an object, the fourth parameter here is not necessary, but
        // we need it here to ensure the callback goes to the right place, i.e.
        // the image_callback function of this object. We also have to get a
        // reference to the class member callback function as opposed to just
        // providing the name of the function as we usually do Note: the first
        // slash is important here! If you do not add the preceding slash, the
        // node will instead subscribe
        // image_sub = it->subscribe("robot1//front_camera/image_raw", 1,
        // &PlayNode::image_callback, this); laser_sub =
        // n->subscribe("robot1//front_laser/scan", 1,
        // &PlayNode::laser_callback, this);

        kinect_sub = n->subscribe("robot1/kinect/depth_registered/points", 1,
                                  &PlayNode::kinect_callback, this);

        pointcloud_processor = PointcloudProcessor();
    }

    void image_callback(const sensor_msgs::Image::ConstPtr &msg) {
        ROS_INFO("Got new laser scan");
        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image = cv_ptr->image;
        got_image = true;
    }

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
        ROS_INFO("Got new image");
        laser_msg = *msg;
        got_laser = true;
    }

    void kinect_callback(const PointCloud::ConstPtr &msg) {
        // ROS_INFO("Got new kinect");
        // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

        // BOOST_FOREACH (const pcl::pub_pointPointXYZRGB& pt, msg->points)
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

    void pub_pointcloud(PointCloud &cloud) {
        PointCloud::Ptr pcl_msg(new PointCloud);
        // msg->header.frame_id = "robot1/base_link";
        msg->header.frame_id = "robot1/odom";

        msg->height = cloud.height;
        msg->width = cloud.width;
        msg->points = cloud.points;

        kinect_pub.publish(pcl_msg);
    }

    tf::Transform get_transform(std::string goal_frame,
                                std::string current_frame) {
        tf::Transform transform;
        geometry_msgs::TransformStamped transformStamped;
        try {
            // Find transformation for pointcloud from the buffer
            // (*cur_kinect_in).header.frame_id ==
            // "robot1/kinect_rgb_optical_frame"
            transformStamped = tfBuffer->lookupTransform(
                goal_frame, current_frame, ros::Time(0));

            // Convert TransformStamped to Transform
            tf::Vector3 vector(transformStamped.transform.translation.x,
                               transformStamped.transform.translation.y,
                               transformStamped.transform.translation.z);

            tf::Quaternion rotation(transformStamped.transform.rotation.x,
                                    transformStamped.transform.rotation.y,
                                    transformStamped.transform.rotation.z,
                                    transformStamped.transform.rotation.w);

            transform.setOrigin(vector);
            transform.setRotation(rotation);
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        return transform;
    }

    void process_messages() {
        // Copy the laser and image. These change whenever the callbacks
        // run, and we don't want the data changing in the middle of
        // processing.
        // cv::Mat cur_img = image;
        // sensor_msgs::LaserScan cur_laser = laser_msg;

        // Copy input cloud
        PointCloudPrt cur_kinect_in(new PointCloud(kinect_msg));
        // Temporary pointcloud for transformation
        PointCloudPrt cur_kinect(new PointCloud);

        // Find transformation for desired frames
        tf::Transform transform_kinect_to_odom =
            get_transform("robot1/odom", "robot1/kinect_rgb_optical_frame");
        tf::Transform transform_odom_to_baselink =
            get_transform("robot1/base_link", "robot1/odom");

        // Transform pointcloud to odom tf frame
        pcl_ros::transformPointCloud(*cur_kinect_in, *cur_kinect,
                                     transform_kinect_to_odom);

        // Process pointcloud
        pointcloud_processor.process_pointcloud(cur_kinect, transform_odom_to_baselink);

        // Publish pointcloud
        pub_pointcloud(*pointcloud_processor.get_recognized_objects());

        got_kinect = false;  // reset
    }

    bool got_messages() const {
        return got_kinect;  // got_image && got_laser;
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

    tf2_ros::Buffer *tfBuffer;
    tf2_ros::TransformListener *tf_listener;

    cv::Mat image;
    sensor_msgs::LaserScan laser_msg;
    PointCloud kinect_msg;

    PointcloudProcessor pointcloud_processor;
};

int main(int argc, char **argv) {
    PlayNode playNode(argc, argv);

    // 30 Hz loop
    ros::Rate r(30);
    while (ros::ok()) {
        // ROS_INFO("%d", playNode.got_messages());

        if (playNode.got_messages()) {
            playNode.process_messages();
        }

        // Need to run spinOnce to get the callback functions to run. This
        // is important!
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
