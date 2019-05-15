#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>

#include "pointcloud_helpers.hpp"

class PlayNode {
   public:
    PlayNode(int argc, char** argv) {
        ros::init(argc, argv, "play_node");
        n = std::make_unique<ros::NodeHandle>();

        tfBuffer = new tf2_ros::Buffer(ros::Duration(100));
        tf_listener = new tf2_ros::TransformListener(*tfBuffer);

        velocity_pub = n->advertise<geometry_msgs::Twist>("cmd_vel", 1000);

        ROS_INFO("Waiting for laser scan message");
        ros::topic::waitForMessage<sensor_msgs::LaserScan>("front_laser/scan");

        ROS_INFO("Waiting for map_node/map");
        ros::topic::waitForMessage<PointCloud>("map_node/map");

        map_sub =
            n->subscribe("map_node/map", 1, &PlayNode::map_callback, this);
        laser_sub = n->subscribe("/front_laser/scan", 1,
                                 &PlayNode::laser_callback, this);
    }

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // ROS_INFO("Got new laser");
        laser_msg = *msg;
        got_laser = true;
    }

    void map_callback(const PointCloud::ConstPtr& msg) {
        // ROS_INFO("Got new map");
        map_objects_msg = *msg;
        got_map = true;
    }

    void set_velocities(float lin_vel, float ang_vel) const {
        geometry_msgs::Twist msg;
        msg.linear.x = lin_vel;
        msg.angular.z = ang_vel;

        velocity_pub.publish(msg);
    }

    void process_messages() {
        sensor_msgs::LaserScan cur_laser = laser_msg;
        got_map = false;
        got_laser = false;
    }

    bool got_messages() const { return got_laser && got_laser; }

   private:
    bool got_map;
    bool got_laser;

    tf2_ros::Buffer* tfBuffer;
    tf2_ros::TransformListener* tf_listener;

    std::unique_ptr<ros::NodeHandle> n;
    ros::Publisher velocity_pub;

    ros::Subscriber map_sub;
    ros::Subscriber laser_sub;

    PointCloud map_objects_msg;
    sensor_msgs::LaserScan laser_msg;
};

int main(int argc, char** argv) {
    PlayNode playNode(argc, argv);

    // 20 Hz loop
    ros::Rate r(20);
    while (ros::ok()) {
        ROS_INFO("%d", playNode.got_messages());

        if (playNode.got_messages()) {
            playNode.process_messages();
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
