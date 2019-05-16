#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
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
#include <tf/transform_broadcaster.h>
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
        ros::topic::waitForMessage<sensor_msgs::LaserScan>(
            "robot1/front_laser/scan");

        ROS_INFO("Waiting for map_node/map");
        // ros::topic::waitForMessage<PointCloud>("map_node/map");

        map_sub =
            n->subscribe("map_node/map", 1, &PlayNode::map_callback, this);
        laser_sub = n->subscribe("robot1/front_laser/scan", 1,
                                 &PlayNode::laser_callback, this);

        odometry_sub =
            n->subscribe("robot1/odom", 1, &PlayNode::odometry_callback, this);
    }

    void tf_map_to_odom_boardcaster(double x, double y, double yaw) {
        static tf::TransformBroadcaster transform_broadcaster;
        // Quaternion from yaw
        geometry_msgs::Quaternion odom_quat =
            tf::createQuaternionMsgFromYaw(yaw);

        // Message
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();

        odom_trans.header.frame_id = "robot1/map";
        odom_trans.child_frame_id = "robot1/odom";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // Send the transform
        transform_broadcaster.sendTransform(odom_trans);
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

    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        odometry_msg = *msg;
        got_odometry = true;
        // std::cout << msg->pose.pose.position << std::endl;
    }

    void set_velocities(float lin_vel, float ang_vel) const {
        geometry_msgs::Twist msg;
        msg.linear.x = lin_vel;
        msg.angular.z = ang_vel;

        velocity_pub.publish(msg);
    }

    void process_messages() {
        // Copy
        sensor_msgs::LaserScan cur_laser = laser_msg;

        // Get required transformations
        tf::Transform transform_laser_to_odom =
            get_transform(tfBuffer, "robot1/odom", "robot1/front_laser");
        tf::Transform transform_odom_to_baselink =
            get_transform(tfBuffer, "robot1/base_link", "robot1/odom");
        tf::Transform transform_odom_to_map =
            get_transform(tfBuffer, "robot1/map", "robot1/base_link");

        tf::Matrix3x3 rotation(transform_odom_to_map.getRotation());
        double roll, pitch, yaw;
        rotation.getRPY(roll, pitch, yaw);

        std::cout << transform_odom_to_map.getOrigin().getX() << " "
                  << transform_odom_to_map.getOrigin().getY() << " " << yaw
                  << std::endl;

        // Calculate closest object in laser message

        // Calculate closest object in map

        // Calculate closest wall of field

        // Find closest obstacle of all
        double closest_obstacle_distance = 1;   // m, in base_link frame
        double closest_obstacle_direction = 0;  // rad, in base_link frame

        double goal_x = 2;  // m, in map frame
        double goal_y = 2;  // m, in map frame

        // Calculate desired speed and rotation
        double speed_linear = 0;
        double speed_rotational = 0;

        if (closest_obstacle_distance < 0.65) {
            speed_linear = 0;
        } else if (closest_obstacle_distance < 1.3) {
            speed_linear = (closest_obstacle_distance - STOP_DISTANCE) *
                           (MAX_LINEAR_SPEED) /
                           (MAX_SPEED_DISTANCE - STOP_DISTANCE);
        } else {
            speed_linear = MAX_LINEAR_SPEED;
        }

        if (closest_obstacle_distance < 0.65) {
            speed_rotational = MAX_ROTATIONAL_SPEEED;
        } else if (closest_obstacle_distance < 1.3) {
            speed_rotational =
                (closest_obstacle_distance - MAX_SPEED_DISTANCE) *
                (MAX_ROTATIONAL_SPEEED) / (MAX_SPEED_DISTANCE - STOP_DISTANCE);

            speed_rotational =
                sign(closest_obstacle_direction) * abs(speed_rotational);
        } else {
            speed_rotational = 0;
        }

        // Publish speed commands
        set_velocities(max(0, min(speed_linear, MAX_LINEAR_SPEED)),
                       max(-MAX_ROTATIONAL_SPEEED,
                           min(speed_rotational, MAX_ROTATIONAL_SPEEED)));

        got_map = false;
        got_laser = false;
        got_odometry = false;
    }

    bool got_messages() const { return got_laser && got_laser && got_odometry; }

   private:
    bool got_odometry;
    bool got_map;
    bool got_laser;

    tf2_ros::Buffer* tfBuffer;
    tf2_ros::TransformListener* tf_listener;

    std::unique_ptr<ros::NodeHandle> n;
    ros::Publisher velocity_pub;

    ros::Subscriber map_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber odometry_sub;

    PointCloud map_objects_msg;
    sensor_msgs::LaserScan laser_msg;
    nav_msgs::Odometry odometry_msg;

    double field_width = 3;                     // m
    double field_length = field_width * 5 / 3;  // m

    const double ROBOT_SAFE_ZONE_WIDTH = 0.5;   // m
    const double ROBOT_SAFE_ZONE_LENGTH = 0.6;  // m
    const double MAX_LINEAR_SPEED = 0.3;        // m/s
    const double MAX_ROTATIONAL_SPEEED = 0.2;   // rad/s
    const double STOP_DISTANCE = 0.65;          // m
    const double MAX_SPEED_DISTANCE = 1.3;      // m
};

int main(int argc, char** argv) {
    PlayNode playNode(argc, argv);

    // 20 Hz loop
    ros::Rate r(20);
    while (ros::ok()) {
        // ROS_INFO("%d", playNode.got_messages());
        playNode.tf_map_to_odom_boardcaster(1.5, 2.5, 0);
        if (playNode.got_messages()) {
            playNode.process_messages();
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
