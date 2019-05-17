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

#include <algorithm>

#include "pointcloud_helpers.hpp"

class PlayNode {
   public:
    PlayNode(int argc, char** argv) {
        ros::init(argc, argv, "play_node");
        n = std::make_unique<ros::NodeHandle>();

        tfBuffer = new tf2_ros::Buffer(ros::Duration(100));
        tf_listener = new tf2_ros::TransformListener(*tfBuffer);

        velocity_pub =
            n->advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1000);

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

    void get_intersection_beween_point_pair(double pa1_x, double pa1_y,
                                            double pa2_x, double pa2_y,
                                            double pb1_x, double pb1_y,
                                            double pb2_x, double pb2_y,
                                            double& intersection_x,
                                            double& intersection_y) {
        // Calculate intersection

        double va_x = pa2_x - pa1_x;
        double va_y = pa2_y - pa1_y;

        double vb_x = pb2_x - pb1_x;
        double vb_y = pb2_y - pb1_y;

        double vab_x = pb1_x - pa1_x;
        double vab_y = pb1_y - pa1_y;

        // double va_len = sqrt(va_x * va_x + va_y * va_y);
        // double vb_len = sqrt(vb_x * vb_x + vb_y * vb_y);

        double det = -vb_x * va_y + vb_y * va_x;
        if (det == 0) {
            // no intersection
            intersection_x = -1;
            intersection_y = -1;
            return;
        }
        // double c1 = (-vab_y * va_x + va_y * vab_x) / det;
        double c2 = (-vb_x * vab_y + vb_y * vab_x) / det;
        if (c2 < 0) {
            // intersection on wrong side
            intersection_x = -1;
            intersection_y = -1;
            return;
        }

        intersection_x = pa1_x + c2 * va_x;
        intersection_y = pa1_y + c2 * va_y;
    }

    double distance_between_ponts(double p1_x, double p1_y, double p2_x,
                                  double p2_y) {
        return sqrt((p1_x - p2_x) * (p1_x - p2_x) +
                    (p1_y - p2_y) * (p1_y - p2_y));
    }

    void get_closest_wall(double robot_map_x, double robot_map_y,
                          double robot_map_yaw, double& closest_intersection_x,
                          double& closest_intersection_y) {
        // Calculate intersection points between clear zone rectangle and field
        // borders

        closest_intersection_x = 100;  // outside of the field
        closest_intersection_y = 100;

        // Field corner points
        double field_p1_x = 0;
        double field_p1_y = 0;

        double field_p2_x = 0;
        double field_p2_y = field_length;

        double field_p3_x = field_width;
        double field_p3_y = field_length;

        double field_p4_x = field_width;
        double field_p4_y = 0;

        std::vector<std::vector<double>> field_edge_vectors;
        field_edge_vectors.push_back(std::vector<double>{
            field_p1_x, field_p1_y, field_p2_x, field_p2_y});

        field_edge_vectors.push_back(std::vector<double>{
            field_p3_x, field_p3_y, field_p2_x, field_p2_y});

        field_edge_vectors.push_back(std::vector<double>{
            field_p3_x, field_p3_y, field_p4_x, field_p4_y});

        field_edge_vectors.push_back(std::vector<double>{
            field_p1_x, field_p1_y, field_p4_x, field_p4_y});

        // Robot safe zone corner points
        double safe_zone_pa1_x =
            cos(robot_map_yaw) * 0 -
            sin(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 + robot_map_x;
        double safe_zone_pa1_y =
            sin(robot_map_yaw) * 0 +
            cos(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 + robot_map_y;

        double safe_zone_pa2_x =
            cos(robot_map_yaw) * ROBOT_SAFE_ZONE_LENGTH -
            sin(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 + robot_map_x;
        double safe_zone_pa2_y =
            sin(robot_map_yaw) * ROBOT_SAFE_ZONE_LENGTH +
            cos(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 + robot_map_y;

        double safe_zone_pb1_x =
            cos(robot_map_yaw) * 0 +
            sin(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 + robot_map_x;
        double safe_zone_pb1_y =
            sin(robot_map_yaw) * 0 -
            cos(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 + robot_map_y;

        double safe_zone_pb2_x =
            cos(robot_map_yaw) * ROBOT_SAFE_ZONE_LENGTH +
            sin(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 + robot_map_x;
        double safe_zone_pb2_y =
            sin(robot_map_yaw) * ROBOT_SAFE_ZONE_LENGTH -
            cos(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 + robot_map_y;

        std::vector<std::vector<double>> safe_zone_edge_vectors;

        safe_zone_edge_vectors.push_back(
            std::vector<double>{safe_zone_pa1_x, safe_zone_pa1_y,
                                safe_zone_pa2_x, safe_zone_pa2_y});

        safe_zone_edge_vectors.push_back(
            std::vector<double>{safe_zone_pb1_x, safe_zone_pb1_y,
                                safe_zone_pb2_x, safe_zone_pb2_y});

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                double intersection_x = -1;
                double intersection_y = -1;

                get_intersection_beween_point_pair(
                    safe_zone_edge_vectors[j][0], safe_zone_edge_vectors[j][1],
                    safe_zone_edge_vectors[j][2], safe_zone_edge_vectors[j][3],
                    field_edge_vectors[i][0], field_edge_vectors[i][1],
                    field_edge_vectors[i][2], field_edge_vectors[i][3],
                    intersection_x, intersection_y);

                if (intersection_x != -1 && intersection_y != -1) {
                    double new_distance =
                        distance_between_ponts(intersection_x, intersection_y,
                                               robot_map_x, robot_map_y);

                    double current_distance = distance_between_ponts(
                        closest_intersection_x, closest_intersection_y,
                        robot_map_x, robot_map_y);

                    if (new_distance < current_distance) {
                        closest_intersection_x = intersection_x;
                        closest_intersection_y = intersection_y;
                    }
                }
            }
        }
    }

    PointCloudPtr laser_msg_to_pointcloud(sensor_msgs::LaserScan& laser) {
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

        // Parameters
        const double MIN_DISTANCE = 0.05;  // m
        const double MAX_DISTANCE = 6;     // m

        double angle_min = laser.angle_min;
        double angle_max = laser.angle_max;
        double angle_increment = laser.angle_increment;

        for (int i = 0; i < laser.ranges.size(); i++) {
            double r = laser.ranges[i];
            if (r > MIN_DISTANCE && r < MAX_DISTANCE) {
                // Skip inf
                double angle = angle_min + i * angle_increment;
                PointType point;
                point.x = r * cos(angle);
                point.y = r * sin(angle);
                point.z = 0;
                point.rgb = to_pcl_rgb(255, 255, 255);
            }
        }
        cloud->is_dense = false;
        cloud->width = 1;
        cloud->height = cloud->points.size();
        return cloud;
    }

    void get_closest_object_in_laser(PointCloudPtr& cloud) {}

    void process_messages() {
  
        // Get required transformations
        tf::Transform transform_laser_to_baselink =
            get_transform(tfBuffer, "robot1/front_laser", "robot1/base_link");
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

        // Robot position in the map frame
        double robot_map_x =
            transform_odom_to_map.getOrigin().getX();  // m, in map frame
        double robot_map_y =
            transform_odom_to_map.getOrigin().getY();  // m, in map frame
        double robot_map_yaw = yaw;                    // rad in map frame

        // Navigation goal
        double goal_map_x = 2;  // m, in map frame
        double goal_map_y = 2;  // m, in map frame

        // Calculate closest object in laser message
        sensor_msgs::LaserScan cur_laser = laser_msg;
        PointCloudPtr laser_cloud = laser_msg_to_pointcloud(laser_msg);
        pcl_ros::transformPointCloud(*laser_cloud, *laser_cloud,
                                     transform_laser_to_baselink);
        //get_closest_object_in_laser();

        // Calculate closest object in map

        // Calculate closest wall of field
        double closest_intersection_x;  // in map frame
        double closest_intersection_y;  // in map frame
        get_closest_wall(robot_map_x, robot_map_y, robot_map_yaw,
                         closest_intersection_x, closest_intersection_y);

        std::cout << "Closest wall x: " << closest_intersection_x
                  << " y: " << closest_intersection_y << std::endl;

        // Find closest obstacle of all
        double closest_obstacle_distance = distance_between_ponts(
            closest_intersection_x, closest_intersection_y, robot_map_x,
            robot_map_y);  // m, in base_link frame
        double closest_obstacle_direction =
            atan2(closest_intersection_y - robot_map_y,
                  closest_intersection_x - robot_map_x) -
            robot_map_yaw;  // rad, in base_link frame

        // Calculate desired speed and rotation
        double speed_linear = 0;
        double speed_rotational = 0;

        double robot_yaw_error =
            atan2(goal_map_y - robot_map_y, goal_map_x - robot_map_x);
        double robot_distance_error =
            sqrt((robot_map_x - goal_map_x) * (robot_map_x - goal_map_x) +
                 (robot_map_y - goal_map_y) * (robot_map_y - goal_map_y));

        // std::cout << "yaw error: " << robot_yaw_error << std::endl;
        // std::cout << "distance error: " << robot_distance_error << std::endl;

        std::cout << "yaw to obstacle: " << closest_obstacle_direction
                  << std::endl;
        std::cout << "distance to obstacle: " << closest_obstacle_distance
                  << std::endl;

        if (closest_obstacle_distance < STOP_DISTANCE) {
            speed_linear = 0;
        } else if (closest_obstacle_distance < MAX_SPEED_DISTANCE) {
            speed_linear = (closest_obstacle_distance - STOP_DISTANCE) *
                           (MAX_LINEAR_SPEED) /
                           (MAX_SPEED_DISTANCE - STOP_DISTANCE);
        } else {
            // No obstacle in view
            speed_linear = MAX_LINEAR_SPEED;
        }

        if (closest_obstacle_distance < STOP_DISTANCE_ROT) {
            speed_rotational = MAX_ROTATIONAL_SPEEED;
        } else if (closest_obstacle_distance < MAX_SPEED_DISTANCE_ROT) {
            speed_rotational =
                (-closest_obstacle_distance + MAX_SPEED_DISTANCE_ROT) *
                (MAX_ROTATIONAL_SPEEED) /
                (MAX_SPEED_DISTANCE_ROT - STOP_DISTANCE_ROT);

            double sign_of_rotation = (closest_obstacle_direction >= 0) -
                                      (closest_obstacle_direction < 0);

            speed_rotational = -sign_of_rotation * fabs(speed_rotational);
        } else {
            // No obstacle in view
            speed_rotational = 0;
        }
        std::cout << "rotational out: " << speed_rotational << std::endl;
        std::cout << "linear out: " << speed_linear << " ************"
                  << std::endl;
        // Publish speed commands
        set_velocities(fmax(0, fmin(speed_linear, MAX_LINEAR_SPEED)),
                       fmax(-MAX_ROTATIONAL_SPEEED,
                            fmin(speed_rotational, MAX_ROTATIONAL_SPEEED)));

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

    // const double MAX_LINEAR_SPEED = 0.3;        // m/s
    // const double MAX_ROTATIONAL_SPEEED = 0.2;   // rad/s

    // For debugging
    const double MAX_LINEAR_SPEED = 0.6;       // m/s
    const double MAX_ROTATIONAL_SPEEED = 0.4;  // rad/s

    const double STOP_DISTANCE = 0.65;      // m
    const double MAX_SPEED_DISTANCE = 1.3;  // m

    const double STOP_DISTANCE_ROT = 0.65 + 0.3;      // m
    const double MAX_SPEED_DISTANCE_ROT = 1.3 + 0.3;  // m
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
    playNode.set_velocities(0, 0);
    return 0;
}
