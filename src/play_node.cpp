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

enum Robot_state { drive_to, drive_random, stop };

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

    double distance_between_points(double p1_x, double p1_y, double p2_x,
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
                        distance_between_points(intersection_x, intersection_y,
                                                robot_map_x, robot_map_y);

                    double current_distance = distance_between_points(
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
        const double MIN_DISTANCE = 0.1;  // m
        const double MAX_DISTANCE = 6;    // m

        double angle_min = laser.angle_min;
        double angle_max = laser.angle_max;
        double angle_increment = laser.angle_increment;

        if (angle_max != 0) {  // Do not read empty messages
            for (int i = 0; i < 360; i++) {
                // std::cout<<"laser i"<<i<< std::endl;
                // std::cout << laser.ranges[i] << std::endl;
                double r = laser.ranges[i];

                if (r > MIN_DISTANCE && r < MAX_DISTANCE) {
                    // Skip inf
                    double angle = angle_min + i * angle_increment;
                    PointType point;
                    point.x = r * cos(angle);
                    point.y = r * sin(angle);
                    point.z = 0;
                    point.rgb = to_pcl_rgb(255, 255, 255);
                    cloud->points.push_back(point);
                }
            }
        }

        cloud->is_dense = false;
        cloud->width = 1;
        cloud->height = cloud->points.size();
        return cloud;
    }

    void get_closest_object_in_laser(PointCloudPtr& cloud,
                                     double& closest_laser_x,
                                     double& closest_laser_y) {
        // Returns closest point to the robot that is within the safe zone of
        // the robot, if no objects are close a large value is returned.

        closest_laser_x = 100;
        closest_laser_y = 100;
        double last_distance = 140;  // reduces calculations

        for (int i = 0; i < cloud->points.size(); i++) {
            double p_x = cloud->points[i].x;
            double p_y = cloud->points[i].y;

            if (p_y <= ROBOT_SAFE_ZONE_WIDTH / 2 &&
                p_y >= -ROBOT_SAFE_ZONE_WIDTH / 2 && p_x >= 0 /*&&
                p_x <= ROBOT_SAFE_ZONE_LENGTH*/) {
                // Inside safezone

                // Calculate distance to origo
                double distance = distance_between_points(p_x, p_y, 0, 0);
                if (distance < last_distance) {
                    closest_laser_x = p_x;
                    closest_laser_y = p_y;
                    last_distance = distance;
                }
            }
        }
    }

    void to_polar(double p_in_x, double p_in_y, double& p_out_r,
                  double& p_out_a, double origin_x = 0, double origin_y = 0,
                  double origin_rotation = 0) {
        // Transform (x,y) point to (r,angle) in relation to a origin point x,y
        // and rotation of the origin frame around the z-axis.

        p_out_r = distance_between_points(p_in_x, p_in_y, origin_x, origin_y);

        // Calculate angle, make sure that angle is between [-pi,pi]
        p_out_a = atan2(p_in_y - origin_y, p_in_x - origin_x) - origin_rotation;
        p_out_a = atan2(sin(p_out_a), cos(p_out_a));
    }

    void set_speeds(double& speed_linear, double& speed_rotational,
                    double closest_obstacle_distance,
                    double closest_obstacle_direction,
                    double robot_distance_error, double robot_yaw_error) {
        // Linear speed
        if (closest_obstacle_distance < DISTANCE_LINEAR_STOP) {
            speed_linear = 0;

            std::cout << "Linear speed mode: COLLISION AVOIDANCE (min)"
                      << std::endl;
        } else if (closest_obstacle_distance < DISTANCE_LINEAR_FREE) {
            speed_linear = (closest_obstacle_distance - DISTANCE_LINEAR_STOP) *
                           (MAX_LINEAR_SPEED) /
                           (DISTANCE_LINEAR_FREE - DISTANCE_LINEAR_STOP);

            // Make robot move with a decet speed when close to stop distance
            speed_linear = fmax(MIN_LINEAR_SPEED, speed_linear);

            std::cout << "Linear speed mode: CONTROL" << std::endl;
        } else {
            // No obstacle in view
            if (state == drive_to) {
                speed_linear = fmax(MIN_LINEAR_SPEED, MAX_LINEAR_SPEED /
                                                          DISTANCE_LINEAR_FREE *
                                                          robot_distance_error);
            } else {
                // Random
                speed_linear = MAX_LINEAR_SPEED;
            }

            std::cout << "Linear speed mode: FREE" << std::endl;
        }

        // Rotational speed
        double sign_of_rotation = (closest_obstacle_direction >= 0) -
                                  (closest_obstacle_direction < 0);
        if (closest_obstacle_distance < DISTANCE_MAX_ROTATION) {
            // Avoid rotational oscillations
            if (speed_linear == 0) {
                // Always same direction
                speed_rotational = MAX_ROTATIONAL_SPEEED;
            } else {
                speed_rotational = -sign_of_rotation * MAX_ROTATIONAL_SPEEED;
            }

            std::cout << "Angular speed mode: COLLISION AVOIDANCE (max)"
                      << std::endl;
        } else if (closest_obstacle_distance < DISTANCE_FREE_ROTATION) {
            speed_rotational =
                (-closest_obstacle_distance + DISTANCE_FREE_ROTATION) *
                (MAX_ROTATIONAL_SPEEED) /
                (DISTANCE_FREE_ROTATION - DISTANCE_MAX_ROTATION);

            speed_rotational = -sign_of_rotation * fabs(speed_rotational);

            std::cout << "Angular speed mode: CONTROL" << std::endl;
        } else {
            // No obstacle in view
            if (state == drive_to) {
                speed_rotational = MAX_ROTATIONAL_SPEEED /
                                   DISTANCE_FREE_ROTATION * robot_yaw_error;
            } else {
                // Random
                speed_rotational = 0;
            }

            std::cout << "Angular speed mode: FREE" << std::endl;
        }
    }

    void process_messages() {
        // -------------------------------------------------
        // Get required transformations
        // -------------------------------------------------

        tf::Transform transform_laser_to_baselink =
            get_transform(tfBuffer, "robot1/front_laser", "robot1/base_link");
        tf::Transform transform_odom_to_baselink =
            get_transform(tfBuffer, "robot1/base_link", "robot1/odom");
        tf::Transform transform_odom_to_map =
            get_transform(tfBuffer, "robot1/map", "robot1/base_link");

        tf::Matrix3x3 rotation(transform_odom_to_map.getRotation());
        double roll, pitch, yaw;
        rotation.getRPY(roll, pitch, yaw);

        std::cout << "Robot position x,y,yaw "
                  << transform_odom_to_map.getOrigin().getX() << "\t"
                  << transform_odom_to_map.getOrigin().getY() << "\t" << yaw
                  << std::endl;

        // -------------------------------------------------
        // Robot position in the map frame
        // -------------------------------------------------

        double robot_map_x =
            transform_odom_to_map.getOrigin().getX();  // m, in map frame
        double robot_map_y =
            transform_odom_to_map.getOrigin().getY();  // m, in map frame
        double robot_map_yaw = yaw;                    // rad in map frame

        // -------------------------------------------------
        // Calculate closest object in laser message
        // -------------------------------------------------

        // Convert laser to pointcloud
        sensor_msgs::LaserScan cur_laser = laser_msg;
        PointCloudPtr laser_cloud(new PointCloud);
        PointCloudPtr temp = laser_msg_to_pointcloud(cur_laser);

        // Transform cloud to base_link frame
        pcl_ros::transformPointCloud(*temp, *laser_cloud,
                                     transform_laser_to_baselink);

        // save_cloud_to_file(laser_cloud,
        // "/home/cnc/Desktop/Hockey/laser.pcd");

        // Get closest object
        double closest_laser_x;  // in base_link frame
        double closest_laser_y;  // in base_link frame
        get_closest_object_in_laser(laser_cloud, closest_laser_x,
                                    closest_laser_y);

        // Transform point to distance,direction
        double closest_laser_distance;   // in base_link frame
        double closest_laser_direction;  // in base_link frame
        to_polar(closest_laser_x, closest_laser_y, closest_laser_distance,
                 closest_laser_direction);

        std::cout << "Closest laser obstacle r: " << closest_laser_distance
                  << "\tangle: " << closest_laser_direction << std::endl;

        // -------------------------------------------------
        // Calculate closest wall to robot of the field
        // -------------------------------------------------

        // Get closest wall of the field intersecting the safe zone of the
        // robot.
        double closest_wall_x;  // in map frame
        double closest_wall_y;  // in map frame
        get_closest_wall(robot_map_x, robot_map_y, robot_map_yaw,
                         closest_wall_x, closest_wall_y);

        // std::cout << "Closest wall x: " << closest_wall_x
        //          << " y: " << closest_wall_y << std::endl;

        // Get distance and angle of the wall in base_link frame.
        double closest_wall_distance;
        double closest_wall_direction;
        to_polar(closest_wall_x, closest_wall_y, closest_wall_distance,
                 closest_wall_direction, robot_map_x, robot_map_y,
                 robot_map_yaw);

        std::cout << "Closest wall  obstacle r: " << closest_wall_distance
                  << "\tangle: " << closest_wall_direction << std::endl;

        // -------------------------------------------------
        // Calculate closest object in map
        // -------------------------------------------------

        // -------------------------------------------------
        // Calculate closest obstacle of all
        // -------------------------------------------------
        const double DETERMINE_ROTATION_DIRECTION_BASED_ON_WALL_DISTANCE =
            DISTANCE_LINEAR_FREE;  // m

        double closest_obstacle_distance = closest_laser_distance;
        double closest_obstacle_direction = closest_laser_direction;
        if (closest_wall_distance < closest_laser_distance) {
            // Wall closer
            closest_obstacle_distance = closest_wall_distance;
            closest_obstacle_direction = closest_wall_direction;

        } else {
            // Obstacle closer
            closest_obstacle_distance = closest_laser_distance;

            if (closest_wall_distance <
                DETERMINE_ROTATION_DIRECTION_BASED_ON_WALL_DISTANCE) {
                // Even if laser obstacle is closer the direction ofrotation is
                // determined by the wall if it is closer than the threshold
                // distance.
                closest_obstacle_direction = closest_wall_direction;
            } else {
                closest_obstacle_direction = closest_laser_direction;
            }
        }

        // -------------------------------------------------
        // Calculate desired speed and rotation
        // -------------------------------------------------

        double speed_linear = 0;
        double speed_rotational = 0;

        // Robot distance and heading error to goal point
        double robot_distance_error = 0;
        double robot_yaw_error = 0;

        // State and parameter update of robot
        if (state == drive_random) {
        } else if (state == drive_to) {
            // Check if goal is valid or it has been reached already
            if (goal_point_x > 0 && goal_point_y > 0 &&
                goal_point_x < field_width && goal_point_y < field_length) {
                // Valid goal, calculate errors
                to_polar(goal_point_x, goal_point_y, robot_distance_error,
                         robot_yaw_error, robot_map_x, robot_map_y,
                         robot_map_yaw);

                if (robot_distance_error <= DISTANCE_GOAL_REACHED) {
                    // Goal reached
                    state = stop;
                    goal_point_x = -1;
                    goal_point_y = -1;
                } else {
                    // Goal not reached
                    state = drive_to;
                }
            } else {
                // No valid goal point change state to stop
                state = stop;
            }

        } else if (state == stop) {
        }

        // Calculate speeds based on calculated parameters and robot state
        if (state == drive_random) {
            set_speeds(speed_linear, speed_rotational,
                       closest_obstacle_distance, closest_obstacle_direction,
                       robot_distance_error, robot_yaw_error);
        } else if (state == drive_to) {
            set_speeds(speed_linear, speed_rotational,
                       closest_obstacle_distance, closest_obstacle_direction,
                       robot_distance_error, robot_yaw_error);
        } else if (state == stop) {
            speed_linear = 0;
            speed_rotational = 0;
        }

        // std::cout << "Yaw error: " << robot_yaw_error << std::endl;
        // std::cout << "Distance error: " << robot_distance_error << std::endl;

        std::cout << "Closest control obstacle r: " << closest_obstacle_distance
                  << "\tangle: " << closest_obstacle_direction << std::endl;

        std::cout << "Rotational out: " << speed_rotational << std::endl;
        std::cout << "Linear out: " << speed_linear
                  << "\n************************************" << std::endl;

        // -------------------------------------------------
        // Publish speed commands
        // -------------------------------------------------

        set_velocities(fmax(0, fmin(speed_linear, MAX_LINEAR_SPEED)),
                       fmax(-MAX_ROTATIONAL_SPEEED,
                            fmin(speed_rotational, MAX_ROTATIONAL_SPEEED)));

        // Reset
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

    // Real driving
    // const double MAX_LINEAR_SPEED = 0.3;        // m/s
    // const double MAX_ROTATIONAL_SPEEED = 0.2;   // rad/s

    // For simulation
    const double MAX_LINEAR_SPEED = 0.6;       // m/s
    const double MAX_ROTATIONAL_SPEEED = 0.4;  // rad/s
    const double MIN_LINEAR_SPEED = 0.1;       // m/s

    // Linear
    const double DISTANCE_LINEAR_STOP = 0.5;  // m
    const double DISTANCE_LINEAR_FREE = 1.2;  // m

    // Rotational
    const double DISTANCE_MAX_ROTATION = 0.65 + 0.1;  // m
    const double DISTANCE_FREE_ROTATION = 1.2;        // m

    // Robot behaviour control
    const double DISTANCE_GOAL_REACHED = 0.1;
    Robot_state state = drive_to;
    double goal_point_x = 1;  // map frame
    double goal_point_y = 1;  // map frame
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
