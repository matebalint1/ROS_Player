#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Vector3.h"
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

enum Robot_state { drive_to, drive_random, rotate, move, stop };

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

        // ROS_INFO("Waiting for map_node/map");
        // ros::topic::waitForMessage<PointCloud>("map_node/map");

        map_sub =
            n->subscribe("map_node/map", 1, &PlayNode::map_callback, this);
        collision_avoidance_cloud_sub =
            n->subscribe("map_node/map", 1,
                         &PlayNode::collision_avoidance_cloud_callback, this);

        laser_sub = n->subscribe("robot1/front_laser/scan", 1,
                                 &PlayNode::laser_callback, this);

        field_width_sub = n->subscribe("field_width_node/width", 1,
                                       &PlayNode::field_width_callback, this);

        // odometry_sub =
        //    n->subscribe("robot1/odom", 1, &PlayNode::odometry_callback,
        //    this);
    }

    void tf_map_to_odom_boardcaster(double x, double y, double yaw) {
        static tf::TransformBroadcaster transform_broadcaster;
        // Quaternion from yaw
        geometry_msgs::Quaternion odom_quat =
            tf::createQuaternionMsgFromYaw(yaw);

        // Message
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();

        odom_trans.header.frame_id = "map";
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

    void collision_avoidance_cloud_callback(const PointCloud::ConstPtr& msg) {
        collision_avoidance_cloud_msg = *msg;
        got_collision_avoidance_cloud = true;
    }

    void field_width_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
        geometry_msgs::Vector3 width = *msg;
        field_width = width.x;
        got_field_width = true;
    }

    /*
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        odometry_msg = *msg;
        got_odometry = true;
        // std::cout << msg->pose.pose.position << std::endl;
    }
    */

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

    void collision_avoidance(double& speed_linear, double& speed_rotational,
                             double closest_obstacle_distance,
                             double closest_obstacle_direction,
                             double robot_distance_error,
                             double robot_yaw_error) {
        // Linear speed
        if (closest_obstacle_distance < DISTANCE_LINEAR_STOP) {
            speed_linear = 0;

            ROS_INFO_STREAM("Linear speed mode: COLLISION AVOIDANCE (min)");
        } else if (closest_obstacle_distance < DISTANCE_LINEAR_FREE) {
            speed_linear = (closest_obstacle_distance - DISTANCE_LINEAR_STOP) *
                           (MAX_LINEAR_SPEED) /
                           (DISTANCE_LINEAR_FREE - DISTANCE_LINEAR_STOP);

            // Make robot move with a decet speed when close to stop distance
            speed_linear = fmax(MIN_LINEAR_SPEED, speed_linear);

            ROS_INFO_STREAM("Linear speed mode: CONTROL");
        } else {
            // No obstacle in view
            // if (state == drive_to) {
            //} else {
            // Random
            speed_linear = MAX_LINEAR_SPEED;
            // }
            ROS_INFO_STREAM("Linear speed mode: FREE");
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

            ROS_INFO_STREAM("Angular speed mode: COLLISION AVOIDANCE (max)");
        } else if (closest_obstacle_distance < DISTANCE_FREE_ROTATION) {
            speed_rotational =
                (-closest_obstacle_distance + DISTANCE_FREE_ROTATION) *
                (MAX_ROTATIONAL_SPEEED) /
                (DISTANCE_FREE_ROTATION - DISTANCE_MAX_ROTATION);

            speed_rotational = -sign_of_rotation * fabs(speed_rotational);

            ROS_INFO_STREAM("Angular speed mode: CONTROL");
        } else {
            // No obstacle in view
            // if (state == drive_to) {
            //} else {
            // Random
            speed_rotational = 0;
            // }
            ROS_INFO_STREAM("Angular speed mode: FREE");
        }
    }

    bool is_obstacle_between_robot_and_goal(double closest_obstacle_distance,
                                            double closest_obstacle_direction,
                                            double robot_distance_error,
                                            double robot_yaw_error) {
        // Checks if obstacle in way when driving directly to goal. The
        // calculation is done using a rectancular box (same width as robot safe
        // zone).

        double angle_diff = fabs(closest_obstacle_direction - robot_yaw_error);
        double obstacle_x = closest_obstacle_distance * cos(angle_diff);
        double obstacle_y = closest_obstacle_distance * sin(angle_diff);

        if (obstacle_x > 0 && obstacle_x < robot_distance_error &&
            obstacle_y > -ROBOT_SAFE_ZONE_WIDTH / 2 &&
            obstacle_y < ROBOT_SAFE_ZONE_WIDTH / 2) {
            // obstacle inside of rectangle
            return true;
        } else {
            return false;
        }
    }

    void set_speeds_drive_to(double& speed_linear, double& speed_rotational,
                             double closest_obstacle_distance,
                             double closest_obstacle_direction,
                             double robot_distance_error,
                             double robot_yaw_error) {
        if (is_obstacle_between_robot_and_goal(
                closest_obstacle_distance, closest_obstacle_direction,
                robot_distance_error, robot_yaw_error)) {
            // Obstacle in between goal and robot
            collision_avoidance(speed_linear, speed_rotational,
                                closest_obstacle_distance,
                                closest_obstacle_direction,
                                robot_distance_error, robot_yaw_error);
        } else {
            // Obstacle not in between goal and robot -> drive direcly to goal

            if ((robot_yaw_error > -MAX_YAW_ERROR_WHEN_DRIVING_TO_GOAL &&
                 robot_yaw_error < MAX_YAW_ERROR_WHEN_DRIVING_TO_GOAL) ||
                (robot_distance_error > 0.6 &&
                 closest_obstacle_distance > 0.6)) {
                // Yaw error is small enough or distance error is large
                // enough
                speed_linear = fmax(MIN_LINEAR_SPEED, MAX_LINEAR_SPEED /
                                                          DISTANCE_LINEAR_FREE *
                                                          robot_distance_error);
            } else {
                // Do not drive in the wrong direction when close to goal
                speed_linear = 0;
            }

            speed_rotational = 2 * MAX_ROTATIONAL_SPEEED /
                               DISTANCE_FREE_ROTATION * robot_yaw_error;
        }
    }

    void process_messages() {
        // -------------------------------------------------
        // Get required transformations
        // -------------------------------------------------

        tf::Transform transform_laser_to_baselink;
        bool succesful_laser_tf =
            get_transform(transform_laser_to_baselink, tfBuffer,
                          "robot1/front_laser", "robot1/base_link");
        tf::Transform transform_odom_to_baselink;
        bool succesful_odom_baselink = get_transform(transform_odom_to_baselink, tfBuffer,
                                       "robot1/odom", "robot1/base_link");
        tf::Transform transform_base_link_to_map;
        bool succesful_robot_pos_tf = get_transform(
            transform_base_link_to_map, tfBuffer, "map", "robot1/base_link");

        if (succesful_laser_tf == false || succesful_odom_baselink == false) {
            ROS_INFO_STREAM("Laser or odom to baselink transformation missing!");
            // return;
        }

        if (succesful_robot_pos_tf == false || got_field_width == false) {
            ROS_INFO_STREAM("Map to Odom transformation missing, rotating.");
            // Make robot rotate untill location and field width found.
            set_velocities(0, MAX_ROTATIONAL_SPEEED);
            return;
        }

        tf::Matrix3x3 rotation(transform_base_link_to_map.getRotation());
        double roll, pitch, yaw;
        rotation.getRPY(roll, pitch, yaw);

        ROS_INFO_STREAM("Robot position x,y,yaw "
                        << transform_base_link_to_map.getOrigin().getX() << "\t"
                        << transform_base_link_to_map.getOrigin().getY() << "\t"
                        << yaw);

        // -------------------------------------------------
        // Robot position in the map frame
        // -------------------------------------------------

        double robot_map_x =
            transform_base_link_to_map.getOrigin().getX();  // m, in map frame
        double robot_map_y =
            transform_base_link_to_map.getOrigin().getY();  // m, in map frame
        double robot_map_yaw = yaw;                         // rad in map frame

        if (robot_map_last_x == -1 && robot_map_last_y == -1) {
            // Initialize
            robot_map_last_x = robot_map_x;
            robot_map_last_y = robot_map_y;
            robot_map_last_yaw = robot_map_yaw;
        }

        // -------------------------------------------------
        // Prepare kinect collision avoidance data
        // -------------------------------------------------
        PointCloudPtr collision_avoidance_cloud(new PointCloud);

        // Transform to baselink frame
        pcl_ros::transformPointCloud(collision_avoidance_cloud_msg,
                                     *collision_avoidance_cloud,
                                     transform_odom_to_baselink);

        // -------------------------------------------------
        // Prepare laser collision avoidance data
        // -------------------------------------------------

        // save_cloud_to_file(temp, "/home/cnc/Desktop/Hockey/laser_old.pcd");

        // Convert laser to pointcloud
        sensor_msgs::LaserScan cur_laser = laser_msg;
        PointCloudPtr laser_cloud(new PointCloud);

        temp = laser_msg_to_pointcloud(cur_laser);

        // Transform cloud to base_link frame
        pcl_ros::transformPointCloud(*temp, *laser_cloud,
                                     transform_laser_to_baselink);

        // save_cloud_to_file(temp, "/home/cnc/Desktop/Hockey/laser_new.pcd");

        // -------------------------------------------------
        // Find closest obstacle in combined laser and kinect cloud
        // -------------------------------------------------
        *collision_avoidance_cloud += *laser_cloud; // combine data

        // Get closest object
        double closest_laser_x;  // in base_link frame
        double closest_laser_y;  // in base_link frame
        get_closest_object_in_laser(collision_avoidance_cloud, closest_laser_x,
                                    closest_laser_y);

        // Transform point to distance,direction
        double closest_laser_distance;   // in base_link frame
        double closest_laser_direction;  // in base_link frame
        to_polar(closest_laser_x, closest_laser_y, closest_laser_distance,
                 closest_laser_direction);

        ROS_INFO_STREAM("Closest laser obstacle r: "
                        << closest_laser_distance
                        << "\tangle: " << closest_laser_direction);

        // -------------------------------------------------
        // Calculate closest wall to robot of the field
        // -------------------------------------------------

        // Get closest wall of the field intersecting the safe zone of the
        // robot.
        double closest_wall_x;  // in map frame
        double closest_wall_y;  // in map frame
        get_closest_wall(robot_map_x, robot_map_y, robot_map_yaw,
                         closest_wall_x, closest_wall_y);

        // Get distance and angle of the wall in base_link frame.
        double closest_wall_distance;
        double closest_wall_direction;
        to_polar(closest_wall_x, closest_wall_y, closest_wall_distance,
                 closest_wall_direction, robot_map_x, robot_map_y,
                 robot_map_yaw);

        ROS_INFO_STREAM("Closest wall  obstacle r: " << closest_wall_distance
                                                     << "\tangle: "
                                                     << closest_wall_direction);

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
        } else if (state == rotate) {
            // Update rotation to go
            int sign2 = (rotation_to_go >= 0) - (rotation_to_go < 0);
            if ((robot_map_yaw >= 0 && robot_map_last_yaw >= 0) ||
                (robot_map_yaw < 0 && robot_map_last_yaw < 0)) {
                rotation_to_go -= robot_map_yaw - robot_map_last_yaw;
            } else {
                // Handle edge cases between -pi,pi and -0,0 properly
                if ((rotation_to_go >= 0)) {
                    if (robot_map_yaw >= 0 && robot_map_last_yaw < 0) {
                        rotation_to_go -= robot_map_yaw - robot_map_last_yaw;
                    } else {
                        rotation_to_go -=
                            2 * 3.1415 + robot_map_yaw - robot_map_last_yaw;
                    }
                } else {
                    if (robot_map_yaw >= 0 && robot_map_last_yaw < 0) {
                        rotation_to_go +=
                            2 * 3.1415 - (robot_map_yaw - robot_map_last_yaw);
                    } else {
                        rotation_to_go += -robot_map_yaw + robot_map_last_yaw;
                    }
                }
            }

            if (rotation_to_go < 0.1 && rotation_to_go > -0.1) {
                // Goal reached
                rotation_to_go = 0;
                state = stop;
            }
        } else if (state == move) {
            int sign = (distance_to_go >= 0) - (distance_to_go < 0);
            distance_to_go -= sign * distance_between_points(
                                         robot_map_x, robot_map_y,
                                         robot_map_last_x, robot_map_last_y);

            if (distance_to_go < DISTANCE_GOAL_REACHED &&
                distance_to_go > -DISTANCE_GOAL_REACHED) {
                // Goal reached
                distance_to_go = 0;
                state = stop;
            }
        } else if (state == stop) {
        }

        // Calculate speeds based on calculated parameters and robot state
        if (state == drive_random) {
            collision_avoidance(speed_linear, speed_rotational,
                                closest_obstacle_distance,
                                closest_obstacle_direction,
                                robot_distance_error, robot_yaw_error);
            ROS_INFO_STREAM("Robot state DRIVE RANDOM");

        } else if (state == drive_to) {
            set_speeds_drive_to(speed_linear, speed_rotational,
                                closest_obstacle_distance,
                                closest_obstacle_direction,
                                robot_distance_error, robot_yaw_error);
            ROS_INFO_STREAM("Robot state DRIVE TO");

        } else if (state == rotate) {
            speed_linear = 0;
            speed_rotational = rotation_to_go * 2 * MAX_ROTATIONAL_SPEEED /
                               DISTANCE_FREE_ROTATION;
            ROS_INFO_STREAM(
                "Robot state ROTATE, to rotate: " << rotation_to_go);

        } else if (state == move) {
            int sign = (distance_to_go >= 0) - (distance_to_go < 0);
            speed_linear = sign * fmax(MIN_LINEAR_SPEED,
                                       fabs(distance_to_go) * MAX_LINEAR_SPEED /
                                           DISTANCE_LINEAR_FREE);
            speed_rotational = 0;
            ROS_INFO_STREAM(
                "Robot state MOVE, distance to move: " << distance_to_go);

        } else if (state == stop) {
            speed_linear = 0;
            speed_rotational = 0;

            ROS_INFO_STREAM("Robot state STOP");
        }

        ROS_INFO_STREAM("Error to goal r: " << robot_distance_error
                                            << "\tangle: " << robot_yaw_error);

        ROS_INFO_STREAM("Closest control obstacle r: "
                        << closest_obstacle_distance
                        << "\tangle: " << closest_obstacle_direction);

        ROS_INFO_STREAM("Rotational out: " << speed_rotational);
        ROS_INFO_STREAM("Linear out: "
                        << speed_linear
                        << "\n************************************");

        // -------------------------------------------------
        // Publish speed commands
        // -------------------------------------------------

        set_velocities(
            fmax(-MAX_LINEAR_SPEED, fmin(speed_linear, MAX_LINEAR_SPEED)),
            fmax(-MAX_ROTATIONAL_SPEEED,
                 fmin(speed_rotational, MAX_ROTATIONAL_SPEEED)));

        // Reset
        got_map = false;
        got_laser = false;
        // got_odometry = false;

        robot_map_last_x = robot_map_x;
        robot_map_last_y = robot_map_y;
        robot_map_last_yaw = robot_map_yaw;
    }

    bool got_messages() const {
        return got_laser && got_laser && got_collision_avoidance_cloud;
    }

   private:
    bool got_odometry = false;
    bool got_map = false;
    bool got_laser = false;
    bool got_field_width = false;
    bool got_collision_avoidance_cloud = false;

    tf2_ros::Buffer* tfBuffer;
    tf2_ros::TransformListener* tf_listener;

    std::unique_ptr<ros::NodeHandle> n;
    ros::Publisher velocity_pub;

    ros::Subscriber map_sub;
    ros::Subscriber collision_avoidance_cloud_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber field_width_sub;
    // ros::Subscriber odometry_sub;

    PointCloudPtr temp = PointCloudPtr(new PointCloud);

    PointCloud map_objects_msg;
    PointCloud collision_avoidance_cloud_msg;

    sensor_msgs::LaserScan laser_msg;
    nav_msgs::Odometry odometry_msg;

    // --------------------------------------------
    // Play field
    // --------------------------------------------

    double field_width = 3;                     // m
    double field_length = field_width * 5 / 3;  // m

    // --------------------------------------------
    // Speed settings
    // --------------------------------------------

    // Real driving
    const double MAX_LINEAR_SPEED = 0.3;       // m/s
    const double MAX_ROTATIONAL_SPEEED = 0.3;  // rad/s
    const double MIN_LINEAR_SPEED = 0.08;      // m/s

    // For simulation
    //    const double MAX_LINEAR_SPEED = 0.6;       // m/s
    //    const double MAX_ROTATIONAL_SPEEED = 0.4;  // rad/s
    // const double MIN_LINEAR_SPEED = 0.1;       // m/s

    // --------------------------------------------
    // Collision Avoidance
    // --------------------------------------------

    // Safety zone
    const double ROBOT_SAFE_ZONE_WIDTH = 0.5;   // m
    const double ROBOT_SAFE_ZONE_LENGTH = 0.6;  // m, not relevant anymore

    // Linear
    const double DISTANCE_LINEAR_STOP = 0.7;  // m
    const double DISTANCE_LINEAR_FREE = 1.3;  // m

    // Rotational
    const double DISTANCE_MAX_ROTATION = 0.75 + 0.1;  // m
    const double DISTANCE_FREE_ROTATION = 1.3;        // m

    // Robot behaviour control
    const double DISTANCE_GOAL_REACHED = 0.1;                             // m
    const double MAX_YAW_ERROR_WHEN_DRIVING_TO_GOAL = 10 * 3.1415 / 180;  // rad

    // --------------------------------------------
    // Robot state parameters
    // --------------------------------------------

    // Robot_state state = drive_random;//drive_to//rotate//move;
    Robot_state state = drive_random;

    // Drive to parameters
    double goal_point_x = 0.6;  // map frame
    double goal_point_y = 0.6;  // map frame

    // Rotate parameters
    double rotation_to_go = 3 * 3.14;  // rad

    // Move parameters
    double distance_to_go = 2;  // m

    // Robot last positon
    double robot_map_last_x = -1;
    double robot_map_last_y = -1;
    double robot_map_last_yaw = 0;
};

int main(int argc, char** argv) {
    PlayNode playNode(argc, argv);

    // 20 Hz loop
    ros::Rate r(20);
    while (ros::ok()) {
        // ROS_INFO("%d", playNode.got_messages());
        //        playNode.tf_map_to_odom_boardcaster(1.5, 2.5, 0);  // for
        //        debugging
        if (playNode.got_messages()) {
            playNode.process_messages();
        }

        ros::spinOnce();
        r.sleep();
    }
    playNode.set_velocities(0, 0);
    return 0;
}
