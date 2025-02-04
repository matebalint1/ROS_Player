#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "player/SendColor.h"
#include "player/SendDimensions.h"
#include "player/TeamReady.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "player/TeamReady.h" //../../../devel/include/
#include "player/SendColor.h"
#include "player/SendDimensions.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cstring>
#include "pointcloud_helpers.hpp"

#define use_referee

enum Robot_state { initialize, drive_to, drive_random, rotate, move, stop };
enum Game_state {
    wait_for_start,
    initialize_location,
    look_for_puck,
    drive_to_puck,
    drive_with_puck_to_goal,
    leave_buck_in_goal
};

bool got_odometry = false;
bool got_map = false;
bool got_laser = false;
bool got_field_width = false;
bool got_collision_avoidance_cloud = false;

tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tf_listener;

std::unique_ptr<ros::NodeHandle> n;
ros::Publisher velocity_pub;
ros::Publisher debug_cloud_pub;

ros::Subscriber wait_for_teams_sub;
ros::Subscriber game_control_sub;
ros::Subscriber map_sub;
ros::Subscriber collision_avoidance_cloud_sub;
ros::Subscriber laser_sub;
ros::Subscriber field_width_sub;

ros::ServiceClient team_ready_client;
ros::ServiceClient send_color_client;
ros::ServiceClient send_dimensions_client;

PointCloudPtr temp = PointCloudPtr(new PointCloud);
PointCloudPtr map_cloud_in_map_frame(new PointCloud);
PointCloudPtr laser_cloud(new PointCloud);
PointCloudPtr collision_avoidance_cloud(new PointCloud);

PointCloud map_objects_msg;
PointCloud collision_avoidance_cloud_msg;

sensor_msgs::LaserScan laser_msg;
nav_msgs::Odometry odometry_msg;

bool game_started_msg = false;
int team_number = 1;

// --------------------------------------------
// Play field size and game settings
// --------------------------------------------

double field_width = 3;                         // m
double field_length = field_width * 5.0 / 3.0;  // m
const int NUMBER_OF_BUCKS_PER_TEAM = 3;

// --------------------------------------------
// Speed settings
// --------------------------------------------

// Real driving
const double MAX_LINEAR_SPEED = 0.3;       // m/s
const double MAX_ROTATIONAL_SPEEED = 0.5;  // rad/s
const double MIN_LINEAR_SPEED = 0.15;      // m/s // 0.08 works
const double MIN_ROTATIONAL_SPEED = 0.2;

// --------------------------------------------
// Collision Avoidance
// --------------------------------------------

// Safety zone
const double ROBOT_SAFE_ZONE_WIDTH = 0.6;   // m 0.5 tested and works
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
Robot_state state = stop;

// This offset changes depending of the speed of the robot
// between values 0 to 0.2 m to avoid stopping in collisions.
double safety_zone_x_offset = 0;
const double SAFETY_ZONE_X_OFFSET_MAX = 0.4;  // m

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

// Robot driving parameters updated by process_messages and needed in
// update_robot_state
double robot_map_x = 0;
double robot_map_y = 0;
double robot_map_yaw = 0;
double closest_obstacle_distance_g = 0;
double closest_obstacle_direction_g = 0;

double drive_to_left = 0; // control wheter or not to drive to left or right.

// --------------------------------------------
// Game state parameters
// --------------------------------------------

std::string team_name = "Green peas";

#ifdef use_referee
Game_state game_state = wait_for_start;
#else
Game_state game_state = initialize_location;
#endif

int is_blue_team = -1;  // -1 not set, 0 false, 1 true
int moves_done = 0;  // used for doing move squences, e.g. leaving buck in goal

// Processing
bool succesful_map_tf = false;
tf::Transform transform_map_to_odom;
tf::Transform transform_map_to_baselink;
int tf_delay_counter = 0;

//--------------------------------------------
//--------------------------------------------

std::string addRobotName( std::string s )
{
    std::string final = "robot";
    final.append( std::to_string( team_number ) );
    final.append( s );
    return final;
}

void pub_pointcloud(PointCloud& cloud, ros::Publisher& pub) {
    PointCloudPtr msg(new PointCloud);
    msg->header.frame_id = addRobotName("/base_link");

    msg->height = cloud.height;
    msg->width = cloud.width;
    msg->points = cloud.points;
    pub.publish(msg);
}

/*
void tf_map_to_odom_boardcaster(double x, double y, double yaw) {
    static tf::TransformBroadcaster transform_broadcaster;
    // Quaternion from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

    // Message
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();

    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = addRobotName("/odom");

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // Send the transform
    transform_broadcaster.sendTransform(odom_trans);
}
*/

void game_control_callback(const std_msgs::Bool::ConstPtr& msg) {
    // ROS_INFO("Got game control");
    game_started_msg = msg->data;
    // got_laser = true;
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
    if (!got_field_width) {
        geometry_msgs::Vector3 width = *msg;
        field_width = width.x;
        field_length = 5.0 / 3.0 * field_width;

        player::SendDimensions send_dimensions_srv;
        send_dimensions_srv.request.team = team_name;
        send_dimensions_srv.request.dimensions.x =
            field_length;  // length in METRES!!!! TODO
        send_dimensions_srv.request.dimensions.y =
            field_width;  // width in METRES!!! TODO
        send_dimensions_srv.request.dimensions.z = 0.0;

        if (send_dimensions_client.call(send_dimensions_srv)) {
            if (send_dimensions_srv.response.ok) {
                ROS_INFO("Dimensions are within error margin");
            } else {
                ROS_INFO("Dimensions are NOT within error margin");
            }

            field_width = send_dimensions_srv.response.correctDimensions.y;
            field_length = send_dimensions_srv.response.correctDimensions.x;

            // Update width in the field width node
            n->setParam("correct_width_set", true);
            n->setParam("field_width", field_width);
        } else {
            ROS_ERROR("Failed to call service SendDimensions");
        }
    }

    got_field_width = true;
}

void set_velocities(float lin_vel, float ang_vel) {
    geometry_msgs::Twist msg;
    msg.linear.x = lin_vel;
    msg.angular.z = ang_vel;

    velocity_pub.publish(msg);
}

double distance_between_points(double p1_x, double p1_y, double p2_x,
                               double p2_y) {
    return sqrt((p1_x - p2_x) * (p1_x - p2_x) + (p1_y - p2_y) * (p1_y - p2_y));
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
    double distance_to_intersection =
        distance_between_points(pa1_x, pa1_y, c2 * va_x, c2 * va_y);
    if (c2 < 0 || distance_to_intersection < safety_zone_x_offset) {
        // intersection on wrong side or to close to the robot
        intersection_x = -1;
        intersection_y = -1;
        return;
    }

    intersection_x = pa1_x + c2 * va_x;
    intersection_y = pa1_y + c2 * va_y;
}

void get_closest_wall(double robot_map_x, double robot_map_y,
                      double robot_map_yaw, double& closest_intersection_x,
                      double& closest_intersection_y) {
    // Calculate intersection points between safe zone side lines and field
    // borders.

    closest_intersection_x = 100;  // outside of the field, default value
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
    field_edge_vectors.push_back(
        std::vector<double>{field_p1_x, field_p1_y, field_p2_x, field_p2_y});

    field_edge_vectors.push_back(
        std::vector<double>{field_p3_x, field_p3_y, field_p2_x, field_p2_y});

    field_edge_vectors.push_back(
        std::vector<double>{field_p3_x, field_p3_y, field_p4_x, field_p4_y});

    field_edge_vectors.push_back(
        std::vector<double>{field_p1_x, field_p1_y, field_p4_x, field_p4_y});

    // Robot safe zone corner points
    double safe_zone_pa1_x = cos(robot_map_yaw) * 0 -
                             sin(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 +
                             robot_map_x;
    double safe_zone_pa1_y = sin(robot_map_yaw) * 0 +
                             cos(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 +
                             robot_map_y;

    double safe_zone_pa2_x = cos(robot_map_yaw) * ROBOT_SAFE_ZONE_LENGTH -
                             sin(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 +
                             robot_map_x;
    double safe_zone_pa2_y = sin(robot_map_yaw) * ROBOT_SAFE_ZONE_LENGTH +
                             cos(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 +
                             robot_map_y;

    double safe_zone_pb1_x = cos(robot_map_yaw) * 0 +
                             sin(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 +
                             robot_map_x;
    double safe_zone_pb1_y = sin(robot_map_yaw) * 0 -
                             cos(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 +
                             robot_map_y;

    double safe_zone_pb2_x = cos(robot_map_yaw) * ROBOT_SAFE_ZONE_LENGTH +
                             sin(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 +
                             robot_map_x;
    double safe_zone_pb2_y = sin(robot_map_yaw) * ROBOT_SAFE_ZONE_LENGTH -
                             cos(robot_map_yaw) * ROBOT_SAFE_ZONE_WIDTH / 2 +
                             robot_map_y;

    std::vector<std::vector<double>> safe_zone_edge_vectors;

    safe_zone_edge_vectors.push_back(std::vector<double>{
        safe_zone_pa1_x, safe_zone_pa1_y, safe_zone_pa2_x, safe_zone_pa2_y});

    safe_zone_edge_vectors.push_back(std::vector<double>{
        safe_zone_pb1_x, safe_zone_pb1_y, safe_zone_pb2_x, safe_zone_pb2_y});

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
                double new_distance = distance_between_points(
                    intersection_x, intersection_y, robot_map_x, robot_map_y);

                double current_distance = distance_between_points(
                    closest_intersection_x, closest_intersection_y, robot_map_x,
                    robot_map_y);

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

void get_closest_object_in_laser(PointCloudPtr& cloud, double& closest_laser_x,
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
            p_y >= -ROBOT_SAFE_ZONE_WIDTH / 2 && p_x >= safety_zone_x_offset) {
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

void to_polar(double p_in_x, double p_in_y, double& p_out_r, double& p_out_a,
              double origin_x = 0, double origin_y = 0,
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
                         double robot_distance_error, double robot_yaw_error) {
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
    double sign_of_rotation =
        (closest_obstacle_direction >= 0) - (closest_obstacle_direction < 0);
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
        // Random
        speed_rotational = 0;
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

    // double angle_diff = fabs(closest_obstacle_direction - robot_yaw_error);
    double angle_diff = closest_obstacle_direction - robot_yaw_error;
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

bool goal_point_inside_of_rectancle(double x, double y){
    if(y > -ROBOT_SAFE_ZONE_WIDTH / 2.0 &&
       y < ROBOT_SAFE_ZONE_WIDTH / 2.0){
        return true;
    } else {
        return false;
    }
}

void remove_points_inside_puck_carry_zone(PointCloudPtr& cloud){
    // Delete pucks in front of the robot (== do not avoid pucks in collison avoidance cloud)
    const double ROBOT_WIDTH = 0.4;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<PointType> extract;

    for (int i = 0; i < cloud->points.size(); i++) {
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;

        if ( x <= SAFETY_ZONE_X_OFFSET_MAX && 
             x > -0.2 &&
             y > -ROBOT_WIDTH / 2.0 &&
             y < ROBOT_WIDTH / 2.0) {
            // Remove these points (these points are not dangerous)
            inliers->indices.push_back(i);
        }
    }

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
}

double find_free_drive_direction(int direction, PointCloudPtr& cloud,
                                        double goal_distance){
    // This function returns the smalle rotation value that the robot shoud drive to
    // to avoid obstacles. returns degrees

    // This function edits the point cloud, rotation parameter values 1 or -1, left rotation
    // is positive (== 1);
    
    const double MIN_FREE_SPACE_IN_FRONT_OF_ROBOT = 1.2; //m, threshold for drivable direction
    PointCloudPtr temp(new PointCloud);
    double min_rotation = 0; // degrees

    // Find optimal rotation
    for(int rotation = 0; rotation < 180; rotation++){
        // Precision of one degree

        double goal_point_x_rotated = goal_distance * cos(rotation * 3.1415/180.0);
        double goal_point_y_rotated = goal_distance * sin(rotation * 3.1415/180.0);
        bool goal_inside_rect = goal_point_inside_of_rectancle(goal_point_x_rotated,goal_point_y_rotated); 

        double x_limit;
        if(goal_inside_rect){
            // Add 0.2 to ensure that the region behind the goal is also safe
            x_limit = fmin(goal_point_x_rotated + 0.2, MIN_FREE_SPACE_IN_FRONT_OF_ROBOT);
        } else {
            x_limit = MIN_FREE_SPACE_IN_FRONT_OF_ROBOT;
        }

        bool obstacle_inside = false;
        for(int i = 0; i < cloud->points.size(); i++){
            double x = cloud->points[i].x;
            double y = cloud->points[i].y;

            if (x > 0.0 && x <= x_limit &&
                y > -(ROBOT_SAFE_ZONE_WIDTH + 0.1) / 2.0 &&
                y < (ROBOT_SAFE_ZONE_WIDTH + 0.1) / 2.0) {
                // obstacle inside of rectangle and not closer than the goal point
                obstacle_inside = true;
                break;
            }
        }
        
        min_rotation = rotation;
        if(obstacle_inside == false){
            // First empty region to the left: stop   
            break;
        }
        
        // Rotate cloud by one degree for next iteration
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.translation() << 0, 0, 0;
        transform_2.rotate (Eigen::AngleAxisf (-1.0 * direction * 3.1415/180.0, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud (*cloud, *temp, transform_2);
        *cloud = *temp;
    }

    return min_rotation;
}

void find_free_drive_direction_and_total_space(int direction, PointCloudPtr& cloud, double goal_distance,
                double& total_space, double& min_rotation){
    // This function returns the smallest rotation value that the robot shoud drive to
    // to avoid obstacles. returns degrees of min rotation and total_space

    // This function edits the point cloud, rotation parameter values 1 or -1, left rotation
    // is positive (== 1);
    
    const double MIN_FREE_SPACE_IN_FRONT_OF_ROBOT = 1; //m, threshold for drivable direction
    PointCloudPtr temp(new PointCloud);
    min_rotation = 180; // degrees
    total_space = 0; // integral 0 to 180 degrees

    // Find optimal rotation
    for(int rotation = 0; rotation < 180; rotation += 2){ // 2 degree resolution for speed
        // Precision of one degree

        double goal_point_x_rotated = goal_distance * cos(rotation * 3.1415/180.0);
        double goal_point_y_rotated = goal_distance * sin(rotation * 3.1415/180.0);
        bool goal_inside_rect = goal_point_inside_of_rectancle(goal_point_x_rotated,goal_point_y_rotated); 

        double x_limit;
        if(goal_inside_rect){
            // Add 0.1 to ensure that the region behind the goal is also safe
            x_limit = fmin(goal_point_x_rotated + 0.1, MIN_FREE_SPACE_IN_FRONT_OF_ROBOT);
        } else {
            x_limit = MIN_FREE_SPACE_IN_FRONT_OF_ROBOT;
        }

        bool obstacle_inside = false;
        double smalles_x = 4;
        for(int i = 0; i < cloud->points.size(); i++){
            double x = cloud->points[i].x;
            double y = cloud->points[i].y;

            if(x < smalles_x && x > 0.2){
                // Find closest obstcle in that direction
                smalles_x = x;
            }

            if (x > 0.0 && x <= x_limit && 
                y > -(ROBOT_SAFE_ZONE_WIDTH + 0.1) / 2.0 &&
                y < (ROBOT_SAFE_ZONE_WIDTH + 0.1)  / 2.0) {
                // obstacle inside of rectangle and not closer than the goal point
                obstacle_inside = true;
            }
        }
        total_space += smalles_x; // sum of empty space in all directions
        
        if(obstacle_inside == false && min_rotation == 180){
            // First empty region -> save value 
            min_rotation = rotation;
        }
        
        // Rotate cloud by one degree for next iteration
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.translation() << 0, 0, 0;
        transform_2.rotate (Eigen::AngleAxisf (-2.0 * direction * 3.1415/180.0, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud (*cloud, *temp, transform_2);
        *cloud = *temp;
    }

}

double get_closest_obstacle_distance_to_goal_point(PointCloudPtr& cloud, double goal_distance){
    // Input cloud must be aligned so that x axis points directly to goal
    double closest_obs_distance = 100;

    double goal_y = 0;
    double goal_x = goal_distance;

    for(int i = 0; i < cloud->points.size(); i++){
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;
        double distance = distance_between_points(goal_x, goal_y, x, y);

        if(distance < closest_obs_distance){
            closest_obs_distance = distance;
        }
    }
    return closest_obs_distance;
}

double get_goal_heading_path_planning(double goal_distance,
                                      double goal_heading) {
    // This function calculates the heading for the robot to go around single
    // obstacles. It calculates the first possible direction to the left and
    // right the one with smaller value is choseen as a new heading for the
    // robot.

    // Make copy of cloud
    PointCloudPtr cloud(new PointCloud(*collision_avoidance_cloud));
    remove_points_inside_puck_carry_zone(cloud);
    PointCloudPtr temp(new PointCloud);


    // Generate field edge cloud in map frame
    PointCloudPtr field_edges_map = get_ideal_field_edge_cloud(field_width);

    // Rotate edge cloud to base_link
    pcl_ros::transformPointCloud(*field_edges_map, *temp,
                                transform_map_to_baselink);
    *cloud += *temp; // combine clouds
    
   
    // Rotate cloud so that x-axis points to the direction of the goal
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0, 0, 0;
    transform_2.rotate (Eigen::AngleAxisf (-goal_heading, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*cloud, *temp, transform_2);
    *cloud = *temp;

    // Make copies of each cloud for both directions
    PointCloudPtr cloud_left(new PointCloud(*cloud));
    PointCloudPtr cloud_right(new PointCloud(*cloud));

    /*if(goal_distance < 0.8){ // TODO testing
        double closest_obstacle_to_goal = get_closest_obstacle_distance_to_goal_point(cloud_left,goal_distance);
        if(closest_obstacle_to_goal < 0.1){
            // obstacle too close -> stop robot
            state = stop;
        }
    }*/
    

    double min_rotation_left = find_free_drive_direction(1, cloud_left, goal_distance); // degrees
    double min_rotation_right = find_free_drive_direction(-1, cloud_right, goal_distance); // degrees
    //double min_rotation_left;
    //double min_rotation_right;
    //double total_space_left;
    //double total_space_right;
    //find_free_drive_direction_and_total_space(1, cloud_left, goal_distance, total_space_left, min_rotation_left);
    //find_free_drive_direction_and_total_space(-1, cloud_right, goal_distance, total_space_right, min_rotation_right);

    // Compare rotations to the left and right and transform to radians as heading error
    // of the robot
    double rot_left_total = min_rotation_left * 3.1415 / 180.0 + goal_heading;
    double rot_right_total = -min_rotation_right * 3.1415 / 180.0 + goal_heading;

    std::cout << "Rot to left: " << min_rotation_left << std::endl;
    std::cout << "Rot to right: " << min_rotation_right << std::endl;
    //std::cout << "Total space to left: " << total_space_left << std::endl;
    //std::cout << "Total space to right: " << total_space_right << std::endl;
    std::cout << "Total Rot to left: " << rot_left_total *180.0/3.14 << std::endl;
    std::cout << "Total Rot to right: " << rot_right_total *180.0/3.14  << std::endl;   
    
    //return rot_left_total;

    // Use averaging to stick on the decision to use the same direction
    drive_to_left = 0.97 * drive_to_left + 0.03 * double(min_rotation_left < min_rotation_right);
    if (drive_to_left > 0.5){
        return rot_left_total;
    } else { 
        return rot_right_total; 
    }
}

void set_speeds_drive_to(double& speed_linear, double& speed_rotational,
                         double closest_obstacle_distance,
                         double closest_obstacle_direction,
                         double goal_point_distance, double goal_point_direction) {
    
    double robot_heading_error = get_goal_heading_path_planning(
                                            goal_point_distance, goal_point_direction);

    if (game_state == initialize_location){ 
        // do not use collision avoidance in initialization
        robot_heading_error = goal_point_direction;
    }

    // Set linear speed
    speed_linear = fmax(MIN_LINEAR_SPEED, MAX_LINEAR_SPEED / DISTANCE_LINEAR_FREE *
                                                                    goal_point_distance);

    // Reduce linear speed if heading error is large. A 1/x function is used.
    speed_linear = speed_linear * fmax(0.0, fmin(1.0, 0.1/fmax(0.01, fabs(robot_heading_error))- 1.0/3.1415));

    if(closest_obstacle_distance_g < 0.65){
        // if obstacle too close set linear speed to zero
       // speed_linear = 0;
    } 

    // Set rotational speed
    speed_rotational = 4 * MAX_ROTATIONAL_SPEEED / DISTANCE_FREE_ROTATION *
                         robot_heading_error;

}

void color_filter(PointCloudPtr& cloud_in, PointCloudPtr& cloud_out, int r,
                  int g, int b) {
    // Filters pointcloud by a specific color
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<PointType> extract;

    for (int i = 0; i < cloud_in->points.size(); i++) {
        uint32_t rgb = *reinterpret_cast<int*>(&cloud_in->points[i].rgb);
        uint8_t bp = (rgb >> 0) & 0xff;
        uint8_t gp = (rgb >> 8) & 0xff;
        uint8_t rp = (rgb >> 16) & 0xff;

        if (r == rp && g == gp && b == bp) {
            // Use these points
            inliers->indices.push_back(i);
        }
    }
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.filter(*cloud_out);
}

bool process_messages() {
    // This function prepares data for driving around, returns false if not
    // succesful.

    // -------------------------------------------------
    // Get required transformations
    // -------------------------------------------------

    tf::Transform transform_laser_to_baselink;
    bool succesful_laser_tf =
        get_transform(transform_laser_to_baselink, tfBuffer, addRobotName("/base_link"),
                      addRobotName("/front_laser") );
    tf::Transform transform_odom_to_baselink;
    bool succesful_odom_baselink =
        get_transform(transform_odom_to_baselink, tfBuffer, addRobotName("/base_link"),
                      addRobotName("/odom") );
    tf::Transform transform_base_link_to_map;
    bool succesful_robot_pos_tf = get_transform(
        transform_base_link_to_map, tfBuffer, "map", addRobotName("/base_link") );

    tf::Transform transform_odom_to_map;
    succesful_map_tf =
        get_transform(transform_odom_to_map, tfBuffer, "map", addRobotName("/odom"));

    succesful_map_tf =
        get_transform(transform_map_to_odom, tfBuffer, addRobotName("/odom"), "map");

    get_transform(transform_map_to_baselink, tfBuffer, addRobotName("/base_link"), "map");

    if (succesful_laser_tf == false || succesful_odom_baselink == false) {
        ROS_INFO_STREAM("Laser or odom to baselink transformation missing!");
        // return false;
    }

    // Update robot pos if tf succesful tf:s*********************************
    if(succesful_robot_pos_tf == true && 
        succesful_laser_tf == true && 
        succesful_odom_baselink == true){
        // -------------------------------------------------
        // Prepare laser collision avoidance data
        // -------------------------------------------------

        // Convert laser to pointcloud
        sensor_msgs::LaserScan cur_laser = laser_msg;

        temp = laser_msg_to_pointcloud(cur_laser);

        // Transform cloud to base_link frame
        pcl_ros::transformPointCloud(*temp, *laser_cloud,
                                    transform_laser_to_baselink);

            // Robot position in map frame
        tf::Matrix3x3 rotation(transform_base_link_to_map.getRotation());
        double roll, pitch, yaw;
        rotation.getRPY(roll, pitch, yaw);

        ROS_INFO_STREAM("Robot position x,y,yaw "
                        << transform_base_link_to_map.getOrigin().getX() << " "
                        << transform_base_link_to_map.getOrigin().getY() << " "
                        << yaw);

        // Robot position in odom frame

        // -------------------------------------------------
        // Robot position in the map frame
        // -------------------------------------------------

        robot_map_x =
            transform_base_link_to_map.getOrigin().getX();  // m, in map frame
        robot_map_y =
            transform_base_link_to_map.getOrigin().getY();  // m, in map frame
        robot_map_yaw = yaw;                                // rad in map frame

        if (robot_map_last_x == -1 && robot_map_last_y == -1) {
            // Initialize, first time
            robot_map_last_x = robot_map_x;
            robot_map_last_y = robot_map_y;
            robot_map_last_yaw = robot_map_yaw;
        }

        // -------------------------------------------------
        // Prepare kinect collision avoidance data
        // -------------------------------------------------

        // Transform to baselink frame
        pcl_ros::transformPointCloud(collision_avoidance_cloud_msg,
                                    *collision_avoidance_cloud,
                                    transform_odom_to_baselink);

        *collision_avoidance_cloud += *laser_cloud;  // combine data

        // -------------------------------------------------
        // Prepare map data
        // -------------------------------------------------
        PointCloudPtr map_cloud(new PointCloud);
        PointCloudPtr safe_points(new PointCloud);

        // Transform to baselink frame
        PointCloudPtr temp2(new PointCloud(map_objects_msg));
        pcl_ros::transformPointCloud(*temp2, *map_cloud,
                                    transform_odom_to_baselink);
        // std::cout << "map cloud: " << map_cloud->points.size() << std::endl;

        // Filter pucks to a own cloud
        color_filter(map_cloud, temp, 0, 0, 255);  // Blue
        *safe_points = *temp;
        color_filter(map_cloud, temp, 255, 255, 0);  // Yelllow
        *safe_points += *temp;

        safe_points->is_dense = false;
        safe_points->width = 1;
        safe_points->height = safe_points->points.size();

        // Map data for playing game, e.g. transform cloud to map frame
        pcl_ros::transformPointCloud(*temp2, *map_cloud_in_map_frame,
                                    transform_odom_to_map);

        // -------------------------------------------------
        // Apply masks for the collision avoidance data, e.g. add safe zones
        // -------------------------------------------------

        // Do not avoid any bucks -> remove all pucks from the collision
        // avoidance cloud
        double safe_zone_radius = 0.2;  // m

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<PointType> extract;

        // Add safe zone around pucks (robot does not avoid them)
        for (int i = 0; i < safe_points->points.size(); i++) {
            for (int j = 0; j < collision_avoidance_cloud->points.size(); j++) {
                if (distance_between_points(
                        safe_points->points[i].x, safe_points->points[i].y,
                        collision_avoidance_cloud->points[j].x,
                        collision_avoidance_cloud->points[j].y) <=
                    safe_zone_radius) {
                    // Remove these points
                    inliers->indices.push_back(j);
                }
            }
        }
        // std::cout << "safe zones: " << safe_points->points.size() << std::endl;

        extract.setInputCloud(collision_avoidance_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*collision_avoidance_cloud);

        collision_avoidance_cloud->is_dense = false;
        collision_avoidance_cloud->width = 1;
        collision_avoidance_cloud->height =
            collision_avoidance_cloud->points.size();

        // For debugging
        pub_pointcloud(*collision_avoidance_cloud, debug_cloud_pub);
    }

   // ---------------------------------------------------------
   // Check if location is really known
   // ---------------------------------------------------------

    bool transformation_set = false;
    n->param("transformation_map_to_odom_set", transformation_set, false);

    if (tf_delay_counter <= 10 && transformation_set) {
        // This delay prevents the feedback from being done with old tf
        // data.
        tf_delay_counter++;
    }

    if (succesful_robot_pos_tf == false || got_field_width == false ||
        transformation_set == false || tf_delay_counter <= 10) {
        ROS_INFO_STREAM("Map to Odom transformation missing");
        // Make robot initialize untill location and field width found.
        return false;  // disable for debugging
    }

   // ---------------------------------------------------------
   // Game has started -> set team:
   // ---------------------------------------------------------

    if (is_blue_team == -1) {
        // Initialize team setting once in the begining
        if (robot_map_y < field_length / 2.0) {
            is_blue_team = 1;  // true
        } else {
            is_blue_team = 0;  // false
        }
        ROS_INFO_STREAM("Setting team: " << is_blue_team);

        player::SendColor send_color_srv;
        send_color_srv.request.team = team_name;

        if (is_blue_team == 1) {
            send_color_srv.request.color = "blue";
        } else {
            send_color_srv.request.color = "yellow";
        }

        if (send_color_client.call(send_color_srv)) {
            if (send_color_srv.response.ok) {
                ROS_INFO("The color is correct");
            } else {
                ROS_INFO("The color was NOT correct");

                if (is_blue_team == 0) {
                    is_blue_team = 1;
                } else {
                    is_blue_team = 0;
                }

                ROS_INFO_STREAM("New team color: " << is_blue_team);
            }
        } else {
            ROS_ERROR("Failed to call service SendColor");
        }
    }

   
    // -------------------------------------------------
    // Find closest obstacle in collision_avoidance_cloud
    // -------------------------------------------------

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

    //ROS_INFO_STREAM("Closest laser obstacle r: " << closest_laser_distance
    //                                             << "\tangle: "
    //                                             << closest_laser_direction);

    // -------------------------------------------------
    // Calculate closest wall to robot of the field
    // -------------------------------------------------

    // Get closest wall of the field intersecting the safe zone of the
    // robot.
    double closest_wall_x;  // in map frame
    double closest_wall_y;  // in map frame
    get_closest_wall(robot_map_x, robot_map_y, robot_map_yaw, closest_wall_x,
                     closest_wall_y);

    // Get distance and angle of the wall in base_link frame.
    double closest_wall_distance;
    double closest_wall_direction;
    to_polar(closest_wall_x, closest_wall_y, closest_wall_distance,
             closest_wall_direction, robot_map_x, robot_map_y, robot_map_yaw);

    //ROS_INFO_STREAM("Closest wall  obstacle r: " << closest_wall_distance
    //                                             << "\tangle: "
    //                                            << closest_wall_direction);

    // -------------------------------------------------
    // Calculate closest obstacle of all
    // -------------------------------------------------
    const double DETERMINE_ROTATION_DIRECTION_BASED_ON_WALL_DISTANCE =
        DISTANCE_LINEAR_FREE;  // m

    closest_obstacle_distance_g = closest_laser_distance;
    closest_obstacle_direction_g = closest_laser_direction;
    if (closest_wall_distance < closest_laser_distance) {
        // Wall closer
        closest_obstacle_distance_g = closest_wall_distance;
        closest_obstacle_direction_g = closest_wall_direction;

    } else {
        // Obstacle closer
        closest_obstacle_distance_g = closest_laser_distance;

        if (closest_wall_distance <
            DETERMINE_ROTATION_DIRECTION_BASED_ON_WALL_DISTANCE) {
            // Even if laser obstacle is closer the direction ofrotation is
            // determined by the wall if it is closer than the threshold
            // distance.
            closest_obstacle_direction_g = closest_wall_direction;
        } else {
            closest_obstacle_direction_g = closest_laser_direction;
        }
    }

    // Reset
    got_map = false;
    got_laser = false;

    return true;
}

void update_robot_state() {
    // -------------------------------------------------
    // Calculate desired speed and rotation
    // -------------------------------------------------

    double speed_linear = 0;
    double speed_rotational = 0;

    // Robot distance and heading error to goal point
    double robot_distance_error = 0;
    double robot_yaw_error = 0;

    // State and parameter update of robot
    if (state == initialize) {
    } else if (state == drive_random) {
    } else if (state == drive_to) {
        if(game_state != initialize_location){
            // Check if goal is valid or it has been reached already
            if (goal_point_x > 0 && goal_point_y > 0 &&
                goal_point_x < field_width && goal_point_y < field_length) {
                // Valid goal, calculate errors
                to_polar(goal_point_x, goal_point_y, robot_distance_error,
                        robot_yaw_error, robot_map_x, robot_map_y, robot_map_yaw);

                if (robot_distance_error <= DISTANCE_GOAL_REACHED) {
                    // Goal reached
                    state = stop;
                    // goal_point_x = -1;
                    // goal_point_y = -1;
                } else {
                    // Goal not reached
                    state = drive_to;
                }
            } else {
                // No valid goal point change state to stop
                state = stop;
            }
        } else {
            // Initialization -> different rules (drive outside of field)
            to_polar(goal_point_x, goal_point_y, robot_distance_error,
                    robot_yaw_error, robot_map_x, robot_map_y, robot_map_yaw);

            if (robot_distance_error <= DISTANCE_GOAL_REACHED) {
                // Goal reached
                state = stop;
            } else {
                // Goal not reached
                state = drive_to;
            }
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
        distance_to_go -=
            sign * distance_between_points(robot_map_x, robot_map_y,
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
    if (state == initialize) {
        // rotate to find location
        speed_linear = 0;
        speed_rotational = MAX_ROTATIONAL_SPEEED;
        ROS_INFO_STREAM("Robot state: INITIALIZE");
    } else if (state == drive_random) {
        collision_avoidance(speed_linear, speed_rotational,
                            closest_obstacle_distance_g,
                            closest_obstacle_direction_g, robot_distance_error,
                            robot_yaw_error);

        ROS_INFO_STREAM("Robot state: DRIVE RANDOM");

    } else if (state == drive_to) {
        set_speeds_drive_to(speed_linear, speed_rotational,
                            closest_obstacle_distance_g,
                            closest_obstacle_direction_g, robot_distance_error,
                            robot_yaw_error);
        ROS_INFO_STREAM("Robot state: DRIVE TO");

    } else if (state == rotate) {
        speed_linear = 0;
        speed_rotational =
            fmax(MIN_ROTATIONAL_SPEED,
            rotation_to_go * 4 * MAX_ROTATIONAL_SPEEED / DISTANCE_FREE_ROTATION);
        ROS_INFO_STREAM("Robot state: ROTATE, to rotate: " << rotation_to_go);

    } else if (state == move) {
        int sign = (distance_to_go >= 0) - (distance_to_go < 0);
        speed_linear = sign * fmax(MIN_LINEAR_SPEED, fabs(distance_to_go) *
                                                         MAX_LINEAR_SPEED /
                                                         DISTANCE_LINEAR_FREE);
        speed_rotational = 0;
        ROS_INFO_STREAM(
            "Robot state: MOVE, distance to move: " << distance_to_go);

    } else if (state == stop) {
        speed_linear = 0;
        speed_rotational = 0;

        ROS_INFO_STREAM("Robot state: STOP");
    }

    ROS_INFO_STREAM("Error to goal r: " << robot_distance_error
                                        << " angle: " << robot_yaw_error);

    //ROS_INFO_STREAM("Closest control obstacle r: "
     //               << closest_obstacle_distance_g
     //               << " angle: " << closest_obstacle_direction_g);

    ROS_INFO_STREAM("Rotational out: " << speed_rotational);
    ROS_INFO_STREAM("Linear out: " << speed_linear
                                   << "\n************************************");

    // -------------------------------------------------
    // Publish speed commands
    // -------------------------------------------------

    set_velocities(
        fmax(-MAX_LINEAR_SPEED, fmin(speed_linear, MAX_LINEAR_SPEED)),
        fmax(-MAX_ROTATIONAL_SPEEED,
             fmin(speed_rotational, MAX_ROTATIONAL_SPEEED)));

    robot_map_last_x = robot_map_x;
    robot_map_last_y = robot_map_y;
    robot_map_last_yaw = robot_map_yaw;

    // Update safety_zone_x_offset base on robot angular speed
    safety_zone_x_offset =SAFETY_ZONE_X_OFFSET_MAX;
      // fmin(fmax(0, -/MAX_ROTATIONAL_SPEEED *
              // speed_rotational + SAFETY_ZONE_X_OFFSET_MAX),
              // SAFETY_ZONE_X_OFFSET_MAX);
    // ROS_INFO_STREAM("safety_zone_x_offset: " << safety_zone_x_offset);
}

bool got_messages() {
    return got_laser && got_laser && got_collision_avoidance_cloud;
}

bool point_inside_goal(double x, double y, bool check_blue_goal) {
    if (check_blue_goal) {
        if (x > field_width / 2.0 - 0.5 && x < field_width / 2.0 + 0.5 &&
            y > field_length * 0.1 && y < field_length * 0.1 + 0.5) {
            return true;
        }
    } else {
        // Check yellow goal, on the other side of the field.
        if (x > field_width / 2.0 - 0.5 && x < field_width / 2.0 + 0.5 &&
            y > field_length * 0.9 - 0.5 && y < field_length * 0.9) {
            return true;
        }
    }
    return false;
}

bool point_inside_goal_and_close_area(double x, double y, bool check_blue_goal) {
    double margin = 0.2; // m
    if (check_blue_goal) {
        if (x > field_width / 2.0 - 0.5 - margin && x < field_width / 2.0 + 0.5 + margin&&
            y > field_length * 0.1 - margin&& y < field_length * 0.1 + 0.5 + margin) {
            return true;
        }
    } else {
        // Check yellow goal, on the other side of the field.
        if (x > field_width / 2.0 - 0.5 - margin && x < field_width / 2.0 + 0.5 + margin &&
            y > field_length * 0.9 - 0.5 - margin && y < field_length * 0.9 + margin) {
            return true;
        }
    }
    return false;
}

bool is_robot_in_home_goal(double x, double y) {
    return point_inside_goal(x, y, is_blue_team == 1);
}

bool is_robot_in_enemy_goal(double x, double y) {
    return point_inside_goal_and_close_area(x, y, is_blue_team == 0);
}

bool robot_has_puck() {
    // This function returns true if the robot has te puck based on
    // laser data.

    // Parameters (in base_link frame)

    const double RECTANCE_SIZE = 0.15;              // m
    const double RECTANGLE_MIDDLE_X_OFFSET = 0.24;  // m

    // Count points inside rectancle
    int points_in_puck_zone = 0;

    for (int i = 0; i < laser_cloud->points.size(); i++) {
        double x = laser_cloud->points[i].x;  // x-axis points forward
        double y = laser_cloud->points[i].y;

        if (y > -0.5 * RECTANCE_SIZE && y < 0.5 * RECTANCE_SIZE &&
            x > RECTANGLE_MIDDLE_X_OFFSET - 0.5 * RECTANCE_SIZE &&
            x < RECTANGLE_MIDDLE_X_OFFSET + 0.5 * RECTANCE_SIZE) {
            // point inside puck zone
            points_in_puck_zone++;
        }
    }
    // std::cout << laser_cloud->points.size() << std::endl;
    return points_in_puck_zone > 2;
}

// game logic ************************************************
int get_closest_puck_to_point(double& x, double& y, double px, double py,
                              PointCloudPtr& map) {
    // returns coordinates of closest puck with correct color, that is not
    // already in the goal. Return int meaning 0: not found, 1: found, -1 all
    // pucks in goal.

    double closest_x = -1;
    double closest_y = -1;
    double last_distance = 1000;

    int number_of_bucks_found = 0;
    int number_of_bucks_outside_of_goals = 0;

    for (int j = 0; j < map->points.size(); j++) {
        double x = map->points[j].x;
        double y = map->points[j].y;

        // Get color of point
        uint32_t rgb = *reinterpret_cast<int*>(&map->points[j].rgb);
        uint8_t b = (rgb >> 0) & 0xff;
        uint8_t g = (rgb >> 8) & 0xff;
        uint8_t r = (rgb >> 16) & 0xff;

        if (is_blue_team == 1) {  // blue team
            if (r == 0 && g == 0 && b == 255) {
                double distance = distance_between_points(px, py, x, y);

                if (!point_inside_goal(x, y, false)) {
                    if (distance < last_distance) {
                        // So far the closest and not in the goal already
                        last_distance = distance;
                        closest_x = x;
                        closest_y = y;
                    }
                    number_of_bucks_outside_of_goals++;
                }
                number_of_bucks_found++;
            }
        } else if (is_blue_team == 0) {  // yellow team
            if (r == 255 && g == 255 && b == 0) {
                double distance = distance_between_points(px, py, x, y);

                if (!point_inside_goal(x, y, true)) {
                    if (distance < last_distance) {
                        // So far the closest and not in the goal already
                        last_distance = distance;
                        closest_x = x;
                        closest_y = y;
                    }
                    number_of_bucks_outside_of_goals++;
                }
                number_of_bucks_found++;
            }
        } else {
            // team color not known -> return
            return 0;
        }
    }

    x = closest_x;
    y = closest_y;

    if (number_of_bucks_found == NUMBER_OF_BUCKS_PER_TEAM &&
        number_of_bucks_outside_of_goals == 0) {
        return -1;
    } else if (closest_x != -1) {
        return 1;
    } else {
        return 0;
    }
}

int get_any_closest_puck_to_point(double& x, double& y, double px, double py,
                              PointCloudPtr& map) {
    // returns coordinates of closest puck.
    // Return int meaning -1: not found, 1 found blue, 0: found yewllow

    double closest_x = -100;
    double closest_y = -100;
    double last_distance = 1000;

    int closest_clolor = -1;

    for (int j = 0; j < map->points.size(); j++) {
        double x = map->points[j].x;
        double y = map->points[j].y;

        // Get color of point
        uint32_t rgb = *reinterpret_cast<int*>(&map->points[j].rgb);
        uint8_t b = (rgb >> 0) & 0xff;
        uint8_t g = (rgb >> 8) & 0xff;
        uint8_t r = (rgb >> 16) & 0xff;


        if (r == 0 && g == 0 && b == 255) {
            double distance = distance_between_points(px, py, x, y);
         
            if (distance < last_distance) {
                // So far the closest
                last_distance = distance;
                closest_x = x;
                closest_y = y;
                closest_clolor = 1; // blue
            }
        } else if (r == 255 && g == 255 && b == 0) {
            double distance = distance_between_points(px, py, x, y);

            if (distance < last_distance) {
                // So far the closest
                last_distance = distance;
                closest_x = x;
                closest_y = y;
                closest_clolor = 0; // yellow
            }
        }
    }

    x = closest_x;
    y = closest_y;
   return closest_clolor;
}

void point_map_to_odom(double x_in, double y_in, double& x_out, double& y_out) {
    // transform single point from map to odometry frame

    if (succesful_map_tf == false) {
        // No transformation set currently
        x_out = -1;
        y_out = -1;
        return;
    }
    PointCloudPtr robot(new PointCloud);
    PointCloudPtr robot_temp(new PointCloud);

    PointType robo_point = PointType();
    robo_point.x = x_in;
    robo_point.y = y_in;
    robo_point.z = 0;
    robot->points.push_back(robo_point);

    pcl_ros::transformPointCloud(*robot, *robot_temp, transform_map_to_odom);

    x_out = robot_temp->points[0].x;
    y_out = robot_temp->points[0].y;
}

void set_team_color(int is_blue){
    is_blue_team = is_blue;
    ROS_INFO_STREAM("Setting team: " << is_blue_team);

    player::SendColor send_color_srv;
    send_color_srv.request.team = team_name;
    if (is_blue_team == 1) {
        send_color_srv.request.color = "blue";
    } else {
        send_color_srv.request.color = "yellow";
    }

    if (send_color_client.call(send_color_srv)) {
        if (send_color_srv.response.ok) {
            ROS_INFO("The color is correct");
        } else {
            ROS_INFO("The color was NOT correct");
            if (is_blue_team == 0) {
                is_blue_team = 1;
            } else {
                is_blue_team = 0;
            }
            ROS_INFO_STREAM("New team color: " << is_blue_team);
        }
    } else {
        ROS_ERROR("Failed to call service SendColor");
    }
}

void update_game_logic(bool data_processing_succesful) {
    // This function updates the game state and controls the robot
    ROS_INFO_STREAM("ROBOT has puck: " << robot_has_puck() <<
                                 " team color blue: " << is_blue_team);

#ifdef use_referee
    // Change state when referee tells to
    if (game_started_msg && game_state == wait_for_start) {
        game_state = initialize_location;
    } else if (game_started_msg == false) {
        game_state = wait_for_start;
        state = stop;
        return;
    }
#endif

    /*/ Check if robot is outside or inside of the field, if outside ->
    // reinitialize
    if (robot_map_x < 0 || robot_map_x > field_width || robot_map_y < 0 ||
        robot_map_y > field_length) {
        game_state = initialize_location;
        state = initialize;
        ROS_INFO_STREAM("Robot outside of the field -> state: initialize.");
        return;  // TODO nice way of handling this case
    }*/

    if (game_state == wait_for_start) {
        state = stop;
        ROS_INFO_STREAM("Game state: wait for start");

    } else if (game_state == initialize_location) {
        ROS_INFO_STREAM("Game state: initialize_location");

        if (data_processing_succesful == true) {
            // robot knows where it is -> start playing game
            state = stop;
            game_state = look_for_puck;
            moves_done = 0; // reset counter
        } else {
            // Robot does not know where it is
            // -> rotate 360
            // -> drive to closest puck
            // -> start rotating to find location

            if(state == stop && moves_done == 0){
                // First move rotate 360
                state = rotate;
                rotation_to_go = 2*3.1415;
                moves_done = 1;
            } else if (state == stop && moves_done == 1){
                // drive to closest buck
                
                double x = -100;
                double y = -100;
                int color = get_any_closest_puck_to_point(x,y,robot_map_x,robot_map_y, map_cloud_in_map_frame);
                if(color != -1){
                    // found puck:
                    state = drive_to;
                    goal_point_x = x;
                    goal_point_y = y;
                    moves_done = 2; 

                    // Set team color:
                    set_team_color(color);
                } else {
                    // not found -> try again
                    state = rotate;
                    rotation_to_go = 2*3.1415;
                    moves_done = 1;
                }
                
            } else if (state == stop && moves_done == 2){
                // robot moved to closest puck -> rotate untill position found
                state = initialize;
                moves_done = 0;
            }  
        }

    } else if (game_state == look_for_puck) {
        ROS_INFO_STREAM("Game state: look_for_puck");
        // Choose closest puck and drive to it, if no buck is found
        // robot drives around.

        if (state == stop || state == drive_to) {
            // Set goal to drive to
            double x = -1;
            double y = -1;
            int success = get_closest_puck_to_point(
                x, y, robot_map_x, robot_map_y, map_cloud_in_map_frame);
            ROS_INFO_STREAM("Get closest puck succes: "
                            << success);
            //ROS_INFO_STREAM("Goal point x: " << x << " y: " << y);
            if (success == 1) {
                // Found puck to drive to
                game_state = drive_to_puck;
                state = drive_to;
                goal_point_x = x;
                goal_point_y = y;

            } else if (success == 0 && state == stop) {
                // No puck found and not driving-> drive middle or home
                if (!is_robot_in_home_goal(robot_map_x, robot_map_y)) {
                    // Drive to home

                    state = drive_to;
                    if (is_blue_team == 1) {  // to home
                        goal_point_x = field_width / 2.0;
                        goal_point_y = 0.1 * field_length + 0.25;
                    } else {
                        goal_point_x = field_width / 2.0;
                        goal_point_y = 0.9 * field_length - 0.25;
                    }

                } else {
                    // Already in home but no buck found, drive to the middel of
                    // the field.
                    state = drive_to;
                    goal_point_x = field_width / 2.0;
                    if(is_blue_team == 1){
                        goal_point_y = field_length / 2.0 + 1;
                    } else {
                        goal_point_y = field_length - (field_length / 2.0 + 1);
                    }
                    
                }

            } else if (success == -1) {
                // All pucks are in the goal of the enemy
                ROS_INFO_STREAM("All pucks in the goal stopping game!");
                game_state = wait_for_start;
                state = stop;
            }
        } else {
            // should not happen
            state = stop;
        }

        ROS_INFO_STREAM("Goal point x: " << goal_point_x
                                         << " y: " << goal_point_y);

    } else if (game_state == drive_to_puck) {
        ROS_INFO_STREAM("Game state: drive_to_puck");

        if (state == stop) {
            // Robot has reached its destination, the puck pick up point.

            // Remove picked goal from map, to avoid wrong/old data in map
            double x = 0, y = 0;
            point_map_to_odom(goal_point_x, goal_point_y, x, y);
            std::cout << "Delete pucks at map x: " << goal_point_x
                      << " y: " << goal_point_y << std::endl;
            //std::cout << "Delete pucks at odom x: " << x << " y: " << y
            //          << std::endl;
            if (x != -1) {
                n->setParam("delete_puck", true);
                n->setParam("puck_x", x);
                n->setParam("puck_y", y);
            }

            if (robot_has_puck()) {
                // Buck hit succesfully and not in goal -> drive to
                // enemy goal -> change game state
                game_state = drive_with_puck_to_goal;
                state = stop;
            } else {
                // Robot did not hit puck -> go to previous state and try again.
                std::cout << "Buck missed -> look_for_puck" << std::endl;
                // remove all pucks from the field
                //n->setParam("reset_all_pucks",
                //            true);  // is this really required TODO??
                game_state = look_for_puck;
                state = stop;
            }

        } else if (state == drive_to) {
            // while diving to the puck, update goal_point
            double x = -1;
            double y = -1;
            int success = get_closest_puck_to_point(
                x, y, goal_point_x, goal_point_y, map_cloud_in_map_frame);
            if (success == 1) {
                // Set updated coordinates
                goal_point_x = x;
                goal_point_y = y;
            } else {
                // Go to previous state and try again, puck lost from map
                state = stop;
                game_state = look_for_puck;
            }
        } else {
            // Shoud not be possible -> reset
            game_state = look_for_puck;
            state = stop;
        }
        ROS_INFO_STREAM("Goal point x: " << goal_point_x
                                         << " y: " << goal_point_y);

    } else if (game_state == drive_with_puck_to_goal) {
        ROS_INFO_STREAM("Game state: drive_with_puck_to_goal, team color: "
                        << is_blue_team);

        if (state == stop) {
            if (!is_robot_in_enemy_goal(robot_map_x, robot_map_y)) {
                state = drive_to;
                goal_point_x = field_width / 2.0;
                // Drive to goal
                if (is_blue_team == 0) {
                    if(robot_map_y > 0.1 * field_length && robot_map_y < 0.1 * field_length + 0.5){
                        // middle
                        goal_point_y = 0.1 * field_length + 0.25;
                    }else if(robot_map_y > 0.1 * field_length + 0.5){
                        goal_point_y = 0.1 * field_length + 0.5;
                    } else {
                         goal_point_y = 0.1 * field_length;
                    }
                    
                } else {

                    if(robot_map_y < 0.9 * field_length && robot_map_y > 0.9 * field_length - 0.5){
                        // middle
                        goal_point_y = 0.9 * field_length - 0.25;
                    }else if(robot_map_y < 0.9 * field_length - 0.5){
                        goal_point_y = 0.9 * field_length - 0.5;
                    } else {
                         goal_point_y = 0.9 * field_length;
                    }
                }
            } else {
                // Already in goal -> change to next state
                std::cout << "Changing to state to: leave_buck_in_goal"
                          << std::endl;
                game_state = leave_buck_in_goal;
            }
        } else if (state == drive_to) {
        } else {
            // Shoud not be possible -> reset
            game_state = look_for_puck;
            state = stop;
        }

        ROS_INFO_STREAM("Goal point x: " << goal_point_x
                                         << " y: " << goal_point_y);

    } else if (game_state == leave_buck_in_goal) {
        // Drive back wards and rotate to leave the puck in the goal
        if (state == stop) {
            // Move/s finished or first move
            if (moves_done == 0) {
                // No moves done, start first move, drive backwards
                distance_to_go = -0.3;  // m
                state = move;
            } else if (moves_done == 1) {
                // start second move, rotate 180 deg
                rotation_to_go = 3.1415;  // rad
                state = rotate;
            } else {
                // Puck leaved in goal -> go to next puck
                state = stop;
                game_state = look_for_puck;
                moves_done = 0;
            }

        } else if (state == move) {
            moves_done = 1;
        } else if (state == rotate) {
            moves_done = 2;
        } else {
            // Shoud not be possible -> reset
            game_state = look_for_puck;
            state = stop;
            moves_done = 0;
        }

        ROS_INFO_STREAM("Game state: leave_buck_in_goal");
    } else {  // end game
        game_state = wait_for_start;
        state = stop;
    }
}

// **************************************************************
void init_node(int argc, char** argv) {
    ros::init(argc, argv, "play_node");
    n = std::make_unique<ros::NodeHandle>();

    n->getParam( "play_node/team", team_number );

    ROS_INFO_STREAM("Team number: " << team_number << "\n" );

    tfBuffer = new tf2_ros::Buffer(ros::Duration(100));
    tf_listener = new tf2_ros::TransformListener(*tfBuffer);

    velocity_pub = n->advertise<geometry_msgs::Twist>(addRobotName("/cmd_vel"), 1000);
    debug_cloud_pub = n->advertise<PointCloud>("play_node/debug", 1);

#ifdef use_referee
    ROS_INFO("Waiting for referee /waitForTeams");
    ros::topic::waitForMessage<std_msgs::Empty>("waitForTeams");
    ROS_INFO("done");
    team_ready_client = n->serviceClient<player::TeamReady>(
        "TeamReady");  // referee instead of player??

    player::TeamReady team_ready_srv;
    team_ready_srv.request.team = team_name;  // team name

    if (team_ready_client.call(team_ready_srv)) {
        if (team_ready_srv.response.ok) {
            ROS_INFO("Team name granted");
        } else {
            team_name = "Not Green peas";
            team_ready_srv.request.team = team_name;
            team_ready_client.call(team_ready_srv);

            if (team_ready_srv.response.ok) {
                ROS_INFO("New team name granted");
            } else {
                ROS_ERROR("DAFUCK");
            }
        }
    } else {
        ROS_ERROR("Failed to call service TeamReady");
    }
#endif

    game_control_sub = n->subscribe("gameControl", 1, &game_control_callback);

    ROS_INFO("Waiting for laser scan message");
    ros::topic::waitForMessage<sensor_msgs::LaserScan>(
        addRobotName( "/front_laser/scan" ) );

    // ROS_INFO("Waiting for map_node/map");
    // ros::topic::waitForMessage<PointCloud>("map_node/map");

    map_sub = n->subscribe("map_node/map", 1, &map_callback);
    collision_avoidance_cloud_sub =
        n->subscribe("pointcloud_node/collision_avoidance", 1,
                     &collision_avoidance_cloud_callback);

    laser_sub = n->subscribe( addRobotName( "/front_laser/scan" ), 1, &laser_callback);

    field_width_sub =
        n->subscribe("field_width_node/width", 1, &field_width_callback);


    send_color_client = n->serviceClient<player::SendColor>("SendColor");

    send_dimensions_client =
        n->serviceClient<player::SendDimensions>("SendDimensions");
}

int main(int argc, char** argv) {
    init_node(argc, argv);

    // 20 Hz loop
    ros::Rate r(20);
    //playNode.tf_map_to_odom_boardcaster(0, 0, 0); // boardcast
    while (ros::ok()) {
        // ROS_INFO("%d", playNode.got_messages());
        //        

        if (got_messages()) {
            bool success = process_messages();
            update_game_logic(success);
            update_robot_state();
        }

        ros::spinOnce();
        r.sleep();
    }
    set_velocities(0, 0);
    return 0;
}