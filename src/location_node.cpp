#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>

#include "pointcloud_helpers.hpp"

class PlayNode {
   public:
    PlayNode(int argc, char **argv) {
        ros::init(argc, argv, "location_node");
        n = std::make_unique<ros::NodeHandle>();

        ROS_INFO("Waiting for map_node/map");
        ros::topic::waitForMessage<PointCloud>("map_node/map");

        ROS_INFO("Waiting for field_width_node/width");
        ros::topic::waitForMessage<geometry_msgs::Vector3>(
            "field_width_node/width");

        field_width_sub = n->subscribe("field_width_node/width", 1,
                                       &PlayNode::field_width_callback, this);

        map_cloud_sub = n->subscribe("map_node/map", 1,
                                     &PlayNode::map_cloud_callback, this);
    }

    void map_cloud_callback(const PointCloud::ConstPtr &msg) {
        // ROS_INFO("Got new detected objects");

        map_cloud_msg = *msg;
        got_map_cloud = true;
    }

    void field_width_callback(const geometry_msgs::Vector3::ConstPtr &msg) {
        geometry_msgs::Vector3 width = *msg;
        field_width = width.x;
        got_field_width = true;
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

    void color_filter(PointCloudPtr &cloud, int r, int g, int b) {
        // Filters pointcloud by a specific color
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<PointType> extract;
        for (int i = 0; i < (*cloud).size(); i++) {
            uint32_t argb = cloud->points[i].rgba;
            uint8_t alpha = (argb >> 24) & 0xff;
            uint8_t bp = (argb >> 0) & 0xff;
            uint8_t gp = (argb >> 8) & 0xff;
            uint8_t rp = (argb >> 16) & 0xff;

            if (r == rp && g == gp && b == bp) {
                // Remove these points
                inliers->indices.push_back(i);
            }
        }
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud);
    }

    PointType get_centroid_of_color(PointCloudPtr &cloud, int r, int g, int b) {
        // Filters pointcloud by a specific color
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<PointType> extract;
        for (int i = 0; i < (*cloud).size(); i++) {
            uint32_t argb = cloud->points[i].rgba;
            uint8_t alpha = (argb >> 24) & 0xff;
            uint8_t bp = (argb >> 0) & 0xff;
            uint8_t gp = (argb >> 8) & 0xff;
            uint8_t rp = (argb >> 16) & 0xff;

            if (r == rp && g == gp && b == bp) {
                // Keep these points
                inliers->indices.push_back(i);
            }
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*color_cloud);

        PointType centroid;
        pcl::computeCentroid(*color_cloud, centroid);
        return centroid;
    }

    Eigen::Affine3f translate_cloud(PointCloudPtr &cloud_in,
                                    PointCloudPtr &cloud_out,
                                    double translation_x, double translation_y,
                                    double rotation_yaw) {
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

        // Define a translation of 2.5 meters on the x axis.
        transform_2.translation() << translation_x, translation_y, 0.0;

        // The same rotation matrix as before; theta radians around Z axis
        transform_2.rotate(
            Eigen::AngleAxisf(rotation_yaw, Eigen::Vector3f::UnitZ()));

        // Print the transformation
        // printf("\nMethod #2: using an Affine3f\n");
        // std::cout << transform_2.matrix() << std::endl;

        // Executing the transformation
        // You can either apply transform_1 or transform_2; they are the same
        pcl::transformPointCloud(*cloud_in, *cloud_out, transform_2);

        return transform_2;
    }

    std::vector<Eigen::Affine3f> translate_cloud_based_on_centroids(
        PointCloudPtr &cloud_target, PointCloudPtr &cloud_map, int r1, int g1,
        int b1, int r2, int g2, int b2) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        // Manual translation based on goals
        PointType goal_orange_target =
            get_centroid_of_color(cloud_target, r1, g1, b1);
        PointType goal_cyan_target =
            get_centroid_of_color(cloud_target, r2, g2, b2);

        PointType goal_orange_map =
            get_centroid_of_color(cloud_map, r1, g1, b1);
        PointType goal_cyan_map = get_centroid_of_color(cloud_map, r2, g2, b2);

        double vector_target_x = goal_cyan_target.x - goal_orange_target.x;
        double vector_target_y = goal_cyan_target.y - goal_orange_target.y;

        double vector_map_x = goal_cyan_map.x - goal_orange_map.x;
        double vector_map_y = goal_cyan_map.y - goal_orange_map.y;

        double vector_target_len = sqrt(vector_target_x * vector_target_x +
                                        vector_target_y * vector_target_y);
        double vector_map_len =
            sqrt(vector_map_x * vector_map_x + vector_map_y * vector_map_y);

        double rotation_map = atan2(vector_map_y, vector_map_x);
        double rotation_target = atan2(vector_target_y, vector_target_x);
        double rotation = rotation_target - rotation_map;

        double translation_x = goal_orange_target.x - goal_orange_map.x;
        double translation_y = goal_orange_target.y - goal_orange_map.y;

        std::vector<Eigen::Affine3f> transforms;

        // Move desired center of rotation to origo
        transforms.push_back(translate_cloud(
            cloud_map, cloud_temp, -goal_orange_map.x, -goal_orange_map.y, 0));
        // Rotate and translate to final location
        transforms.push_back(translate_cloud(
            cloud_temp, cloud_map, translation_x + goal_orange_map.x,
            translation_y + goal_orange_map.y, rotation));

        return transforms;
    }

    int get_number_of_coloured_points(PointCloudPtr &cloud, int r, int g,
                                      int b) {
        int number = 0;
        for (int i = 0; i < (*cloud).size(); i++) {
            uint32_t argb = cloud->points[i].rgba;
            uint8_t alpha = (argb >> 24) & 0xff;
            uint8_t bp = (argb >> 0) & 0xff;
            uint8_t gp = (argb >> 8) & 0xff;
            uint8_t rp = (argb >> 16) & 0xff;

            if (r == rp && g == gp && b == bp) {
                number++;
            }
        }
        return number;
    }

    void process_messages() {
        PointCloudPtr cloud_map(new PointCloud(map_cloud_msg));
        PointCloudPtr cloud_target = get_ideal_field_cloud(2.7, true);

        // Filter puck colors from input map
        color_filter(cloud_map, 255, 255, 0);  // Yellow
        color_filter(cloud_map, 0, 0, 255);    // Blue
        // color_filter(cloud_map, 255, 140, 0); // Orange goal for testing
        // color_filter(cloud_map, 0, 255, 255); // Cyan goal for testing

        // std::cout << "Cloud map cloud input size: " <<
        // cloud_map->points.size()
        //          << std::endl;
        // std::cout << "Cloud target map size:" << cloud_target->points.size()
        //          << std::endl;

        // Manual translation
        int orange_points =
            get_number_of_coloured_points(cloud_map, 255, 140, 0);
        int cyan_points = get_number_of_coloured_points(cloud_map, 0, 255, 255);

        // std::cout << "cyan" << cyan_points << " orange" << orange_points
        //          << std::endl;
        std::vector<Eigen::Affine3f> transforms;

        if (cyan_points >= 2 && orange_points >= 2) {
            // Use goals for translation
            transforms = translate_cloud_based_on_centroids(
                cloud_target, cloud_map, 255, 140, 0, 0, 255, 255);
        } else if (cyan_points > orange_points) {
            // Use cyan goals and green poles for translation
            transforms = translate_cloud_based_on_centroids(
                cloud_target, cloud_map, 0, 255, 255, 0, 255, 0);
        } else {
            // Use orange goals and green poles for translation
            transforms = translate_cloud_based_on_centroids(
                cloud_target, cloud_map, 255, 140, 0, 0, 255, 0);
        }

        // Final translation using iterative closestpoint
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(cloud_map);
        icp.setInputTarget(cloud_target);

        PointCloudPtr Final(new PointCloud);
        icp.align(*Final);

        ROS_INFO_STREAM("Has converged:" << icp.hasConverged() << " score: "
                                         << icp.getFitnessScore());
        // std::cout << icp.getFinalTransformation() << std::endl;

        // Check if succesful
        if (icp.hasConverged() == false || icp.getFitnessScore() > 0.06) {
            // Not succesful -> stop
            ROS_INFO_STREAM("Not sucessfull transformation!");
            return;
        }

        Eigen::Affine3f transform_icp = Eigen::Affine3f::Identity();
        transform_icp.matrix() = icp.getFinalTransformation();
        transforms.push_back(transform_icp);

        // Calculate total transformation
        Eigen::Affine3f transform_odom_to_map = Eigen::Affine3f::Identity();
        transform_odom_to_map = transform_odom_to_map * transforms[2];
        transform_odom_to_map = transform_odom_to_map * transforms[1];
        transform_odom_to_map = transform_odom_to_map * transforms[0];

        // Set transformation
        transform_odom_to_map_latest = transform_odom_to_map;
        transform_set = true;
        got_map_cloud = false;  // reset
    }

    bool got_messages() const { return got_map_cloud && got_field_width; }
    bool is_transform_set() const { return transform_set; }
    Eigen::Affine3f get_latest_transformation() {
        return transform_odom_to_map_latest;
    };

   private:
    bool got_map_cloud = false;
    bool got_field_width = false;

    bool transform_set = false;
    Eigen::Affine3f transform_odom_to_map_latest;

    std::unique_ptr<ros::NodeHandle> n;
    ros::Subscriber map_cloud_sub;
    ros::Subscriber field_width_sub;

    PointCloud map_cloud_msg;
    double field_width = 3;
};

int main(int argc, char **argv) {
    PlayNode playNode(argc, argv);

    // 20 Hz loop
    ros::Rate r(20);
    while (ros::ok()) {
        if (playNode.got_messages()) {
            playNode.process_messages();
        }

        if (playNode.is_transform_set()) {
            // Publish latest succesful transfomation
            playNode.tf_map_to_odom_boardcaster(
                playNode.get_latest_transformation()(0, 3),
                playNode.get_latest_transformation()(1, 3),
                acos(playNode.get_latest_transformation().rotation()(0, 0)));
        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
