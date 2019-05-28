#include <cmath>
#include <iostream>
#include <thread>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"

// using namespace std::literals::chrono_literals;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

typedef pcl::PointXYZRGBA PointTypeRGBA;
typedef pcl::PointCloud<PointTypeRGBA> PointCloudRGBA;
typedef pcl::PointCloud<PointTypeRGBA>::Ptr PointCloudPtrRGBA;

bool got_map = false;

bool width_calculation_finished = false;  // to reset set to true
double field_width_out = -1;              // The value to be published
int number_of_samples = 0;                // Used for setting final value
const int MIN_NUMBER_OF_SAMPLES = 30;     // Stops after this number is reached

PointCloud map_objects_msg;

void color_filter(PointCloudPtr& cloud_in, PointCloudPtr& cloud_out, int r,
                  int g, int b) {
    // Filters pointcloud by a specific color
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<PointType> extract;
    for (int i = 0; i < (*cloud_in).size(); i++) {
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

void map_callback(const PointCloud::ConstPtr& msg) {
    // ROS_INFO("Got new map");
    map_objects_msg = *msg;
    got_map = true;
}

bool got_messages() { return got_map; }

void process_messages() {
    // initialize PointClouds
    PointCloudPtr cloud(new PointCloud(map_objects_msg));
    PointCloudPtr cloud_out(new PointCloud);
    PointCloudPtr cloud_out_second(new PointCloud);
    PointCloudPtr cloud_green_1(new PointCloud);
    PointCloudPtr cloud_green_2(new PointCloud);

    color_filter(cloud, cloud_green_1, 0, 255, 0);

    if (cloud_green_1->points.size() < 5) {
        // Too few points
        return;
    }

    // First Line:

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);

    seg.setInputCloud(cloud_green_1);
    seg.segment(*inliers, *coefficients);

    // std::cout << *coefficients << std::endl;

    // copies all inliers of the model computed to another PointCloud
    // pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

    // Create the filtering object
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud_green_1);
    extract.setIndices(inliers);

    // Extract the inliers
    extract.setNegative(false);
    extract.filter(*cloud_out);

    extract.setNegative(true);
    extract.filter(*cloud_green_2);

    if (cloud_green_2->points.size() < 4) {
        // Too few points
        return;
    }

    // Second Line:

    pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg2;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);

    seg.setInputCloud(cloud_green_2);
    seg.segment(*inliers2, *coefficients2);

    // std::cout << *coefficients2 << std::endl;

    // Create the filtering object
    pcl::ExtractIndices<PointType> extract2;
    extract2.setInputCloud(cloud_green_2);
    extract2.setIndices(inliers2);

    // Extract the inliers
    extract2.setNegative(false);
    extract2.filter(*cloud_out_second);

    if (coefficients->values.size() == 0 || coefficients2->values.size() == 0) {
        // Two lines were not found
        return;
    }

    // Calculate distance and angle between found lines

    std::vector<float> u = coefficients->values;
    std::vector<float> v = coefficients2->values;

    double distance =
        fabs(u[4] * v[0] - u[3] * v[1] + (u[3] * u[1] - u[0] * u[4])) /
        sqrt(pow(u[4], 2) + pow(u[3], 2));
    double distance2 =
        fabs(v[4] * u[0] - v[3] * u[1] + (v[3] * v[1] - v[0] * v[4])) /
        sqrt(pow(v[4], 2) + pow(v[3], 2));
    double mean_distance = (distance + distance2) / 2;

    double angle = fabs( acos( ( u[3] * v[3] + u[4] * v[4] )
					/ sqrt( ( pow( u[3], 2 ) + pow( u[4], 2 ) ) * ( pow( v[3], 2 ) + pow( v[4], 2 ) ) ) ) );  

	std::cout << "The angle between the two lines is " << angle << "[rad]" << std::endl;

    // std::cout << "distance = " << distance << std::endl;
    // std::cout << "distance2 = " << distance2 << std::endl;

    // Check if measuremt is good
    if (mean_distance < 10 && mean_distance > 2 && angle < 10.0 * 3.1415 / 180) {
        ROS_INFO_STREAM("Field width = " << mean_distance
                                           << ", samples so far: "
                                           << number_of_samples);

        if (field_width_out == -1) {
            // First run
            field_width_out = mean_distance;
            number_of_samples = 1;
        } else {
            // Calculate average to get a good estimate
            field_width_out =
                27.0 / 30.0 * field_width_out + 3.0 / 30.0 * mean_distance;
            number_of_samples++;
        }
    }

    if (number_of_samples > MIN_NUMBER_OF_SAMPLES) {
        width_calculation_finished = true;
        ROS_INFO_STREAM("Final field width ready");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "field_width_node");
    std::unique_ptr<ros::NodeHandle> n;

    n = std::make_unique<ros::NodeHandle>();
    ros::Publisher width_publisher;
    width_publisher =
        n->advertise<geometry_msgs::Vector3>("field_width_node/width", 1000);

    ROS_INFO("Waiting for map_node/map");
    ros::topic::waitForMessage<PointCloud>("map_node/map");
    ros::Subscriber map_sub = n->subscribe("map_node/map", 1, map_callback);

    // 20 Hz loop
    ros::Rate r(20);
    while (ros::ok()) {
        if (got_messages() && !width_calculation_finished) {
            process_messages();
        }

        if (field_width_out != -1) {
            // Width value set
            geometry_msgs::Vector3 msg;
            msg.x = field_width_out;
            width_publisher.publish(msg);
        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
