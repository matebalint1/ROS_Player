#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#pragma once

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

typedef pcl::PointXYZRGBA PointTypeRGBA;
typedef pcl::PointCloud<PointTypeRGBA> PointCloudRGBA;
typedef pcl::PointCloud<PointTypeRGBA>::Ptr PointCloudPtrRGBA;

bool get_transform(tf::Transform &transform, tf2_ros::Buffer *&tfBuffer,
                   std::string goal_frame, std::string current_frame) {
    // Returns true if succesful

    geometry_msgs::TransformStamped transformStamped;
    try {
        // Find transformation for pointcloud from the buffer
        // (*cur_kinect_in).header.frame_id ==
        // "robot1/kinect_rgb_optical_frame"
        transformStamped =
            tfBuffer->lookupTransform(goal_frame, current_frame, ros::Time(0));

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
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
    return true;
}

void voxel_grid_filter_m(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out,
                         double leaf_size = 0.02, int min_n_points = 0) {
    pcl::VoxelGrid<PointType> voxel_grid_filter = pcl::VoxelGrid<PointType>();
    // Voxel grid filter
    voxel_grid_filter.setInputCloud(cloud_in);
    voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.setMinimumPointsNumberPerVoxel(min_n_points);
    voxel_grid_filter.filter(*cloud_out);
}

void voxel_grid_filter_m(PointCloudPtrRGBA &cloud_in,
                         PointCloudPtrRGBA &cloud_out, double leaf_size = 0.02,
                         int min_n_points = 0) {
    pcl::VoxelGrid<PointTypeRGBA> voxel_grid_filter =
        pcl::VoxelGrid<PointTypeRGBA>();
    // Voxel grid filter
    voxel_grid_filter.setInputCloud(cloud_in);
    voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.setMinimumPointsNumberPerVoxel(min_n_points);
    voxel_grid_filter.filter(*cloud_out);
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

PointTypeRGBA get_centroid_of_color(PointCloudPtrRGBA &cloud, int r, int g,
                                    int b) {
    // Filters pointcloud by a specific color
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<PointTypeRGBA> extract;
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

    PointCloudPtrRGBA color_cloud(new PointCloudRGBA);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*color_cloud);

    PointTypeRGBA centroid;
    pcl::computeCentroid(*color_cloud, centroid);
    return centroid;
}

float to_pcl_rgb(uint8_t r, uint8_t g, uint8_t b) {
    // pack r/g/b into rgb
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    return *reinterpret_cast<float *>(&rgb);
}

void save_cloud_to_file(PointCloudPtr cloud, std::string path_and_name) {
    if (cloud->points.size() > 0) {
        pcl::io::savePCDFileASCII(path_and_name, *cloud);
    }
}

void save_cloud_to_file(PointCloudPtrRGBA cloud, std::string path_and_name) {
    if (cloud->points.size() > 0) {
        pcl::io::savePCDFileASCII(path_and_name, *cloud);
    }
}

PointCloudPtr get_ideal_field_cloud(double field_width, bool is_blue_team) {
    // This function generates an pointcloud of the hockey field based on the
    // given width. Different objects are representet with differently colored
    // points. It includes poles and goals == fixed structures on the field.

    // Origo is defined in the left home corner, the positive y-axis goes along
    // the length of the field and the positive x-axis along the widt of the
    // field.

    PointCloudPtr field = PointCloudPtr(new PointCloud);
    PointType pole_point;
    PointType goal_point_cyan;
    PointType goal_point_orange;

    pole_point.z = 0.2;
    goal_point_cyan.z = 0.2;
    goal_point_orange.z = 0.2;

    pole_point.rgb = to_pcl_rgb(0, 255, 0);
    goal_point_cyan.rgb = to_pcl_rgb(0, 255, 255);
    goal_point_orange.rgb = to_pcl_rgb(255, 140, 0);

    double field_lenght = field_width * 5.0 / 3.0;

    // Add poles
    for (double x = 0; x <= field_width; x += field_width) {
        pole_point.x = x;

        pole_point.y = 0;
        field->points.push_back(pole_point);
        pole_point.y = 0.1 * field_lenght;
        field->points.push_back(pole_point);
        pole_point.y = 0.25 * field_lenght;
        field->points.push_back(pole_point);
        pole_point.y = 0.5 * field_lenght;
        field->points.push_back(pole_point);
        pole_point.y = 0.75 * field_lenght;
        field->points.push_back(pole_point);
        pole_point.y = 0.9 * field_lenght;
        field->points.push_back(pole_point);
        pole_point.y = field_lenght;
        field->points.push_back(pole_point);
    }

    // Add goals
    for (double x = field_width / 2 - 0.5; x <= field_width / 2 - 0.5 + 1;
         x++) {
        goal_point_cyan.x = x;
        goal_point_orange.x = x;

        if (is_blue_team) {
            goal_point_cyan.y = 0.1 * field_lenght;
            field->points.push_back(goal_point_cyan);
            goal_point_cyan.y = 0.1 * field_lenght + 0.5;
            field->points.push_back(goal_point_cyan);

            goal_point_orange.y = 0.9 * field_lenght;
            field->points.push_back(goal_point_orange);
            goal_point_orange.y = 0.9 * field_lenght - 0.5;
            field->points.push_back(goal_point_orange);
        } else {
            goal_point_orange.y = 0.1 * field_lenght;
            field->points.push_back(goal_point_orange);
            goal_point_orange.y = 0.1 * field_lenght + 0.5;
            field->points.push_back(goal_point_orange);

            goal_point_cyan.y = 0.9 * field_lenght;
            field->points.push_back(goal_point_cyan);
            goal_point_cyan.y = 0.9 * field_lenght - 0.5;
            field->points.push_back(goal_point_cyan);
        }
    }

    field->is_dense = false;
    field->width = 1;
    field->height = field->points.size();

    return field;
}

/*
void generate_puck_pointcloud(PointCloudPtr& cloud) {
    double radius = 0.05;  // m
    double radius_second_part = 0.0325;
    double radial_step = 0.002;
    double height = 0.18;  // m
    double height_second_part = 0.17;
    double height_step = 0.002;
    cloud->clear();

    // Lower part
    for (double h = 0; h <= height; h += height_step) {
        for (double a = 0; a <= 2 * 3.14 / 3.0; a += radial_step / radius) {
            double x = radius * cos(a);
            double y = radius * sin(a);

            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = h;
            uint32_t rgb =
                ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
            point.rgb = rgb;
            cloud->points.push_back(point);
        }
    }

    // Upper part
    for (double h = height; h <= height + height_second_part;
         h += height_step) {
        for (double a = 0; a <= 2 * 3.14 / 3.0;
             a += radial_step / radius_second_part) {
            double x = radius_second_part * cos(a);
            double y = radius_second_part * sin(a);

            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = h;
            uint32_t rgb =
                ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
            point.rgb = rgb;
            cloud->points.push_back(point);
        }
    }
    cloud->is_dense = false;
    cloud->width = 1;
    cloud->height = cloud->points.size();
}

void generate_pole_pointcloud(PointCloudPtr& cloud) {
    double radius = 0.0425;  // m
    double radial_step = 0.002;
    double height = 0.5;  // m
    double height_step = 0.002;
    cloud->clear();

    // Lower part
    for (double h = 0; h <= height; h += height_step) {
        for (double a = 0; a <= 2 * 3.14 / 3.0; a += radial_step / radius) {
            double x = radius * cos(a);
            double y = radius * sin(a);

            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = h;
            uint32_t rgb =
                ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
            point.rgb = rgb;
            cloud->points.push_back(point);
        }
    }
    cloud->is_dense = false;
    cloud->width = 1;
    cloud->height = cloud->points.size();
}
*/