#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

typedef pcl::PointXYZRGBA PointTypeRGBA;
typedef pcl::PointCloud<PointTypeRGBA> PointCloudRGBA;
typedef pcl::PointCloud<PointTypeRGBA>::Ptr PointCloudPtrRGBA;

float to_pcl_rgb(uint8_t r, uint8_t g, uint8_t b) {
    // pack r/g/b into rgb
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    return *reinterpret_cast<float*>(&rgb);
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

    // Origo is defined in the right home corner, the positive y-axis goes along
    // the width of the field and the positive x-axis along the length of the
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

    double field_lenght = field_width * 5 / 3;

    // Add poles
    for (double y = 0; y <= field_width; y += field_width) {
        pole_point.y = y;

        pole_point.x = 0;
        field->points.push_back(pole_point);
        pole_point.x = 0.1 * field_lenght;
        field->points.push_back(pole_point);
        pole_point.x = 0.25 * field_lenght;
        field->points.push_back(pole_point);
        pole_point.x = 0.5 * field_lenght;
        field->points.push_back(pole_point);
        pole_point.x = 0.75 * field_lenght;
        field->points.push_back(pole_point);
        pole_point.x = 0.9 * field_lenght;
        field->points.push_back(pole_point);
        pole_point.x = field_lenght;
        field->points.push_back(pole_point);
    }

    // Add goals
    for (double y = field_width / 2 - 0.5; y <= field_width / 2 - 0.5 + 1;
         y++) {
        goal_point_cyan.y = y;
        goal_point_orange.y = y;

        if (is_blue_team) {
            goal_point_cyan.x = 0.1 * field_lenght;
            field->points.push_back(goal_point_cyan);
            goal_point_cyan.x = 0.1 * field_lenght + 0.5;
            field->points.push_back(goal_point_cyan);

            goal_point_orange.x = 0.9 * field_lenght;
            field->points.push_back(goal_point_orange);
            goal_point_orange.x = 0.9 * field_lenght - 0.5;
            field->points.push_back(goal_point_orange);
        } else {
            goal_point_orange.x = 0.1 * field_lenght;
            field->points.push_back(goal_point_orange);
            goal_point_orange.x = 0.1 * field_lenght + 0.5;
            field->points.push_back(goal_point_orange);

            goal_point_cyan.x = 0.9 * field_lenght;
            field->points.push_back(goal_point_cyan);
            goal_point_cyan.x = 0.9 * field_lenght - 0.5;
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