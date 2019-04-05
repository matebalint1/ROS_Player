#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPrt;

class PointcloudProcessor {
   public:
    PointcloudProcessor() {
        // Constructor
        pointcloud_temp = PointCloudPrt(new PointCloud);
        pointcloud_floor = PointCloudPrt(new PointCloud);
        pointcloud_not_floor = PointCloudPrt(new PointCloud);

        pass_through_filter = pcl::PassThrough<pcl::PointXYZRGB>();
        voxel_grid_filter = pcl::VoxelGrid<pcl::PointXYZRGB>();
    }

    ~PointcloudProcessor(){
        // Destructor
    }

    void pass_trough_filter_m(PointCloudPrt& cloud_in, PointCloudPrt& cloud_out,
                              std::string axis, double min, double max) {
        // Pass trough filter
        pass_through_filter.setInputCloud(cloud_in);
        pass_through_filter.setFilterFieldName(axis);
        pass_through_filter.setFilterLimits(min, max);
        pass_through_filter.filter(*cloud_out);
    }

    void voxel_grid_filter_m(PointCloudPrt& cloud_in, PointCloudPrt& cloud_out,
                           double leaf_size = 0.02) {
        // Voxel grid filter
        voxel_grid_filter.setInputCloud(cloud_in);
        voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid_filter.filter(*cloud_out);
    }

    void process_pointcloud(PointCloudPrt& cloud) {
        // Processes a new point cloud and extracts useful information
        voxel_grid_filter_m(cloud, pointcloud_temp);
        pass_trough_filter_m(pointcloud_temp, pointcloud_floor, "z", -0.2, 0.02);
        pass_trough_filter_m(pointcloud_temp, pointcloud_not_floor, "z", 0.02, 1);
    }

    PointCloudPrt& get_floor_pointcloud() { return pointcloud_floor; }
    PointCloudPrt& get_not_floor_pointcloud() { return pointcloud_not_floor; }

   private:
    pcl::PassThrough<pcl::PointXYZRGB> pass_through_filter;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;

    PointCloudPrt pointcloud_temp; // temporary
    PointCloudPrt pointcloud_floor;
    PointCloudPrt pointcloud_not_floor;
};