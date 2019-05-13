#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

typedef pcl::PointXYZRGBA PointTypeRGBA;
typedef pcl::PointCloud<PointTypeRGBA> PointCloudRGBA;
typedef pcl::PointCloud<PointTypeRGBA>::Ptr PointCloudPtrRGBA;



PointCloudPtr get_ideal_fiel_cloud(double field_width) {
    // This function generates an pointcloud of the hockey field based on the
    // given width. Different objects are representet with differently colored
    // points.




}