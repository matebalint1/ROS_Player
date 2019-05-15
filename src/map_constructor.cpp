kinect_pub =
    n->advertise<PointCloud>("pointcloud_node/detected_objects", 1);

 #include "ros/ros.h"
 #include "std_msgs/String.h"
 #include <pcl_ros/point_cloud.h>
 #include <pcl/point_types.h>
 #include <vector>


 private:
   pcl::PointCloud<pcl::PointXYZRGB> pcl_detected_objects;



public:

    void pcl_Callback(const PointCloud::ConstPtr &msg)
    {
      pcl_detected_objects = *msg;

    }

pcl::PointCloud<pcl::PointXYZRGB>



 pcl::PointCloud<pcl::PointXYZRGB>::Ptr::ConstPtr& recunstruct(
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr::ConstPtr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);

 {

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  return &icp;


 }




 pcl::PointCloud<pcl::PointXYZRGB>::Ptr::ConstPtr&  recunstruct(
   vector<std::vector<float>> detected_map, vector<std::vector<float>> ideal_map);

 {

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  return &Final;


 }


std::vector<std::vector<int>> v;<float> copy_to_array (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector<float> deteced_map_array  )

{
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
        deteced_map_array[i][1] = cloud_in->point[i].x;
        deteced_map_array[i][2] = cloud_in->points[i].y;
    }

    return deteced_map_array;


}

std::vector<std::vector<float>> ideal_map_array ()

{
  // Full width is assumed to be 5 units;
  vector<vector<float> > vect{
                             { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
                             { 0, .5, 1.25, 2.5, 3.75, 4,5, 5, 0, .5, 1.25, 2.5, 3.75, 4,5, 5  },
                             };


    return ideal_map_array;


}


std::vector<float> localize (std::vector<std::vector<float> > ideal_map, std::vector<std::vector<float> > detected_map   )

{
 for ( )

}







  int main(int argc, char **argv)
   {


    ros::init(argc, argv, "remapper_node");
    ros::NodeHandle n;

    ros::Subscriber sub ("pointcloud_node/detected_objects", 1, pcl_Callback);
    ros::Publisher remap_pub = n.advertise <pcl::PointCloud<pcl::PointXYZRGB>> ("remapper_node/out_map");
    ros::Publisher remap_pub = n.advertise <pcl::PointCloud<pcl::PointXYZRGB>> ("remapper_node/differenceVector");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detected_map (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ideal_map (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr reconstruced_map=
      recunstruct(detected_map, ideal_map);

    ros::spin();


  }
