kinect_pub =
    n->advertise<PointCloud>("pointcloud_node/detected_objects", 1);

 #include "ros/ros.h"
 #include "std_msgs/String.h"
 #include <sstream>
 class Labeled_PCL
 {
  private:
     pcl::PointCloud<pcl::PointXYZRGB> pcl_RGB;
     std::unique_ptr<int[]>labels_enum;

  public:
    Labeled_PCL (pcl::PointCloud<pcl::PointXYZRGB> inPut_pcl)
    {
      pcl_RGB = inPut_pcl;
      labels_enum (new int[pcl_RGB.points.size]);
    }

  ~Labeled_PCL()
  {
    delete [] pcl_RGB;
  }


  }






 class remapper
 {
 private:
   pcl::PointCloud<pcl::PointXYZRGB> pcl_detected_objects;


public:




    void pcl_Callback(const PointCloud::ConstPtr &msg)
    {
      pcl_detected_objects = *msg;
    }



 int main(int argc, char **argv)
  {


   ros::init(argc, argv, "remapper_node");
   ros::NodeHandle n;
   ros::Subscriber sub ("pointcloud_node/detected_objects", 1, pcl_Callback);
   ros::Publisher remap_pub = n.advertise <pcl::PointCloud<pcl::PointXYZRGB>> ("remapper_node/out_map");
   ros::Publisher remap_pub = n.advertise <pcl::PointCloud<pcl::PointXYZRGB>> ("remapper_node/differenceVector");


   ros::spin();



   }
