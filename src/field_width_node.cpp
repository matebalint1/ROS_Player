#include <iostream>
#include <cmath>
#include <thread>

#include "ros/ros.h"
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "geometry_msgs/Vector3.h"

using namespace std::literals::chrono_literals;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

typedef pcl::PointXYZRGBA PointTypeRGBA;
typedef pcl::PointCloud<PointTypeRGBA> PointCloudRGBA;
typedef pcl::PointCloud<PointTypeRGBA>::Ptr PointCloudPtrRGBA;

bool got_map = false;
PointCloud map_objects_msg;



void color_filter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out,
                      int r, int g, int b) {
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

bool got_messages() {
        return got_map;
    }

void process_messages(ros::Publisher& width_publisher)
{

    // initialize PointClouds
    PointCloudPtr cloud (new PointCloud(map_objects_msg));
    PointCloudPtr cloud_out (new PointCloud);
    PointCloudPtr cloud_out_second (new PointCloud);
    PointCloudPtr cloud_green_1 (new PointCloud);
    PointCloudPtr cloud_green_2 (new PointCloud);


   color_filter( cloud, cloud_green_1, 0, 255, 0 );

    
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); 
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices); 
   // Create the segmentation object 
   pcl::SACSegmentation<pcl::PointXYZRGB> seg; 
   // Optional 
   seg.setOptimizeCoefficients (true); 
   // Mandatory 
   seg.setModelType (pcl::SACMODEL_LINE); 
   seg.setMethodType (pcl::SAC_RANSAC); 
   seg.setDistanceThreshold (0.1); 

   seg.setInputCloud (cloud_green_1); 
   seg.segment (*inliers, *coefficients); 

	
std::cout << *coefficients << std::endl;


    // copies all inliers of the model computed to another PointCloud
    //pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

 // Create the filtering object
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(cloud_green_1);
        extract.setIndices(inliers);

        // Extract the inliers
        extract.setNegative(false);
        extract.filter(*cloud_out);



		extract.setNegative(true);
		extract.filter(*cloud_green_2);

		pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients); 
		pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices); 
	   // Create the segmentation object 
	   pcl::SACSegmentation<pcl::PointXYZRGB> seg2; 
	   // Optional 
	   seg.setOptimizeCoefficients (true); 
	   // Mandatory 
	   seg.setModelType (pcl::SACMODEL_LINE); 
	   seg.setMethodType (pcl::SAC_RANSAC); 
	   seg.setDistanceThreshold (0.1); 

	   seg.setInputCloud (cloud_green_2); 
	   seg.segment (*inliers2, *coefficients2); 


std::cout << *coefficients2 << std::endl;

 // Create the filtering object
        pcl::ExtractIndices<PointType> extract2;
        extract2.setInputCloud(cloud_green_2);
        extract2.setIndices(inliers2);

        // Extract the inliers
        extract2.setNegative(false);
        extract2.filter(*cloud_out_second);




    pcl::visualization::PCLVisualizer viewer ("visualize");
    viewer.addPointCloud (cloud_green_1,  "scene_cloud0");
    viewer.addPointCloud (cloud_out,  "scene_cloud1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_cloud0");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "scene_cloud1");
    viewer.addCoordinateSystem();

    pcl::visualization::PCLVisualizer viewer2 ("visualize");
    viewer2.addPointCloud (cloud_green_2,  "scene_cloud2");
    viewer2.addPointCloud (cloud_out_second,  "scene_cloud3");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_cloud2");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "scene_cloud3");
    viewer2.addCoordinateSystem();

	std::vector<float> u = coefficients->values;
	std::vector<float> v = coefficients2->values;

	double distance = fabs( u[4] * v[0] - u[3] * v[1] + ( u[3] * u[1] - u[0] * u[4] ) ) / sqrt( pow( u[4], 2) + pow( u[3], 2) );
	double distance2 = fabs( v[4] * u[0] - v[3] * u[1] + ( v[3] * v[1] - v[0] * v[4] ) ) / sqrt( pow( v[4], 2) + pow( v[3], 2) );
	double mean_distance = ( distance + distance2 ) / 2;

	std::cout << "distance = " << distance << std::endl;
	std::cout << "distance2 = " << distance2 << std::endl;
	std::cout << "mean_distance = " << ( distance + distance2 ) / 2 << std::endl;

	geometry_msgs::Vector3 msg;
	msg.x = mean_distance;
	width_publisher.publish(msg);
 
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "field_width_node");
	std::unique_ptr<ros::NodeHandle> n;

    n = std::make_unique<ros::NodeHandle>();
	ros::Publisher width_publisher;
	width_publisher = n->advertise<geometry_msgs::Vector3>	("field_width_node/Width", 1000);


	ROS_INFO("Waiting for map");
    ros::topic::waitForMessage<PointCloud>("map_node/map");

	ros::Subscriber map_sub = n->subscribe("map_node/map", 1, map_callback);


    // 20 Hz loop
    ros::Rate r(20);
    while (ros::ok()) {
        if (got_messages()) {
            process_messages(width_publisher);
        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;


  //  while (!viewer.wasStopped ())
  //  {
  //      viewer.spinOnce (100);
  //      std::this_thread::sleep_for(100ms);
  //  }
  
}
