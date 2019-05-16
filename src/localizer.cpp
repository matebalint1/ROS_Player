
 #include <vector>
 #include <math.h>

 #include "ros/ros.h"
 #include <pcl/point_cloud.h>
 #include <pcl_ros/point_cloud.h>
 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/registration/icp.h>
 #include <sensor_msgs/PointCloud2.h>
 #include <tf/transform_broadcaster.h>

 // #include <pcl/point_types.h>

 #include "pointcloud_helpers.hpp"


typedef  std::vector<std::vector<float>> Coordinates_Vector;
typedef  std::vector<float> Coordinates;
const Coordinates_Vector Real_Map_Vector
                             { {0,0}, {0,.5}, {0,1.25} , {0,2.5} , {0,3.75}, {0,4,5}, {0,5},
                             {3,0}, {3,.5}, {3,1.25} , {3,2.5} , {3,3.75}, {3,4,5}, {3,5} } ;






   pcl::PointCloud<pcl::PointXYZRGB> pcl_detected_objects;




    void pcl_Callback(const PointCloud::ConstPtr &msg)
    {
      pcl_detected_objects = *msg;

    }



 /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr::ConstPtr& recunstruct(
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr::ConstPtr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);

 {

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  return &icp;*/

  void copy_to_array (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, Coordinates_Vector& deteced_map_array  )

  {
      for (size_t i = 1; i < cloud_in->points.size (); ++i)
      {
          deteced_map_array[i][1] = cloud_in->points[i].x;
          deteced_map_array[i][2] = cloud_in->points[i].y;
      }
}





 void reconstruct(
   pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr   cloud_out)

 {

  std::vector<std::vector<float>> final_map_array;
  pcl::IterativeClosestPoint<PointType, PointType> icp;

  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<PointType>::Ptr final (new PointCloud);
  icp.align(*final);

//  return &final;

 copy_to_array(final, final_map_array );



 }

 float distance (Coordinates a, Coordinates b)

 {



   return (sqrt(abs (pow((a[1]-b[1]),2)-pow((a[2]-b[2]),2))));

 }

int index_of_minimumDistance (std::vector<float> distance_vector)
{
float min;
int index;
 for(size_t i=1;i<15;i++)
   {
       if(distance_vector[i]<min)
       min=distance_vector[i];
   }

   for(size_t i=1;i<15;i++)
     {
         if(distance_vector[i]==min)
          index = i;
     }
     return index;

}



Coordinates maximum_likelihood (Coordinates detected_position)
{

  std::vector<float> distance_vector;


  for (size_t i = 1; i<15; i++)
  {
        distance_vector[i] = distance(detected_position, Real_Map_Vector[i]);


  }


//  std::sort (distance_vector.begin(),distance_vector.end());
  return Real_Map_Vector[index_of_minimumDistance (distance_vector)];


}


int get_label (Coordinates pole_position)

{



  int label=0;




   for (size_t i=1 ; i<15; i++ )
   {
     if (Real_Map_Vector[i] == maximum_likelihood(pole_position) )
        {label=i;
        break;}
   }
return label;

}




/*std::vector<float> localize (std::vector<std::vector<float> > ideal_map, std::vector<std::vector<float> > detected_map   )

{
 // for ( )

}*/








  //void get_field_components ( PointCloudPtr cloud_in  )



  void tf_map_to_odom_boardcaster(double x, double y, double yaw) {
          static tf::TransformBroadcaster transform_broadcaster;
          // Quaternion from yaw
          geometry_msgs::Quaternion odom_quat =
              tf::createQuaternionMsgFromYaw(yaw);

          // Message
          geometry_msgs::TransformStamped odom_trans;
          odom_trans.header.stamp = ros::Time::now();

          odom_trans.header.frame_id = "robot1/map";
          odom_trans.child_frame_id = "robot1/odom";

          odom_trans.transform.translation.x = x;
          odom_trans.transform.translation.y = y;
          odom_trans.transform.translation.z = 0.0;
          odom_trans.transform.rotation = odom_quat;

          // Send the transform
          transform_broadcaster.sendTransform(odom_trans);
      }



  int main(int argc, char **argv)
   {

    ros::init(argc, argv, "localization_node");
    ros::NodeHandle n;


    ros::Subscriber sub = n.subscribe ("map_node/map", 1, pcl_Callback);

    //ros::Publisher remap_pub = n.advertise <Coordinates> ("remapper_node/out_map");

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr detected_map (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr ideal_map (new pcl::PointCloud<pcl::PointXYZRGB>);

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr reconstruced_map=
    //  recunstruct(detected_map, ideal_map);

    ros::spin();


  }
