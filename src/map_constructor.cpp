kinect_pub =
    n->advertise<PointCloud>("pointcloud_node/detected_objects", 1);

 #include "ros/ros.h"
 #include <pcl_ros/point_cloud.h>
 #include <pcl/point_types.h>
 #include <vector>
typedef  std::vector<std::vector<std::std::vector<float>> Coordinates_Matrix;
typedef std::vector<std::vector<float>> Coordinates;
 const std::vector<std::vector<std::std::vector<float>> {

                            { { {0,0} {0,.5}, {0,1.25} , {0,2.5} , {0,3.75}, {0,4,5}, {0,5} },
                            { {3,0} {3,.5}, {3,1.25} , {3,2.5} , {3,3.75}, {3,4,5}, {3,5} }  },
                          } Real_Map_Vecor;


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

  std::vector<std::vector<float>> final_map_array;
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZ> icp;

  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> final;
  icp.align(Final);

//  return &final;

 copy_to_array(final, final_map_array )



 }


void copy_to_array (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector<float>& deteced_map_array  )

{
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
        deteced_map_array[i][1] = cloud_in->point[i].x;
        deteced_map_array[i][2] = cloud_in->points[i].y;
    }




}
Coordinates distance (Coordinates a, Coordinates b)

{

  a[1]=a[1]-b[1];
  a[2]=a[2]-b[2]

  return (sqrt(abs ((a[1]-b[1])²-(a[2]-b[2])²)));

}

Coordinates maximum_likelihood (Coordinates detected_position)
{

  std::vector<flat> likelihood_vector;


  for (size_t i = 1; i<15; i++)
  {
        distance_vector.i = distance(detected_position.i, Real_Map_Vecor.i)


  }

  std::sort (distance_vector.begin(),distance_vector.end());
  return distance_vector[1];


}


int get_label (Coordinates pole_position)

{

-

  int label=0;




   for (size_t i=1 ; i<15, i++ )
   {
     if (Real_Map_Vecor.i == maximum_likelihood(pole_position) )
        {label=i;
        break;}
   }
return label;

}




std::vector<float> localize (std::vector<std::vector<float> > ideal_map, std::vector<std::vector<float> > detected_map   )

{
 // for ( )

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
