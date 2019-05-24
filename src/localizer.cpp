
#include <vector>
#include <math.h>

#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

// #include <pcl/point_types.h>

#include "pointcloud_helpers.hpp"

typedef std::vector<float> Coordinates;
typedef struct Coordinate_c_s
{
  Coordinates point;
  char color;
} Coordinates_Colored;

typedef std::vector<std::vector<float>> Coordinates_Vector;
typedef std::vector<Coordinates_Colored> Coordinates_Vector_Colored;

const Coordinates_Vector Real_Map_Vector{{0, 0}, {0, .5}, {0, 1.25}, {0, 2.5}, {0, 3.75}, {0, 4, 5}, {0, 5}, {3, 0}, {3, .5}, {3, 1.25}, {3, 2.5}, {3, 3.75}, {3, 4, 5}, {3, 5}};

const Coordinates Default_Goals_Vector{0, 4}; // Takes the left most point in the middle of
                                                                        // yellow goal area and the right most point in the middle
                                                                        // of the blue goal area as beginning and end point of
                                                                        // the goal vector.

pcl::PointCloud<pcl::PointXYZRGB> pcl_detected_objects;

void pcl_Callback(const PointCloud::ConstPtr &msg)
{
  pcl_detected_objects = *msg;
}

/*pcl::PCoordinates rotation_vectointCloud<pcl::PointXYZRGB>::Ptr::ConstPtr& recunstruct(
   pcl::PCoordinates rotation_vectointCloud<pcl::PointXYZRGB>::Ptr::ConstPtr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);

 {

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  return &icp;*/
char get_color(PointType point)
{

  Eigen::Vector3i rgbMatrix = point.getRGBVector3i();

  return 0;
}

void copy_to_array(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, Coordinates_Vector_Colored &detected_map_array)

{
  for (size_t i = 0; i < cloud_in->points.size(); ++i)
  {
    detected_map_array[i].point[0] = cloud_in->points[i].x;
    detected_map_array[i].point[1] = cloud_in->points[i].y;
    detected_map_array[i].color = get_color(cloud_in->points[i]);
  }
}

/*void reconstruct(
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



 }*/

Coordinates distance(Coordinates a, Coordinates b)

{

  return {a[0] - b[0], a[1] - b[1]};
  //return (sqrt(abs (pow((a[1]-b[1]),2)-pow((a[2]-b[2]),2))));
}

int index_of_minimumDistance(std::vector<float> distance_vector)
{
  float min = distance_vector[0];
  int index;
  for (auto i : distance_vector)
  {
    if (i < min)
      min = i;
  }

  for (size_t i = 0; i < distance_vector.size(); i++)
  {
    if (distance_vector[i] == min)
      index = i;
  }
  return index;
}

Coordinates maximum_likelihood(Coordinates detected_position)
{

  std::vector<float> distance_vector;

  for (size_t i = 0; detected_position.size(); i++)
  {

    distance_vector[i] = abs(pow(distance(detected_position, Real_Map_Vector[i])[0], 2) +
                             pow(distance(detected_position, Real_Map_Vector[i])[1], 2));
  }

  //  std::sort (distance_vector.begin(),distance_vector.end());
  return Real_Map_Vector[index_of_minimumDistance(distance_vector)];
}

int get_label(Coordinates pole_position)

{

  int label = 0;

  for (size_t i = 0; pole_position.size(); i++)
  {
    if (Real_Map_Vector[i] == maximum_likelihood(pole_position))
    {
      label = i;
      break;
    }
  }
  return label;
}

/*std::vector<float> localize (std::vector<std::vector<float> > ideal_map, std::vector<std::vector<float> > detected_map   )

{
 // for ( )

}*/

//void get_field_components ( PointCloudPtr cloud_in  )

void tf_map_to_odom_boardcaster(double x, double y, double yaw)
{
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

Coordinates get_goalsVector(Coordinates_Vector_Colored map_in)
{
  Coordinates vector_s = {0, 0};
  Coordinates vector_e = {0, 0};

  int count = 0;

  for (auto point_colored : map_in)
  {
    if (point_colored.color == 'b') //Start point of goals vector
    {
      ++count;
      vector_s[1] = vector_e[1] - point_colored.point[1];
      vector_s[0] = vector_e[0] - point_colored.point[0];
    }
  }
  vector_s[0] = vector_s[0] / count;
  vector_s[1] = vector_s[1] / count;
  count = 0;

  for (auto point_colored : map_in)
  {
    if (point_colored.color == 'y') //Start point of goals vector
    {
      ++count;
      vector_e[1] = vector_e[1] + point_colored.point[1];
      vector_e[0] = vector_e[0] + point_colored.point[0];
    }
  }

  vector_e[0] = vector_e[0] / count;
  vector_e[1] = vector_e[1] / count;

  return distance(vector_e, vector_s);
}


Coordinates_Vector_Colored translate(Coordinates_Vector_Colored map_in)
{
  Coordinates_Vector_Colored translated_map;
  Coordinates goals_vector_detected = get_goalsVector(map_in);
  for (size_t i = 1; i < goals_vector_detected.size(); i++)
  {
    translated_map[i].point= distance (map_in[i].point , distance(goals_vector_detected, Default_Goals_Vector));
  }
  return translated_map;
}

Coordinates_Vector_Colored rotate(Coordinates_Vector_Colored map_in)
{
  Coordinates detected_goals_vector = get_goalsVector(map_in);

  float alpha = acos(( detected_goals_vector[0]*Default_Goals_Vector[0]
        +detected_goals_vector[1]*Default_Goals_Vector[1])/ (sqrt(pow(Default_Goals_Vector[0],2)+pow(Default_Goals_Vector[1],2))*
                                                                sqrt(pow(detected_goals_vector[0],2)+pow(detected_goals_vector[0],2))));
  Coordinates_Vector_Colored map_result;

  for (size_t i = 0; i < map_in.size(); i++)

  {
    map_result[i].point[0] = map_in[i].point[0] * cos(alpha) - map_in[i].point[1] * sin(alpha);
    map_result[i].point[1] = map_in[i].point[1] * sin(alpha) + map_in[i].point[1] * cos(alpha);
    map_result[i].color = map_in[i].color;
  }

  return map_result;
}

Coordinates_Vector_Colored frame_rematch(Coordinates_Vector_Colored map_in)
{
  Coordinates_Vector_Colored map_in_editted = translate(map_in);
  map_in_editted = rotate(map_in_editted);

  return map_in_editted;
}

Coordinates_Vector_Colored reconstruct(pcl::PointCloud<PointType>::Ptr map_in)
{
  Coordinates_Vector_Colored map_in_copy, map_reconstructed;
  copy_to_array(map_in, map_in_copy);
  Coordinates_Vector_Colored map_recostructed = frame_rematch(map_in_copy);

  for (size_t i = 0; i < map_in_copy.size(); i++)
  {
    map_reconstructed[get_label(map_in_copy[i].point)] = map_in_copy[i];
  }

  return map_reconstructed;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "localization_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map_node/map", 1, pcl_Callback);

  //ros::Publisher remap_pub = n.advertise <Coordinates> ("remapper_node/out_map");

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr detected_map (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr ideal_map (new pcl::PointCloud<pcl::PointXYZRGB>);

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr reconstruced_map=
  //  recunstruct(detected_map, ideal_map);

  ros::spin();
}
