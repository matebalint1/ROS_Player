
#include <vector>
#include <math.h>
//#include <iostream.h>

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

typedef std::vector<double> Coordinates;
typedef struct Coordinate_c_s
{
  Coordinates point;
  char color;
} Coordinates_Colored;

typedef std::vector<std::vector<double>> Coordinates_Vector;
typedef std::vector<Coordinates_Colored> Coordinates_Vector_Colored;

bool got_map = false;
bool got_width = false;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_detected_objects;

void pcl_Callback(const PointCloud::ConstPtr &msg)
{
  pcl_detected_objects = PointCloudPtr(new PointCloud(*msg));
  
  got_map = true;
  //ROS_INFO_STREAM ("Got map already!");
}

double width;

void width_Callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  width = msg->x;
  got_width = true;
}



Coordinates_Vector Real_Map_Vector{{0, 0}, {.5,0}, {1.25,0}, {2.5,0}, {3.75,0}, {4.5,0}, {5,0}, {0,3}, {.5,3}, {1.25 ,3}, {2.5,3}, {3.75,3}, {4.5,3}, {5,3}};

Coordinates_Vector Default_Goals_Vector{{.5 *1 +.25,.5}, {4.5 * 1 - .25,.5}}; // Takes the left most point in the middle of
                                                                               // yellow goal area and the right most point in the middle
                                                                               // of the blue goal area as beginning and end point of
                                                                               // the goal vector.
void scale(Coordinates_Vector &vector_in)
{
  for (auto i : vector_in)
  {
    for (auto j : i)
    {
      j =   width / 3 * j;
    }
  }
}


void scale_down(Coordinates_Vector &vector_in)
{
  for (auto i : vector_in)
  {
    for (auto j : i)
    {
      j =   3 / width * j;
    }
  }
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
  if (rgbMatrix(0) == 0 && rgbMatrix(1) * rgbMatrix(2) != 0)
    return 'b';
  else if (rgbMatrix(0) == 0 && rgbMatrix(2) == 0)
    return 'g';
  else
    return 'y';
}


void flush_array (Coordinates_Vector_Colored &detected_map_array)

{
  for (auto i:detected_map_array)
  {
    detected_map_array.pop_back();
  }
}

void copy_to_array(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, Coordinates_Vector_Colored &detected_map_array)

{
  
  for (size_t i = 0; i < cloud_in->points.size(); i++)
  {
    //ROS_INFO_STREAM("Now Copying Coordinates!");
    Coordinates_Colored coordinate; 
    Coordinates point (2);
    point[0]=cloud_in->points[i].x;
    point[1]=cloud_in->points[i].y;
    coordinate.point=point;
    coordinate.color= get_color(cloud_in->points[i]);
    detected_map_array.push_back(coordinate); 
    //std::cout << "Map Size is:" << detected_map_array.size();
    
    
    
    //ROS_INFO_STREAM("Succeeded Copying the Point.");
  }

  //ROS_INFO_STREAM("Succeeded Cloning the Map.");


}

/*void reconstruct(
   pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr   cloud_out)

 {

  std::vector<std::vector<double>> final_map_array;
  pcl::IterativeClosestPoint<PointType, PointType> icp;

  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<PointType>::Ptr final (new PointCloud);
  icp.align(*final);

//  return &final;

 copy_to_array(final, final_map_array );



 }*/

double absolute(Coordinates_Vector in)
{
  return sqrt(abs(pow((in[1][1] - in[0][1]), 2) + pow((in[1][0] - in[0][0]), 2)));
  //return result;}
}

double dot(Coordinates in1, Coordinates in2)
{
  return in1[0] * in2[0] + in1[1] * in2[1];
}
Coordinates distance(Coordinates a, Coordinates b)

{

  return {a[0] - b[0], a[1] - b[1]};
}

Coordinates add(Coordinates a, Coordinates b)

{

  return {a[0] + b[0], a[1] + b[1]};
}

int index_of_minimumDistance(std::vector<double> distance_vector)
{
  double min = distance_vector[0];
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

  std::vector<double> distance_vector;

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

/*std::vector<double> localize (std::vector<std::vector<double> > ideal_map, std::vector<std::vector<double> > detected_map   )

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

Coordinates_Vector get_goalsVector(Coordinates_Vector_Colored map_in)
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

  return {vector_s, vector_e};
}

Coordinates get_translation(Coordinates_Vector_Colored map_in)
{
  //ROS_INFO_STREAM ("Calculating the translation!");
  Coordinates_Vector goals_vector = get_goalsVector(map_in);
  return distance(Default_Goals_Vector[0], goals_vector[0]);
  
}

Coordinates_Vector_Colored translate(Coordinates_Vector_Colored map_in)
{
  Coordinates_Vector_Colored translated_map;

  Coordinates translation = get_translation(map_in);
  for (size_t i = 1; i < map_in.size(); i++)
  {
    translated_map[i].point = add(map_in[i].point, translation);
    translated_map[i].color = map_in[i].color;
  }
  return translated_map;
}

double get_rotation(Coordinates_Vector_Colored map_in)
{

  return acos(dot(distance(get_goalsVector(map_in)[1], get_goalsVector(map_in)[0]),
                  distance(Default_Goals_Vector[1], Default_Goals_Vector[0])) /
              (absolute(get_goalsVector(map_in)) * absolute(Default_Goals_Vector)));
}

Coordinates_Vector_Colored rotate(Coordinates_Vector_Colored map_in)
{

  double alpha = get_rotation(map_in);
  Coordinates_Vector_Colored map_result;
  //ROS_INFO_STREAM ("Getting the rotation!");
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

void print_colored_coordinates (Coordinates_Vector_Colored vector_in)
{
  
 // std::cout <<"Chechickinng: "<<  vector_in[0].point[0];
  for (auto point_colored: vector_in)
  {
    std::cout << " X: "  <<  point_colored.point[0];// << "  Y:" ; 
               //point_colored.point[1]//<< " Color: " << point_colored.color << std::endl;
  }

  
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
  //ROS_INFO_STREAM("STARTING");

  ros::init(argc, argv, "localization_node");
  ros::NodeHandle n;

  ros::Subscriber map_sub = n.subscribe("map_node/map", 1, pcl_Callback);
  ros::Subscriber dimension_sub = n.subscribe("field_width_node/width", 1, width_Callback);
  ros::Rate r(20);
  int clock=0;

  Coordinates_Vector_Colored map_in(0);
  while (ros::ok())
  {
    if (clock % 400 == 7)
      clock=0;
    if (got_width == true)
    {
      scale(Default_Goals_Vector);
      scale(Real_Map_Vector);

     // ROS_INFO_STREAM("Scaling Successful!");
    }

    

    
   // ROS_INFO_STREAM("");

    if (got_map == true){
        
      copy_to_array(pcl_detected_objects, map_in);
      //if (clock==0) print_colored_coordinates(map_in);
    

    Coordinates translation = get_translation(map_in);
    double rotation = get_rotation(map_in);// * 180 / M_PI;
    tf_map_to_odom_boardcaster(translation[0], translation[1], rotation);
    
    
    
    scale_down (Real_Map_Vector);
    scale_down (Default_Goals_Vector);
    flush_array(map_in);
    got_map=false;
    got_width=false;
    //std::cout << std::endl <<std::endl <<std::endl;

    
  
      std::cout << "Translation X:"<< translation[0] <<" Translation Y:" << translation[1]<< " Rotation:" << rotation*180/M_PI << std::endl;    

    }
    //ros::Publisher remap_pub = n.advertise <Coordinates> ("remapper_node/out_map");

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr detected_map (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr ideal_map (new pcl::PointCloud<pcl::PointXYZRGB>);

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr reconstruced_map=
    //  recunstruct(detected_map, ideal_map);

    ros::spinOnce();
    clock++;
    r.sleep();
  }
}
