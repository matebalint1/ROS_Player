
#include <math.h>
#include <vector>
//#include <iostream.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include "ros/ros.h"

// #include <pcl/point_types.h>

#include "pointcloud_helpers.hpp"

typedef std::vector<double> Coordinates;
typedef struct Coordinate_c_s {
    Coordinates point;
    char color;
    int label;
} Coordinates_Colored;

typedef std::vector<std::vector<double>> Coordinates_Vector;
typedef std::vector<Coordinates_Colored> Coordinates_Vector_Colored;

bool got_map = false;
bool got_width = false;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_detected_objects;

void pcl_Callback(const PointCloud::ConstPtr &msg) {
    pcl_detected_objects = PointCloudPtr(new PointCloud(*msg));

    got_map = true;
    // ROS_INFO_STREAM ("Got map already!");
}

double width;

void width_Callback(const geometry_msgs::Vector3::ConstPtr &msg) {
    width = msg->x;
    got_width = true;
}

Coordinates_Vector Real_Map_Vector{
    {0, 0}, {0, .5}, {0, 1.25}, {0, 2.5}, {0, 3.75}, {0, 4.5}, {0, 5},
    {3, 0}, {3, .5}, {3, 1.25}, {3, 2.5}, {3, 3.75}, {3, 4.5}, {3, 5}};

Coordinates_Vector Default_Goals_Vector{
    {.5 * 1 + .25, 1.5},
    {4.5 * 1 - .25, 1.5}};  // Takes the left most point in the middle of
                            // yellow goal area and the right most point in the
                            // middle of the blue goal area as beginning and end
                            // point of the goal vector.

void scale_Default_Goals() {
    Default_Goals_Vector = {{.5 * width / 3 + .25, 1.5},
                            {4.5 * width / 3 - .25, 1.5}};
}

void scale(Coordinates_Vector &vector_in) {
    for (auto i : vector_in) {
        for (auto j : i) {
            j = width / 3 * j;
        }
    }
}

void scale_down(Coordinates_Vector &vector_in) {
    for (auto i : vector_in) {
        for (auto j : i) {
            j = 3 / width * j;
        }
    }
}

/*pcl::PCoordinates rotation_vectointCloud<pcl::PointXYZRGB>::Ptr::ConstPtr&
 recunstruct( pcl::PCoordinates
 rotation_vectointCloud<pcl::PointXYZRGB>::Ptr::ConstPtr& cloud_in,
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);

 {

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  return &icp;*/
char get_color(PointType point) {
    Eigen::Vector3i rgbMatrix = point.getRGBVector3i();
    if (rgbMatrix(0) == 0 && rgbMatrix(1) * rgbMatrix(2) != 0)
        return 'b';
    else if (rgbMatrix(0) == 0 && rgbMatrix(2) == 0)
        return 'g';
    else if (rgbMatrix(0) == 255 && rgbMatrix(1)==140 && rgbMatrix(2) == 0)
        return 'y' ;
}

void flush_array(Coordinates_Vector_Colored &detected_map_array)

{
    for (auto i : detected_map_array) {
        detected_map_array.pop_back();
    }
}

double absolute(Coordinates_Vector in) {
    return sqrt(
        abs(pow((in[1][1] - in[0][1]), 2) + pow((in[1][0] - in[0][0]), 2)));
    // return result;}
}

Coordinates distance(Coordinates a, Coordinates b){
    return {a[0] - b[0], a[1] - b[1]};
}

int index_of_minimumDistance(std::vector<double> distance_vector) {
    // ROS_INFO_STREAM("DEBUG get_label_index_of");
    double min = distance_vector[0];
    int index;
    for (auto i : distance_vector) {
        if (i < min) min = i;
    }

    for (size_t i = 0; i < distance_vector.size(); i++) {
        if (distance_vector[i] == min) index = i;
    }
    return index;
}

Coordinates maximum_likelihood(Coordinates detected_position) {
    // ROS_INFO_STREAM("DEBUG get_label_maximumlikelihood");
    std::vector<double> distance_vector(0);

    for (size_t i = 0; i < Real_Map_Vector.size(); i++) {
        // ROS_INFO_STREAM("DEBUG get_label_maximumlikelihood_for");
        distance_vector.push_back(
            absolute({Real_Map_Vector[i], detected_position}));
    }

    //  std::sort (distance_vector.begin(),distance_vector.end());
    return Real_Map_Vector[index_of_minimumDistance(distance_vector)];
}

int get_label(Coordinates pole_position){
    // ROS_INFO_STREAM("DEBUG get_label");
    int label = 0;

    for (size_t i = 0; Real_Map_Vector.size(); i++) {
        // std::cout<<std::endl;
        if (Real_Map_Vector[i] == maximum_likelihood(pole_position)) {
            label = i;
            break;
        }
    }
    return label;
}

Coordinates add(Coordinates a, Coordinates b)

{

  return {a[0] + b[0], a[1] + b[1]};
}

Coordinates get_centroid (Coordinates_Vector_Colored map_in)
{
       
  int count = 0;
  Coordinates result = {0,0};
  for (auto i: map_in)
  {
    result = add (result, i.point);
    ++count;
  }
  return {result[0]/count, result[1]/count}; 

}

Coordinates_Vector_Colored get_coordinates_with_label (Coordinates_Vector_Colored map_in, int label)
{
  Coordinates_Vector_Colored result(0);
  for (auto i : map_in)
  {
    if (i.label == label)
      result.push_back(i);
  }
  return result;

}



void sort_based_on_label (Coordinates_Vector_Colored &detected_map_array)
{
    Coordinates_Vector_Colored::iterator i; 
    Coordinates_Colored swap_intermediate;
   
    for (i = detected_map_array.begin(); i != detected_map_array.end(); ++i) { 
  {
      
      
      for (Coordinates_Vector_Colored::iterator j=next(i,1); j != detected_map_array.end(); ++j)
      {
          if (j->label < i->label)
          {
          
             swap_intermediate = *i;
             *i = *j;
             *j=swap_intermediate;
          
          }
       }
  
   }
  }
}


void label_map (Coordinates_Vector_Colored &map_in)
{
  for (auto coordinate: map_in)
    coordinate.label= get_label(coordinate.point);
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
    sort_based_on_label (detected_map_array);
    
    //std::cout << "Map Size is:" << detected_map_array.size();
    
    
    
    //ROS_INFO_STREAM("Succeeded Copying the Point.");
  }

  //ROS_INFO_STREAM("Succeeded Cloning the Map.");


    // ROS_INFO_STREAM("Succeeded Cloning the Map.");
}

/*void reconstruct(
   pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr
cloud_out)

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

double dot(Coordinates_Vector in1, Coordinates_Vector in2) {
    return distance(in1[1], in1[0])[0] * distance(in2[1], in2[0])[0] +
           distance(in1[1], in1[0])[1] * distance(in2[1], in2[0])[1];
}




/*std::vector<double> localize (std::vector<std::vector<double> > ideal_map,
std::vector<std::vector<double> > detected_map   )

{
 // for ( )

}*/

// void get_field_components ( PointCloudPtr cloud_in  )

void tf_map_to_odom_boardcaster(double x, double y, double yaw) {
    static tf::TransformBroadcaster transform_broadcaster;
    // Quaternion from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

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

Coordinates_Vector get_centroidsVector(Coordinates_Vector_Colored map_in) {
    Coordinates vector_s = {0, 0};
    Coordinates vector_e = {0, 0};
    int count_s = 0;
    int count_e = 0;

    for (auto point_colored : map_in) {
        int label_modulus = point_colored.label % 8;
        if (label_modulus % 8 == 1 || label_modulus % 8 == 2 ||
            label_modulus % 8 == 3) {
            vector_s[0] = vector_s[1] + point_colored.point[0];
            count_s++;
        }

        if (label_modulus % 8 == 1 || label_modulus % 8 == 2 ||
            label_modulus % 8 == 3) {
            vector_e[0] = vector_s[0] + point_colored.point[0];
            count_e++;
        }
        vector_s[1] = vector_e[1] + point_colored.point[1];
        vector_e[1] = vector_e[1] + point_colored.point[1];
    }

    vector_s[0] = vector_s[1] / count_s;
    vector_e[0] = vector_e[1] / count_e;
    vector_s[1] = vector_s[1] / (count_s + count_e);
    vector_e[1] = vector_e[1] / ((count_e + count_e));

    return {vector_s, vector_e};
}

Coordinates_Vector get_goalsVector(Coordinates_Vector_Colored map_in) {
    Coordinates vector_s = {0, 0};
    Coordinates vector_e = {0, 0};

    int count = 0;

    for (auto point_colored : map_in) {
        if (point_colored.color == 'b')  // Start point of goals vector
        {
            ++count;
            vector_s[1] = vector_e[1] + point_colored.point[1];
            vector_s[0] = vector_e[0] + point_colored.point[0];
        }
    }
    vector_s[0] = vector_s[0] / count;
    vector_s[1] = vector_s[1] / count;
    count = 0;

    for (auto point_colored : map_in) {
        if (point_colored.color == 'y')  // Start point of goals vector
        {
            ++count;
            vector_e[1] = vector_e[1] + point_colored.point[1];
            vector_e[0] = vector_e[0] + point_colored.point[0];
        }
    }

    if (count != 0) {
        vector_e[0] = vector_e[0] / count;
        vector_e[1] = vector_e[1] / count;

        // std::cout<< vector_e[0]<<"::"<<vector_e[1]<<std::endl;

        return {vector_s, vector_e};
    } else
        return get_centroidsVector(map_in);
}

Coordinates get_translation(Coordinates_Vector_Colored map_in) {
     //ROS_INFO_STREAM ("Calculating the translation!");
    Coordinates_Vector goals_vector = get_goalsVector(map_in);
    return distance(Default_Goals_Vector[0], goals_vector[0]);
}

Coordinates_Vector_Colored translate(Coordinates_Vector_Colored map_in)
{
  Coordinates_Vector_Colored translated_map(map_in.size());
    
  Coordinates translation = get_translation(map_in);
  //ROS_INFO_STREAM ("Getting into loop!");

  for (size_t i = 0; i < map_in.size(); i++)
  {
    translated_map[i].point = add(map_in[i].point, translation);
    translated_map[i].color = map_in[i].color;
    //std::cout<<i<<std::endl;
  }
  
  //ROS_INFO_STREAM ("Now Returning!");
  return translated_map;
}

double angle (Coordinates_Vector in1, Coordinates_Vector in2 )

{
 double a= atan( distance( in1[1], in1[0] )[1] /distance( in1[1], in1[0] )[0])-
 atan( distance( in2[1], in2[0] )[1] /distance( in2[1], in2[0] )[0]);
 // double a= acos(dot (in1, in2) /
             // (absolute(in1) * absolute(in2)));
    //std::cout << "Abs1: " << absolute(in1) << "Abs2: " << absolute(in2) << std::endl << dot(in1,in2) << std::endl;

    
  return a;

}



double get_rotation(Coordinates_Vector_Colored map_in)
{


              /*std::cout << "Goals_Vector's size:" << absolute(get_goalsVector(map_in)) << std::endl;
              std::cout << "Default Goals_Vector's size:" << absolute(Default_Goals_Vector) << std::endl;
              std::cout << "Dot Product:" << dot(get_goalsVector(map_in), Default_Goals_Vector)<< std::endl;*/
  if (distance(get_goalsVector(map_in)[1],get_goalsVector(map_in)[0])[1] >0)
  return angle(get_goalsVector(map_in), Default_Goals_Vector);
  else return (angle(get_goalsVector(map_in), Default_Goals_Vector));
              
              
}



Coordinates_Vector_Colored rotate(Coordinates_Vector_Colored map_in) {
      
    double alpha = get_rotation(map_in);
    //  std::cout << alpha << std::endl;
    Coordinates_Vector_Colored map_result(map_in.size());
     //ROS_INFO_STREAM ("Getting the rotation!");
    for (size_t i = 0; i < map_in.size(); i++)

    {
        Coordinates point= {0,0};
         
        point[0] =
            map_in[i].point[0] * cos(alpha) - map_in[i].point[1] * sin(alpha);
        point[1] =
            map_in[i].point[1] * sin(alpha) + map_in[i].point[1] * cos(alpha);
        map_result[i].point=point;
        map_result[i].color = map_in[i].color;
    //std::cout <<i << std::endl;
    }

    return map_result;
}

Coordinates_Vector_Colored frame_rematch(Coordinates_Vector_Colored map_in) {
    
    Coordinates_Vector_Colored map_in_editted = translate(map_in);
    
    
    map_in_editted = rotate(map_in_editted);

    return map_in_editted;
}

void print_colored_coordinates(Coordinates_Vector_Colored vector_in) {
    // std::cout <<"Chechickinng: "<<  vector_in[0].point[0];
    for (auto point_colored : vector_in) {
        std::cout
            << " X: "
            << point_colored.point[0]   << "  Y: " << 
                                         point_colored.point[1] << " Color: "
                                         << std::isprint(point_colored.color) << std::endl;
    }
}

int main(int argc, char **argv) {
    // ROS_INFO_STREAM("STARTING");

    ros::init(argc, argv, "localization_node");
    ros::NodeHandle n;

    ros::Subscriber map_sub = n.subscribe("map_node/map", 1, pcl_Callback);
    ros::Subscriber dimension_sub =
        n.subscribe("field_width_node/width", 1, width_Callback);
    ros::Rate r(20);
    int clock = 0;

    Coordinates_Vector_Colored map_in(0);
    while (ros::ok()) {
        if (clock % 400 == 7) clock = 0;

        // ROS_INFO_STREAM("");

        if (got_map == true && got_width == true) {
            scale_Default_Goals();
            scale(Real_Map_Vector);

            copy_to_array(pcl_detected_objects, map_in);
            // if (clock==0) print_colored_coordinates(map_in);

            Coordinates translation = get_translation(map_in);
            double rotation = 0;
            rotation = get_rotation(map_in);  //*180/M_PI;

            // std::cout << std::endl <<std::endl <<std::endl;
        }

    Coordinates translation = get_translation(map_in);
    double rotation =0;
    rotation = get_rotation(map_in);
    //map_reconstructed =map_in;
    Coordinates translation_error;

    
    
     
    
int count=0;
//while (count<10){
count++;
        Coordinates_Vector_Colored  map_reconstructed =frame_rematch (map_in);
    
    label_map (map_reconstructed);
    //std:: cout<< get_goalsVector(map_in)[0][0] << " " << get_goalsVector(map_in)[0][1] ; 

    
     translation_error = distance(Real_Map_Vector[0],
                         get_centroid(get_coordinates_with_label( map_reconstructed ,0)));
    //print_colored_coordinates(get_coordinates_with_label( map_reconstructed ,0));
    
    //ROS_INFO_STREAM ("Angle Error Calculation");
    
    //double rotation_error =  angle({{0,0}, get_centroid(get_coordinates_with_label( map_reconstructed ,0))
      //                             }, {{0,0}, translation);

     

    //ROS_INFO_STREAM ("WTF!");
    translation = add (translation, translation_error);    
//}


     
    if(rotation==rotation && translation==translation){
        tf_map_to_odom_boardcaster(translation[0], translation[1], rotation);
       std::cout << "Translation X:"<< translation[0] <<" Translation Y:" << translation[1]<< " Rotation:" << rotation*180/M_PI << std::endl;
        std::cout << "Correction X:"<< translation_error[0] <<" Correction Y:" << translation_error[1] << std::endl;
    }
      flush_array(map_in);
      got_map=false;
      got_width=false;
      scale_down (Real_Map_Vector);

    //std::cout << std::endl <<std::endl <<std::endl;

    

  
          

        ros::spinOnce();
        clock++;
        r.sleep();
    }}

    

