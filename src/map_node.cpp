#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/centroid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

// Clustering
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pointcloud_helpers.hpp"

class PlayNode {
   public:
    PlayNode(int argc, char **argv) {
        ros::init(argc, argv, "map_node");
        n = std::make_unique<ros::NodeHandle>();
        map_pub = n->advertise<PointCloud>("map_node/map", 1);
        map_raw_pub = n->advertise<PointCloud>("map_node/map_raw", 1);

        // Subscribers for feedback
        tfBuffer = new tf2_ros::Buffer(ros::Duration(100));
        tf_listener = new tf2_ros::TransformListener(*tfBuffer);
        field_width_sub = n->subscribe("field_width_node/width", 1,
                                       &PlayNode::field_width_callback, this);

        map_cloud = PointCloudPtrRGBA(new PointCloudRGBA);
        temp_cloud = PointCloudPtrRGBA(new PointCloudRGBA);

        ROS_INFO("Waiting for pointcloud_node/detected_objects");
        ros::topic::waitForMessage<PointCloudRGBA>(
            "pointcloud_node/detected_objects");

        detected_objects =
            n->subscribe("pointcloud_node/detected_objects", 1,
                         &PlayNode::detected_objects_callback, this);
    }

    void detected_objects_callback(const PointCloudRGBA::ConstPtr &msg) {
        // ROS_INFO("Got new detected objects");

        detected_objects_msg = *msg;
        got_detected_objects = true;
    }

    void field_width_callback(const geometry_msgs::Vector3::ConstPtr &msg) {
        geometry_msgs::Vector3 width = *msg;
        field_width = width.x;
        got_field_width = true;
    }

    void pub_pointcloud(PointCloudRGBA &cloud, ros::Publisher &pub) {
        PointCloudPtrRGBA msg(new PointCloudRGBA);
        msg->header.frame_id = "robot1/odom";

        msg->height = cloud.height;
        msg->width = cloud.width;

        msg->points = cloud.points;

        pub.publish(msg);
    }

    void color_filter(PointCloudPtrRGBA &cloud_in, PointCloudPtrRGBA &cloud_out,
                      int r, int g, int b) {
        // Filters pointcloud by a specific color
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<PointTypeRGBA> extract;
        for (int i = 0; i < (*cloud_in).size(); i++) {
            uint32_t argb = cloud_in->points[i].rgba;
            uint8_t alpha = (argb >> 24) & 0xff;
            uint8_t bp = (argb >> 0) & 0xff;
            uint8_t gp = (argb >> 8) & 0xff;
            uint8_t rp = (argb >> 16) & 0xff;

            if (r == rp && g == gp && b == bp) {
                // Use these points
                inliers->indices.push_back(i);
            }
        }
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.filter(*cloud_out);
    }

    void alpha_filter(PointCloudPtrRGBA &cloud, uint8_t max_alpha) {
        // Get points with an alpha value smaller than given alpha = removes too
        // large alpha values.

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<PointTypeRGBA> extract;

        for (int i = 0; i < (*cloud).size(); i++) {
            uint32_t argb = cloud->points[i].rgba;
            uint8_t alpha = (argb >> 24) & 0xff;

            if (alpha >= max_alpha) {
                // Remove these points
                inliers->indices.push_back(i);
            }
        }
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud);
    }

    void set_alpha(PointCloudPtrRGBA &cloud, int alpha) {
        // Set alpha value to a specific value
        for (auto pt = cloud->begin(); pt != cloud->end(); ++pt) {
            uint8_t alpha2 = alpha;
            uint32_t rgb = (pt->rgba & 0xffffff);
            pt->rgba = ((alpha2 << 24) | rgb);
        }
    }

    void radius_outlier_removal(PointCloudPtrRGBA &cloud, double radius = 0.05,
                                int min_neighbors = 5) {
        pcl::RadiusOutlierRemoval<PointTypeRGBA> outrem;
        // build the filter
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(radius);
        outrem.setMinNeighborsInRadius(min_neighbors);
        // apply filter
        outrem.filter(*cloud);
    }

    PointTypeRGBA is_buck_or_pole_or_corner(PointCloudPtrRGBA &cloud) {
        // This method takes as input a pointcloud of a suspected pole or puck
        // or corner locations. Returns a point containing information
        // (x,y,z,r,g,b) of the object if it fulfills the requirements. If
        // requirement are not fulfilled, a black point will be
        // returned.

        PointTypeRGBA result_point = PointTypeRGBA();
        result_point.x = 0;
        result_point.y = 0;
        result_point.z = 0.2;

        // Calculate metrics of the pointcloud
        PointTypeRGBA min_point;
        PointTypeRGBA max_point;
        PointTypeRGBA centroid;
        pcl::getMinMax3D(*cloud, min_point, max_point);
        pcl::computeCentroid(*cloud, centroid);

        double average_x = centroid.x;
        double average_y = centroid.y;
        /*       double diagonal_xy = sqrt(pow(max_point.y - min_point.y, 2) +
                                         pow(max_point.x - min_point.x, 2));

               // Check if size of the pointcloud is within the limits
               if (diagonal_xy >= 0.21) {
                   // std::cout << "too big, diagonal: " << diagonal_xy <<
           std::endl; result_point.rgb = 0; result_point.x = 0; result_point.y =
           0;

                   // size does not match -> return a black point in origo
                   return result_point;
               }*/

        // Return point, color represents point type: green -> buck,
        // blue or yellow -> pole.
        result_point.rgb = get_main_rgb_value(cloud);
        result_point.x = average_x;
        result_point.y = average_y;

        return result_point;
    }

    PointCloudPtrRGBA combine_measurements(PointCloudPtrRGBA &cloud,
                                           double cluster_tolerance = 0.05,
                                           int min_cluster_size = 3,
                                           int max_cluster_size = 1000) {
        // This algorithm uses Euclidean Cluster Extraction to segment
        // the cloud into regions. After segmentation objects are
        // classified with the is_buck_or_pole_or_corner function.

        PointCloudPtrRGBA result(new PointCloudRGBA);

        if (cloud->points.size() == 0) {
            return result;
        }

        // Creating the KdTree object for the search method of the
        // extraction
        pcl::search::KdTree<PointTypeRGBA>::Ptr tree(
            new pcl::search::KdTree<PointTypeRGBA>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointTypeRGBA> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it =
                 cluster_indices.begin();
             it != cluster_indices.end(); ++it) {
            pcl::PointCloud<PointTypeRGBA>::Ptr cloud_cluster(
                new pcl::PointCloud<PointTypeRGBA>);

            for (std::vector<int>::const_iterator pit = it->indices.begin();
                 pit != it->indices.end(); ++pit) {
                cloud_cluster->points.push_back(cloud->points[*pit]);
            }

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            PointTypeRGBA point = is_buck_or_pole_or_corner(cloud_cluster);
            if (*reinterpret_cast<int *>(&point.rgb) != 0) {
                // Add only succesfull detections (== not black points)
                result->points.push_back(point);
            }
        }

        // Set header information
        result->is_dense = false;
        result->width = 1;
        result->height = result->points.size();

        return result;
    }

    float get_main_rgb_value(PointCloudPtrRGBA &cloud,
                             int color_threshold = 1) {
        // This function retruns the most common color in the point cloud. Only
        // 5 different highlighted colors are counted.

        // Calculate values for colors
        float c_yellow = to_pcl_rgb(255, 255, 0);
        float c_blue = to_pcl_rgb(0, 0, 255);
        float c_green = to_pcl_rgb(0, 255, 0);
        float c_black = to_pcl_rgb(0, 0, 0);
        float c_cyan = to_pcl_rgb(0, 255, 255);
        float c_orange = to_pcl_rgb(255, 140, 0);

        uint32_t yellow = 0xffff00;
        uint32_t blue = 0x0000ff;
        uint32_t green = 0x00ff00;
        uint32_t cyan = 0x00ffff;
        uint32_t orange = 0xFF8C00;

        // Calculate color frequencies
        int blue_points = 0;
        int green_points = 0;
        int yellow_points = 0;
        int cyan_points = 0;    // blue goals
        int orange_points = 0;  // yellow goals

        for (auto pt = cloud->begin(); pt < cloud->end(); pt++) {
            uint32_t pt_rgb = *reinterpret_cast<int *>(&pt->rgba) & 0xffffff;

            if (pt_rgb == yellow) {
                yellow_points++;
            } else if (pt_rgb == green) {
                green_points++;
            } else if (pt_rgb == blue) {
                blue_points++;
            } else if (pt_rgb == cyan) {
                cyan_points++;
            } else if (pt_rgb == orange) {
                orange_points++;
            }
        }

        // A vector is used for finding the max value
        std::vector<int> colors{blue_points, green_points, yellow_points,
                                cyan_points, orange_points};

        // Determine to which catecory the object belongs to
        int max_frequency_index =
            std::max_element(colors.begin(), colors.end()) - colors.begin();
        int num_of_max_color = *std::max_element(colors.begin(), colors.end());

        float choosen_color = c_black;
        if (num_of_max_color >= color_threshold) {
            if (max_frequency_index == 0) {
                // Blue
                choosen_color = c_blue;
            } else if (max_frequency_index == 1) {
                // Green
                choosen_color = c_green;
            } else if (max_frequency_index == 2) {
                // Yellow
                choosen_color = c_yellow;
            } else if (max_frequency_index == 3) {
                // Blue goal
                choosen_color = c_cyan;
            } else if (max_frequency_index == 4) {
                // Yellow goal
                choosen_color = c_orange;
            } else {
                // Unknown objects
            }
        }
        return choosen_color;
    }

    uint8_t increace_single_point_alpha_by_one(PointCloudPtrRGBA &cloud,
                                               int index) {
        uint32_t argb = cloud->points[index].rgba;
        uint8_t alpha = (argb >> 24) + 1;
        argb = (alpha << 24) | (argb & 0xffffff);
        cloud->points[index].rgba = argb;
        return alpha;
    }

    inline void remove_and_replace_cluster(
        std::vector<pcl::PointIndices>::const_iterator &it,
        PointCloudPtrRGBA &cloud_cluster, PointCloudPtrRGBA &new_points,
        pcl::PointIndices::Ptr &points_to_be_removed, double average_x,
        double average_y) {
        PointTypeRGBA new_point;
        new_point.x = average_x;
        new_point.y = average_y;
        new_point.z = 0;
        new_point.rgba = 0;
        new_point.rgb = get_main_rgb_value(cloud_cluster, 1);
        new_points->points.push_back(new_point);

        for (std::vector<int>::const_iterator pit = it->indices.begin();
             pit != it->indices.end(); ++pit) {
            points_to_be_removed->indices.push_back(*pit);
        }
    }

    void update_map(PointCloudPtrRGBA &cloud, double cluster_tolerance = 0.05,
                    int min_cluster_size = 3, int max_cluster_size = 1000,
                    double max_cluster_diagonal = 0.2,
                    int max_cluster_size_before_changing_to_single_point = 20) {
        // This function updates the current map_cloud it uses Euclidean Cluster
        // Extraction to segment the cloud into cluster. Too big clusters are
        // removed and replaced with a single point. Clusters with only one
        // point are time stamped (alpha value) and on each call their time
        // stamp is increased by one (eventually they will be removed if no
        // other detections nearby occur).

        // Parameters
        const uint8_t MAX_AGE_SINGLE_POINT =
            160;  // 255;   // Larger -> longer life
        const uint8_t MAX_AGE_CLUSTER_POINT =
            90;  // 255;  // Larger -> longer life

        PointCloudPtrRGBA new_points(new PointCloudRGBA);
        pcl::PointIndices::Ptr points_to_be_removed(new pcl::PointIndices());

        if (cloud->points.size() == 0) {
            return;
        }

        // Creating the KdTree object for the search method of the
        // extraction
        pcl::search::KdTree<PointTypeRGBA>::Ptr tree(
            new pcl::search::KdTree<PointTypeRGBA>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointTypeRGBA> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it =
                 cluster_indices.begin();
             it != cluster_indices.end(); ++it) {
            pcl::PointCloud<PointTypeRGBA>::Ptr cloud_cluster(
                new pcl::PointCloud<PointTypeRGBA>);

            for (std::vector<int>::const_iterator pit = it->indices.begin();
                 pit != it->indices.end(); ++pit) {
                cloud_cluster->points.push_back(cloud->points[*pit]);
            }

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            int cluster_size = cloud_cluster->points.size();
            PointTypeRGBA min_point;
            PointTypeRGBA max_point;
            PointTypeRGBA centroid;
            pcl::getMinMax3D(*cloud_cluster, min_point, max_point);
            pcl::computeCentroid(*cloud_cluster, centroid);
            double average_x = centroid.x;
            double average_y = centroid.y;
            double diagonal_xy = sqrt(pow(max_point.y - min_point.y, 2) +
                                      pow(max_point.x - min_point.x, 2));

            if (cluster_size == 1) {
                // Increase timestamp of this single point cluster by one and if
                // max value is reached the point is removed.
                pcl::PointIndices ind = *it;
                int index = ind.indices[0];
                int new_alpha_value =
                    increace_single_point_alpha_by_one(cloud, index);

                if (new_alpha_value >= MAX_AGE_SINGLE_POINT) {
                    // Remove single points with value 255
                    points_to_be_removed->indices.push_back(index);
                    // std::cout << "single point removal, too old" <<
                    // std::endl;
                }

            } else if (diagonal_xy > max_cluster_diagonal ||
                       cluster_size >
                           max_cluster_size_before_changing_to_single_point) {
                // A too big cluster -> remove these cluster points and replace
                // it with a single point.
                remove_and_replace_cluster(it, cloud_cluster, new_points,
                                           points_to_be_removed, average_x,
                                           average_y);
            } else {
                // A good cluster, increase the time stamp of one puck, if all
                // puck have a time stamp of 255 -> replace cloud with a single
                // point. This is needed to get rid of old detections that could
                // exist otherwise forever.

                for (std::vector<int>::const_iterator pit = it->indices.begin();
                     pit != it->indices.end(); ++pit) {
                    if ((cloud->points[*pit].rgba >> 24) <
                        MAX_AGE_CLUSTER_POINT) {
                        // Increase alpha value of a single point and exit the
                        // loop.
                        increace_single_point_alpha_by_one(cloud, *pit);
                        break;

                    } else if (pit + 1 == it->indices.end()) {
                        // Last point -> all points have an alpha value of 255
                        // -> remove cluster and replace with a single point.
                        // std::cout << "remove cluster, too old" << std::endl;
                        remove_and_replace_cluster(
                            it, cloud_cluster, new_points, points_to_be_removed,
                            average_x, average_y);
                    }
                }
            }
        }

        // Filter points to be removed
        pcl::ExtractIndices<PointTypeRGBA> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(points_to_be_removed);
        extract.setNegative(true);
        extract.filter(*cloud);

        // Add new points
        *cloud += *new_points;

        // Set header information
        cloud->is_dense = false;
        cloud->width = 1;
        cloud->height = cloud->points.size();
    }

    void remove_outlier_based_on_feedback(
        PointCloudPtrRGBA &puck_and_pole_cloud, PointCloudPtrRGBA &goal_cloud) {
        // This function removes wrong detections from map cloud, based on known
        // information of field and robot location.
        const double MAX_DISTANCE_FROM_IDEAL = 0.2;  // m

        tf::Transform transform_odom_to_map;
        bool succesful_robot_pos_tf = get_transform(
            transform_odom_to_map, tfBuffer, "map", "robot1/odom");

        bool transformation_set = false;
        n->param("transformation_map_to_odom_set", transformation_set, false);
        // std::cout << "got to map transform: " << transformation_set
        //          << std::endl;

        if (tf_delay_counter <= 10 && transformation_set) {
            // This delay prevents the feedback from being done with old tf
            // data.
            tf_delay_counter++;
        }

        if (got_field_width == true && succesful_robot_pos_tf == true &&
            transformation_set == true && tf_delay_counter >= 10) {
            // Got required data for performing feedback operation, e.g removing
            // wrong detections from map_cloud.

            // Create copy of clouds
            PointCloudPtrRGBA temp_goal_cloud(new PointCloudRGBA(*goal_cloud));
            PointCloudPtrRGBA temp_puck_and_pole_cloud(
                                new PointCloudRGBA(*puck_and_pole_cloud));

            PointCloudPtrRGBA map_goal_cloud(new PointCloudRGBA);
            PointCloudPtrRGBA map_puck_and_pole_cloud(new PointCloudRGBA);

            // Transform clouds to map frame
            pcl_ros::transformPointCloud(*temp_goal_cloud, *map_goal_cloud,
                                         transform_odom_to_map);
            pcl_ros::transformPointCloud(*temp_puck_and_pole_cloud,
                                         *map_puck_and_pole_cloud,
                                         transform_odom_to_map);

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::ExtractIndices<PointTypeRGBA> extract;

            for (int j = 0; j < map_puck_and_pole_cloud->points.size(); j++) {
                // Get color of point
                uint32_t rgb = *reinterpret_cast<int *>(
                    &map_puck_and_pole_cloud->points[j].rgb);
                uint8_t b = (rgb >> 0) & 0xff;
                uint8_t g = (rgb >> 8) & 0xff;
                uint8_t r = (rgb >> 16) & 0xff;

                double x = map_puck_and_pole_cloud->points[j].x;
                double y = map_puck_and_pole_cloud->points[j].y;

                if (r == 0 && g == 255 && b == 0) {  // green -> pole
                    if (!(((x > -MAX_DISTANCE_FROM_IDEAL &&
                            x < MAX_DISTANCE_FROM_IDEAL) ||
                           (x > field_width - MAX_DISTANCE_FROM_IDEAL &&
                            x < field_width + MAX_DISTANCE_FROM_IDEAL)) &&
                          y > -MAX_DISTANCE_FROM_IDEAL &&
                          y < 5.0 / 3.0 * field_width +
                                  MAX_DISTANCE_FROM_IDEAL)) {
                        // Pole outside of correct area -> remove
                        // Remove these points
                        std::cout << "Pole removed x: " << x << " y: " << y
                                  << std::endl;
                        inliers->indices.push_back(j);
                    }
                } else {  // yellow or blue -> buck
                    if (!(((x > -MAX_DISTANCE_FROM_IDEAL &&
                            x < field_width + MAX_DISTANCE_FROM_IDEAL)) &&
                          y > -MAX_DISTANCE_FROM_IDEAL &&
                          y < 5.0 / 3.0 * field_width +
                                  MAX_DISTANCE_FROM_IDEAL)) {
                        // Puck outside of field -> remove
                        // Remove these points
                        inliers->indices.push_back(j);

                        std::cout << "Puck removed x: " << x << " y: " << y
                                  << std::endl;
                    } 
                }
            }
            extract.setInputCloud(puck_and_pole_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*puck_and_pole_cloud);

            // Remove goals that are outside of the field.
            inliers = pcl::PointIndices::Ptr(new pcl::PointIndices());
            for (int j = 0; j < map_goal_cloud->points.size(); j++) {
                double x = map_goal_cloud->points[j].x;
                double y = map_goal_cloud->points[j].y;

                if (!(((x > -MAX_DISTANCE_FROM_IDEAL &&
                        x < field_width + MAX_DISTANCE_FROM_IDEAL)) &&
                      y > -MAX_DISTANCE_FROM_IDEAL &&
                      y < 5.0 / 3.0 * field_width + MAX_DISTANCE_FROM_IDEAL)) {
                    // Goal point outside of field -> remove
                    // Remove these points
                    inliers->indices.push_back(j);
                    std::cout << "Goal point removed x: " << x << " y: " << y
                              << std::endl;
                }
            }
            extract.setInputCloud(goal_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*goal_cloud);
        }
    }

    void process_messages() {
        // -------------------------------------------------------
        // Update and add new data to map_cloud
        // -------------------------------------------------------

        // Add new detections to map_cloud
        *temp_cloud = detected_objects_msg;
        set_alpha(temp_cloud, 0);
        *map_cloud += *temp_cloud;

        // Divide map_cloud by object type into two clouds
        PointCloudPtrRGBA goal_cloud = PointCloudPtrRGBA(new PointCloudRGBA);
        PointCloudPtrRGBA goal_cloud_orange =
            PointCloudPtrRGBA(new PointCloudRGBA);
        PointCloudPtrRGBA goal_cloud_cyan =
            PointCloudPtrRGBA(new PointCloudRGBA);

        PointCloudPtrRGBA puck_and_pole_cloud =
            PointCloudPtrRGBA(new PointCloudRGBA);
        PointCloudPtrRGBA temp = PointCloudPtrRGBA(new PointCloudRGBA);

        color_filter(map_cloud, temp, 0, 255, 255);  // Cyan == Blue goal
        *goal_cloud = *temp;
        color_filter(map_cloud, temp, 255, 140, 0);  // Orange == Yelllow
        *goal_cloud += *temp;

        color_filter(map_cloud, temp, 0, 0, 255);  // Blue
        *puck_and_pole_cloud = *temp;
        color_filter(map_cloud, temp, 0, 255, 0);  // Green
        *puck_and_pole_cloud += *temp;
        color_filter(map_cloud, temp, 255, 255, 0);  // Yelllow
        *puck_and_pole_cloud += *temp;
        // color_filter(map_cloud, temp, 255, 0,
        //             255);  // Magenta == unknow clolored puck or pole
        //*puck_and_pole_cloud += *temp;

        // Remove too big or too old (time stamp == alpha value) clusters,
        // increase time stamp of single points and cluster.
        update_map(puck_and_pole_cloud, 0.15, 1, 10000, 0.2, 10);
        update_map(goal_cloud, 0.15, 1, 10000, 0.2, 10);

        // Feed back, remove clearly wrong detections: poles and pucks outside
        // of the field or inside of the field and pucks out
        if(poles_in_map_cloud > MIN_NUMBER_OF_POLES_IN_MAP){
            //remove_outlier_based_on_feedback(puck_and_pole_cloud, goal_cloud);
        }

        // update_map(goal_cloud, 1.2, 1, 20000, 1.2, 100);
        // Simple goal detection*********************
        // color_filter(map_cloud, temp, 0, 255, 255);  // Cyan == Blue goal
        // voxel_grid_filter_m(temp, goal_cloud_cyan, 0.05, 1);

        // color_filter(map_cloud, temp, 255, 140, 0);  // Orange == Yelllow
        // goal voxel_grid_filter_m(temp, goal_cloud_orange, 0.05, 1);
        // ******************************************

        // -------------------------------------------------------
        // Create estimate of the environment
        // -------------------------------------------------------
        // Pucks and Poles:
        *temp_cloud = *combine_measurements(puck_and_pole_cloud, 0.15, 1,
                                            1000);  // 0.15->0.2
        
        // Calculate number of poles in cloud
        color_filter(temp_cloud, temp, 0, 255, 0);  // Green
        poles_in_map_cloud = temp->points.size();

        // Goals:
        *temp_cloud +=
            *combine_measurements(goal_cloud, 0.15, 1, 1000);  // 0.15->0.2

        // Simple
        // temp_cloud->points.push_back(
        //    get_centroid_of_color(goal_cloud, 255, 140, 0));
        // temp_cloud->points.push_back(
        //    get_centroid_of_color(goal_cloud, 0, 255, 255));
        // temp_cloud->is_dense = false;
        // temp_cloud->width = 1;
        // temp_cloud->height = temp_cloud->points.size();

        // -------------------------------------------------------
        // Publish and prepare for next iteration
        // -------------------------------------------------------

        // Save clouds for next round
        *map_cloud = *puck_and_pole_cloud;
        *map_cloud += *goal_cloud;
        //*map_cloud += *goal_cloud_cyan;   
          //*map_cloud += *goal_cloud_orange;

        // Publish final map
        set_alpha(temp_cloud, 0xff);
        pub_pointcloud(*temp_cloud,
                       map_pub);  // final map of the environment

        //save_cloud_to_file(temp_cloud,
        //                   "/home/ros/team2/map_cloud.pcd");

        // Publish raw map for debugging
        *temp_cloud = *map_cloud;  // copy
        set_alpha(temp_cloud, 0xff);
        pub_pointcloud(*temp_cloud,
                       map_raw_pub);  // raw map for debugging

        got_detected_objects = false;  // reset
    }

    bool got_messages() const { return got_detected_objects; }

   private:
   const double MIN_NUMBER_OF_POLES_IN_MAP = 12; // TODOOO

    bool got_detected_objects = false;
    bool got_field_width = false;

    std::unique_ptr<ros::NodeHandle> n;
    ros::Subscriber detected_objects;
    ros::Subscriber field_width_sub;

    ros::Publisher map_pub;
    ros::Publisher map_raw_pub;

    tf2_ros::Buffer *tfBuffer;
    tf2_ros::TransformListener *tf_listener;
    int tf_delay_counter = 0;  // iterations before using feedback
    int poles_in_map_cloud = 0;

    double field_width = 3;
    PointCloudRGBA detected_objects_msg;
    PointCloudPtrRGBA map_cloud;   // contains points of individual detections
    PointCloudPtrRGBA temp_cloud;  // temorary
};

int main(int argc, char **argv) {
    PlayNode playNode(argc, argv);

    // 30 Hz loop
    ros::Rate r(30);
    while (ros::ok()) {
        if (playNode.got_messages()) {
            playNode.process_messages();
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
