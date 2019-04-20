#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

// Clustering
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointCloud<PointTypeRGB> PointCloudRGB;
typedef pcl::PointCloud<PointTypeRGB>::Ptr PointCloudPtrRGB;

class PlayNode {
   public:
    PlayNode(int argc, char **argv) {
        ros::init(argc, argv, "map_node");
        n = std::make_unique<ros::NodeHandle>();
        map_pub = n->advertise<PointCloudRGB>("map_node/map", 1);

        map_cloud = PointCloudPtr(new PointCloud);
        temp_cloud = PointCloudPtr(new PointCloud);

        // tfBuffer = new tf2_ros::Buffer(ros::Duration(100));
        // tf_listener = new tf2_ros::TransformListener(*tfBuffer);

        ROS_INFO("Waiting for pointcloud_node/detected_objects");
        ros::topic::waitForMessage<PointCloud>(
            "pointcloud_node/detected_objects");

        detected_objects =
            n->subscribe("pointcloud_node/detected_objects", 1,
                         &PlayNode::detected_objects_callback, this);
    }

    void detected_objects_callback(const PointCloud::ConstPtr &msg) {
        // ROS_INFO("Got new detected objects");

        detected_objects_msg = *msg;
        got_detected_objects = true;
    }

    void pub_pointcloud(PointCloud &cloud) {
        PointCloudPtr msg(new PointCloud);
        msg->header.frame_id = "robot1/odom";

        msg->height = cloud.height;
        msg->width = cloud.width;

        msg->points = cloud.points;

        map_pub.publish(msg);
    }

    // Get points with an alpha value smaller than given alpha

    void alpha_filter(PointCloudPtr &cloud, uint8_t max_alpha) {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<PointType> extract;
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

    void set_alpha(PointCloudPtr &cloud, int alpha) {
        for (auto pt = cloud->begin(); pt != cloud->end(); ++pt) {
            uint8_t alpha = alpha;
            uint32_t rgb = (pt->rgba & 0xffffff);
            pt->rgba = ((alpha << 24) | rgb);
        }
    }

    void radius_outlier_removal(PointCloudPtr &cloud, double radius = 0.05,
                                int min_neighbors = 5) {
        pcl::RadiusOutlierRemoval<PointType> outrem;
        // build the filter
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(radius);
        outrem.setMinNeighborsInRadius(min_neighbors);
        // apply filter
        outrem.filter(*cloud);
    }

    PointType is_buck_or_pole(PointCloudPtr &cloud) {
        // This method takes as input a pointcloud of a suspected pole or puck.
        // Returns a point containing information (x,y,z,r,g,b) of the object if
        // it fulfills the requirements.

        int color_threshold = 3;  // min number of points in main color
        PointType result_point = PointType();
        result_point.x = 0;
        result_point.y = 0;
        result_point.z = 0.2;

        PointType min_point;
        PointType max_point;
        pcl::getMinMax3D(*cloud, min_point, max_point);

        // Calculate metrics of the pointcloud
        double average_x = 0.5 * (max_point.x + min_point.x);
        double average_y = 0.5 * (max_point.y + min_point.y);
        double diagonal_xy = sqrt(pow(max_point.y - min_point.y, 2) +
                                  pow(max_point.x - min_point.x, 2));

        // Check if size of the pointcloud is within the limits
        if (diagonal_xy >= 0.18) {
            std::cout << "too big, diagonal: " << diagonal_xy << std::endl;
            result_point.rgba = 0;
            result_point.x = 0;
            result_point.y = 0;

            // size does not match -> return a black point in origo
            return result_point;
        }

        // Calculate values for colors
        uint32_t yellow = 0xffff00;
        uint32_t blue = 0x0000ff;
        uint32_t green = 0x00ff00;

        // Calculate color frequencies
        int blue_points = 0;
        int green_points = 0;
        int yellow_points = 0;
        for (auto pt = cloud->begin(); pt < cloud->end(); pt++) {
            
            uint32_t pt_rgb = *reinterpret_cast<int*>(&pt->rgba) & 0xffffff; 
            //std::cout << std::hex << pt_rgb << std::endl;
            if (pt_rgb == yellow) {
                yellow_points++;
            } else if (pt_rgb == green) {
                green_points++;
            } else if (pt_rgb == blue) {
                blue_points++;
            }
        }

        std::cout <<"b"<< blue_points << " g" << green_points << " y"
                  << yellow_points << std::endl;

        // Determine to which catecory the object belongs to
        float choosen_color = 0;
        if (blue_points > green_points && blue_points > yellow_points &&
            blue_points > color_threshold) {
            // Blue
            choosen_color = blue;

        } else if (green_points > blue_points && green_points > yellow_points &&
                   green_points > color_threshold) {
            // Green
            choosen_color = green;

        } else if (yellow_points > green_points &&
                   yellow_points > blue_points &&
                   yellow_points > color_threshold) {
            // Yellow
            choosen_color = yellow;
        } else {
            // Put unknown objects to origo
            average_x = 0;
            average_y = 0;
            std::cout << "unkown cloud" << std::endl;
        }

        // Return point, color represents point type: green -> buck, blue or
        // yellow -> pole.
        result_point.rgba = choosen_color;
        result_point.x = average_x;
        result_point.y = average_y;

        return result_point;
    }

    PointCloudPtr combine_measurements(PointCloudPtr &cloud) {
        // This algorithm uses Euclidean Cluster Extraction to segment the cloud
        // into regions. After segmentation objects are filtered by size (min
        // and max xy coordinates). Finally, the objects are detectect by the
        // highlighted colors.

        PointCloudPtr result(new PointCloud);

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<PointType>::Ptr tree(
            new pcl::search::KdTree<PointType>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance(0.05);  // 5cm
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(500);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        int j = 1;
        for (std::vector<pcl::PointIndices>::const_iterator it =
                 cluster_indices.begin();
             it != cluster_indices.end(); ++it) {
            pcl::PointCloud<PointType>::Ptr cloud_cluster(
                new pcl::PointCloud<PointType>);

            for (std::vector<int>::const_iterator pit = it->indices.begin();
                 pit != it->indices.end(); ++pit) {
                cloud_cluster->points.push_back(cloud->points[*pit]);
            }

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // std::cout << "PointCloud representing the Cluster: "
            //          << cloud_cluster->points.size() << " data points."
            //          << std::endl;
            PointType point = is_buck_or_pole(cloud_cluster);
            std::cout << point.x << " " << point.y << " j" << j << std::endl;
            if (point.rgba != 0) {
                // Add only succesfull detections
                result->points.push_back(point);
            }

            j++;
        }

        // Set header information
        result->is_dense = false;
        result->width = 1;
        result->height = result->points.size();

        return result;
    }

    void to_environment(PointCloudPtr &cloud) {
        // This function takes the sum of detectect objets and based on that
        // finds an estimate of the real buck and pole locations in the
        // environment.

        // Filter outliers
        radius_outlier_removal(cloud, 0.03, 5);

        // Cluster cloud
        cloud = combine_measurements(cloud);
    }

    void process_messages() {
        // Add new detections to map_cloud
        *map_cloud += detected_objects_msg;

        // Update current map_cloud, e.g remove old stuff
        if (alpha_counter >= INCREASE_ALPHA_AFTER_N_MESSAGES) {
            // Increase alpha by one
            for (auto pt = map_cloud->begin(); pt != map_cloud->end(); ++pt) {
                uint8_t alpha = ((pt->rgba >> 24) & 0xff) + 1;
                uint32_t rgb = (pt->rgba & 0xffffff);

                pt->rgba = ((alpha << 24) | rgb);
            }

            // Remove points with value 255
            alpha_filter(map_cloud, 255);
            alpha_counter = 0;
        } else {
            alpha_counter++;
        }

        // Filter pointcloud to get an estimate of the environment
        // TODO

        // Publish map
        *temp_cloud = *map_cloud;  // copy
        set_alpha(temp_cloud, 255);
        //to_environment(temp_cloud);

        pub_pointcloud(*temp_cloud);  // sum of all detections

        got_detected_objects = false;  // reset
    }

    bool got_messages() const { return got_detected_objects; }

   private:
    bool got_detected_objects;
    const int INCREASE_ALPHA_AFTER_N_MESSAGES = 2;
    int alpha_counter = 0;

    std::unique_ptr<ros::NodeHandle> n;
    ros::Subscriber detected_objects;
    ros::Publisher map_pub;

    // tf2_ros::Buffer *tfBuffer;
    // tf2_ros::TransformListener *tf_listener;

    PointCloud detected_objects_msg;
    PointCloudPtr map_cloud;
    PointCloudPtr temp_cloud;
};

int main(int argc, char **argv) {
    PlayNode playNode(argc, argv);

    // 20 Hz loop
    ros::Rate r(20);
    while (ros::ok()) {
        if (playNode.got_messages()) {
            playNode.process_messages();
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
