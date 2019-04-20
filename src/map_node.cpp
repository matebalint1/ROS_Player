#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

typedef pcl::PointXYZRGBA PointTypeRGB;
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

    void detected_objects_callback(const PointCloudRGB::ConstPtr &msg) {
        // ROS_INFO("Got new detected objects");

        detected_objects_msg = *msg;
        got_detected_objects = true;
    }

    void pub_pointcloud(PointCloudRGB &cloud) {
        PointCloudPtrRGB msg(new PointCloudRGB);
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

    /*
        PointCloudPtrRGB to_rgb_cloud(PointCloudPtr &cloud_rgba) {
            PointCloudPtrRGB cloud_rgb = PointCloudPtrRGB(new PointCloudRGB());
            copyPointCloud(*cloud_rgba, *cloud_rgb); // convert rgba -> rgb
            return cloud_rgb;
        }
     */

    void process_messages() {
        // Add new detections to map
        copyPointCloud(detected_objects_msg,
                *temp_cloud); // convert rgb -> rgba
        *map_cloud += *temp_cloud;


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
        *temp_cloud = *map_cloud; // copy
        set_alpha(temp_cloud, 255);
        pub_pointcloud(*temp_cloud);

        got_detected_objects = false; // reset
    }

    bool got_messages() const {
        return got_detected_objects;
    }

private:
    bool got_detected_objects;
    const int INCREASE_ALPHA_AFTER_N_MESSAGES = 2;
    int alpha_counter = 0;

    std::unique_ptr<ros::NodeHandle> n;
    ros::Subscriber detected_objects;
    ros::Publisher map_pub;

    // tf2_ros::Buffer *tfBuffer;
    // tf2_ros::TransformListener *tf_listener;

    PointCloudRGB detected_objects_msg;
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
