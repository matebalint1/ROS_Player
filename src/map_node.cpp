#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

class PlayNode {
   public:
    PlayNode(int argc, char **argv) {
        ros::init(argc, argv, "map_node");
        n = std::make_unique<ros::NodeHandle>();
        map_cloud = PointCloudPtr(new PointCloud);

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
        ROS_INFO("Got new detected objects");

        detected_objects_msg = *msg;
        got_detected_objects = true;
    }

    void pub_pointcloud(PointCloud &cloud) {
        PointCloud::Ptr msg(new PointCloud);
        msg->header.frame_id = "robot1/base_link";

        msg->height = cloud.height;
        msg->width = cloud.width;

        msg->points = cloud.points;

        kinect_pub.publish(msg);
    }

    void process_messages() {
        // Add new detections to map
        *map_cloud += detected_objects_msg;

        // increase alpha by one
        for (auto pt = map_cloud->begin(); pt != map_cloud->end(); ++pt) {
            uint8_t alpha = ((pt->rgba >> 24) & 0xff) + 1;
            uint32_t rgb = (pt->rgba && 0xffffff);

            pt->rgba = ((alpha << 24) | rgb);
            //std::cout << "alpha: " << int(alpha) << std::endl;
        }

        got_detected_objects = false;  // reset
    }

    bool got_messages() const { return got_detected_objects; }

   private:
    bool got_detected_objects;

    std::unique_ptr<ros::NodeHandle> n;
    ros::Publisher kinect_pub;
    ros::Subscriber detected_objects;

    // tf2_ros::Buffer *tfBuffer;
    // tf2_ros::TransformListener *tf_listener;

    PointCloud detected_objects_msg;
    PointCloudPtr map_cloud;
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
