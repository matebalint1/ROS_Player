#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPrt;


class PlayNode {
   public:
    PlayNode(int argc, char **argv) {
        ros::init(argc, argv, "map_node");
        n = std::make_unique<ros::NodeHandle>();

        //tfBuffer = new tf2_ros::Buffer(ros::Duration(100));
        //tf_listener = new tf2_ros::TransformListener(*tfBuffer);

        ROS_INFO("Waiting for player/kinect_processed");
        ros::topic::waitForMessage<PointCloud>("player/kinect_processed");

        detected_objects =
            n->subscribe("player/kinect_processed", 1,
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

        /*
        pcl::PointXYZRGB p (1.0, 2.0, 3.0);
        uint8_t r = 255, g = 255, b = 0;    // Example: Red color
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        p.rgb = *reinterpret_cast<float*>(&rgb);
        msg->points.push_back (p);
        */
        msg->points = cloud.points;

        kinect_pub.publish(msg);
    }

    void process_messages() {


        got_detected_objects = false;  // reset
    }

    bool got_messages() const {
        return got_detected_objects; 
    }

   private:
    bool got_detected_objects;

    std::unique_ptr<ros::NodeHandle> n;
    ros::Publisher kinect_pub;
    ros::Subscriber detected_objects;

    //tf2_ros::Buffer *tfBuffer;
    //tf2_ros::TransformListener *tf_listener;

    PointCloud detected_objects_msg;
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
