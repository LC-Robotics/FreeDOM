#ifndef _FREENODE_H
#define _FREENODE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>

#include "freedom/freedom.h"
#include "freedom/visualization.h"

namespace freedom{
class FreeNode{
public:
    FreeNode();

    ~FreeNode();

private:
    FreeDOM static_map;

    std::string map_tf_frame;
    std::string sensor_tf_frame;

    ros::CallbackQueue callback_queue;
    std::thread thread;

    std::string pointcloud_topic;
    ros::Subscriber pointcloud_sub;

    std::string save_map_topic;
    std::string save_map_path;
    ros::Subscriber save_map_sub;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    void get_params(ros::NodeHandle& nh_private,FreeDOM::Config& map_config,Visualizer::Config& vis_config);
    void mapping_thread();
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud);
    void save_map_callback(const std_msgs::Empty::ConstPtr& msg);

    bool enable_visualization;
    bool enable_raycast_enhancement;
    Visualizer visualizer;

    Timer timer;
};
}
#endif