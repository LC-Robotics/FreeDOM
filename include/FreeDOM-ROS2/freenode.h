//
// Created by ZhiangQi on 25-5-5.
//

#ifndef FREENODE_H
#define FREENODE_H
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <thread>
#include <sensor_msgs/msg/point_cloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <std_msgs/msg/empty.hpp>

#include <FreeDOM-ROS2/freedom.h>
#include <FreeDOM-ROS2/visualization.h>

namespace freedom{
    class FreeNode: public rclcpp::Node{

    public:
        FreeNode();

        ~FreeNode();

    private:
        FreeDOM static_map;

        std::string map_tf_frame;
        std::string sensor_tf_frame;

        rclcpp::CallbackGroup::SharedPtr mapping_callback_group_;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> mapping_executor_;
        std::thread thread;

        std::string pointcloud_topic;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;

        std::string save_map_topic;
        std::string save_map_path;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr save_map_sub;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        void get_params(FreeDOM::Config& map_config, Visualizer::Config& vis_config);
        void mapping_thread();
        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud);
        void save_map_callback(const std_msgs::msg::Empty::SharedPtr msg);

        bool enable_visualization;
        bool enable_raycast_enhancement;
        Visualizer visualizer;

        Timer timer;
    };
}

#endif //FREENODE_H
