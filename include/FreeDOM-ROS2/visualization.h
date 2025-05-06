//
// Created by ZhiangQi on 25-5-5.
//

#ifndef VISUALIZATION_H
#define VISUALIZATION_H
#include <Eigen/Eigen>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs//msg/point_cloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.h>
#include <sensor_msgs/msg/image.hpp>

#include "FreeDOM-ROS2/freedom.h"
namespace freedom{
class Visualizer{
public:
    struct Config
    {
        std::string map_tf_frame;
        double sub_voxel_size;
        int voxel_depth;
        int block_depth;
        bool enable_raycast_enhancement;
    };

    Visualizer(){}

    void set_params(const Config& config,rclcpp::Node& nh);

    void visualize_scan_removal_result(const ScanMap& scan);
    void visualize_raycast_enhancement_result(const DepthImage& image);
    void visualize_map_removal_result(const MRMap& map);
private:
    std::string map_tf_frame;
    double sub_voxel_size;
    double voxel_size;
    double block_size;
    double half_sub_voxel_size;
    double half_voxel_size;
    double half_block_size;
    Point half_sub_voxel_bias;
    Point half_voxel_bias;
    Point half_block_bias;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_blocks_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_voxels_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr enhanced_depth_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr enhanced_pointcloud_pub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raycasted_blocks_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raycasted_voxels_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr free_blocks_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr free_voxels_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_blocks_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_voxels_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_subvoxels_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_pointcloud_pub;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scan_map_range_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_map_range_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr raycast_map_range_pub;

    bool enable_raycast_enhancement;

    // scan removal results
    void visualize_scan_blocks(const ScanMap& scan);
    void visualize_scan_voxels(const ScanMap& scan);
    void visualize_clusters(const ScanMap& scan);

    // raycast enhancement results
    void visualize_depth_image(const DepthImage& image);
    void visualize_enhanced_pointcloud(const DepthImage& image);

    // map removal results
    void visualize_raycasted_blocks(const MRMap& map);
    void visualize_raycasted_voxels(const MRMap& map);
    void visualize_free_blocks(const MRMap& map);
    void visualize_free_voxels(const MRMap& map);

    void visualize_static_blocks(const MRMap& map);
    void visualize_static_voxels(const MRMap& map);
    void visualize_static_subvoxels(const MRMap& map);
    void visualize_static_pointcloud(const MRMap& map);

    void visualize_scan_map_range(const ScanMap& scan);
    void visualize_local_map_range(const MRMap& map);
    void visualize_raycast_map_range(const MRMap& map);
};
}
#endif //VISUALIZATION_H
