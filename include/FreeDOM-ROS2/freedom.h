//
// Created by ZhiangQi on 25-5-5.
//

#ifndef FREEDOM_H
#define FREEDOM_H

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <future>
#include <functional>

#include <FreeDOM-ROS2/utils.h>
#include <FreeDOM-ROS2/common_types.h>
#include <FreeDOM-ROS2/mrmap.h>
#include <FreeDOM-ROS2/scanmap.h>
#include <FreeDOM-ROS2/depth_image.h>

namespace freedom
{
    class FreeDOM
    {
    public:
        struct Config
        {
            double sensor_min_range;
            double sensor_max_range;
            double sensor_min_z;
            double sensor_max_z;

            double sub_voxel_size;
            unsigned int voxel_depth;
            unsigned int block_depth;

            bool enable_local_map;
            double local_map_range;
            double local_map_min_z;
            double local_map_max_z;

            double raycast_max_range;
            double raycast_min_z;
            double raycast_max_z;

            unsigned int counts_to_free;
            unsigned int counts_to_revert;

            unsigned int conservative_connectivity;
            unsigned int aggressive_connectivity;

            bool enable_raycast_enhancement;

            double lidar_horizon_fov;
            double lidar_vertical_fov_upper;
            double lidar_vertical_fov_lower;
            unsigned int depth_image_vertical_lines;

            double depth_image_min_range;
            double max_raycast_enhancement_range;
            double raycast_enhancement_depth_margin;

            unsigned int inpaint_size;
            unsigned int erosion_size;
            double min_raycast_enhancement_area;
            double depth_image_top_margin;

            bool learn_fov;
            bool enable_fov_mask;
            std::string fov_mask_path;

            unsigned int num_threads;

        };

        FreeDOM(){}

        void set_params(const Config& config);
        void set_scan_removal_callback(std::function<void(const ScanMap&)> callback);
        void set_raycast_enhancement_callback(std::function<void(const DepthImage&)> callback);
        void set_map_removal_callback(std::function<void(const MRMap&)> callback);

        void pointcloud_integrate(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform);
        void save_map(const std::string save_map_path);
    private:
        // params
        bool enable_raycast_enhancement;

        // variables
        ScanMap scan;
        DepthImage depth_image;
        MRMap map;

        unsigned int scan_seq;

        // scan-removal callback
        std::function<void(const ScanMap&)> scan_removal_callback;
        // raycast-enhancement callback
        std::function<void(const DepthImage&)> raycast_enhancement_callback;
        // map-removal callback
        std::function<void(const MRMap&)> map_removal_callback;

        Timer timer;
    };
}
#endif //FREEDOM_H
