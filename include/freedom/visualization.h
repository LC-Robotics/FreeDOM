#ifndef _VISUALIZATION_H
#define _VISUALIZATION_H

#include <Eigen/Eigen>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>

#include "freedom/freedom.h"

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

    void set_params(const Config& config,ros::NodeHandle& nh);

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

    ros::Publisher scan_blocks_pub;
    ros::Publisher scan_voxels_pub;
    ros::Publisher clusters_pub;
    
    ros::Publisher depth_image_pub;
    ros::Publisher enhanced_depth_image_pub;
    ros::Publisher enhanced_pointcloud_pub;

    ros::Publisher raycasted_blocks_pub;
    ros::Publisher raycasted_voxels_pub;
    ros::Publisher free_blocks_pub;
    ros::Publisher free_voxels_pub;
    ros::Publisher static_blocks_pub;
    ros::Publisher static_voxels_pub;
    ros::Publisher static_subvoxels_pub;
    ros::Publisher static_pointcloud_pub;

    ros::Publisher scan_map_range_pub;
    ros::Publisher local_map_range_pub;
    ros::Publisher raycast_map_range_pub;

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
#endif