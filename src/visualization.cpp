//
// Created by ZhiangQi on 25-5-5.
//

#include "FreeDOM-ROS2/visualization.h"

namespace freedom{
void Visualizer::set_params(const Config& config,rclcpp::Node& nh)
{
    map_tf_frame = config.map_tf_frame;
    sub_voxel_size = config.sub_voxel_size;
    voxel_size = sub_voxel_size*pow(2,config.voxel_depth);
    block_size = sub_voxel_size*pow(2,config.block_depth);
    half_sub_voxel_size = sub_voxel_size/2;
    half_voxel_size = voxel_size/2;
    half_block_size = block_size/2;
    half_sub_voxel_bias = Eigen::Vector3d(sub_voxel_size,sub_voxel_size,sub_voxel_size)/2.0;
    half_voxel_bias = Eigen::Vector3d(voxel_size,voxel_size,voxel_size)/2.0;
    half_block_bias = Eigen::Vector3d(block_size,block_size,block_size)/2.0;
    enable_raycast_enhancement = config.enable_raycast_enhancement;

    scan_blocks_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("scan_blocks",10);
    scan_voxels_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("scan_voxels",10);
    clusters_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("clusters",10);
    scan_map_range_pub = nh.create_publisher<visualization_msgs::msg::Marker>("scan_map_range",10);

    depth_image_pub = nh.create_publisher<sensor_msgs::msg::Image>("depth_image",10);
    enhanced_depth_image_pub = nh.create_publisher<sensor_msgs::msg::Image>("enhanced_depth_image",10);
    enhanced_pointcloud_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("enhanced_pointcloud",10);

    raycasted_blocks_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("raycasted_blocks",10);
    raycasted_voxels_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("raycasted_voxels",10);
    free_blocks_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("free_blocks",10);
    free_voxels_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("free_voxels",10);

    static_blocks_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("static_blocks",10);
    static_voxels_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("static_voxels",10);
    static_subvoxels_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("static_subvoxels",10);
    static_pointcloud_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("static_pointcloud",10);

    raycast_map_range_pub = nh.create_publisher<visualization_msgs::msg::Marker>("raycast_range",10);
    local_map_range_pub = nh.create_publisher<visualization_msgs::msg::Marker>("local_map_range",10);
}

void Visualizer::visualize_scan_removal_result(const ScanMap& scan)
{
    if(scan_blocks_pub->get_subscription_count() > 0)
        visualize_scan_blocks(scan);

    if(scan_voxels_pub->get_subscription_count() > 0)
        visualize_scan_voxels(scan);

    if(clusters_pub->get_subscription_count() > 0)
        visualize_clusters(scan);

    if(scan_map_range_pub->get_subscription_count()>0)
        visualize_scan_map_range(scan);
}

void Visualizer::visualize_scan_blocks(const ScanMap& scan)
{
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    const ScanMap::ScanBlockList& scan_blocks = scan.get_scan_blocks();

    pointcloud.resize(scan_blocks.size());
    auto pointcloud_it =  pointcloud.begin();
    for(const ScanMap::ScanBlock& scan_block : scan_blocks)
    {
        Eigen::Vector3d block_coord = block_size * scan_block.block_idx.cast<double>() + half_block_bias;

        pointcloud_it->x = block_coord.x();
        pointcloud_it->y = block_coord.y();
        pointcloud_it->z = block_coord.z();
        ++ pointcloud_it;
    }
    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;
    scan_blocks_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_scan_voxels(const ScanMap& scan)
{
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    const ScanMap::ScanBlockList& scan_blocks = scan.get_scan_blocks();

    size_t total_scan_voxels = 0;
    for(const ScanMap::ScanBlock& scan_block : scan_blocks){
        total_scan_voxels += scan_block.scan_voxels.size();
    }

    pointcloud.resize(total_scan_voxels);
    auto pointcloud_it =  pointcloud.begin();
    for(const ScanMap::ScanBlock& scan_block : scan_blocks)
    {
        for(const ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels)
        {
            Eigen::Vector3d voxel_coord = voxel_size * scan_voxel.voxel_idx.cast<double>() + half_voxel_bias;

            pointcloud_it->x = voxel_coord.x();
            pointcloud_it->y = voxel_coord.y();
            pointcloud_it->z = voxel_coord.z();
            // pointcloud_it->x = scan_voxel.center.x();
            // pointcloud_it->y = scan_voxel.center.y();
            // pointcloud_it->z = scan_voxel.center.z();

            switch(scan_voxel.dynamic_level)
            {
                case DynamicLevel::STATIC:
                    pointcloud_it->r = 255;
                    pointcloud_it->g = 255;
                    pointcloud_it->b = 255;
                    break;
                case DynamicLevel::AGGRESSIVE_DYNAMIC:
                    pointcloud_it->r = 0;
                    pointcloud_it->g = 255;
                    pointcloud_it->b = 255;
                    break;
                case DynamicLevel::MODERATE_DYNAMIC:
                    pointcloud_it->r = 128;
                    pointcloud_it->g = 255;
                    pointcloud_it->b = 0;
                    break;
                case DynamicLevel::CONSERVATIVE_DYNAMIC:
                    pointcloud_it->r = 255;
                    pointcloud_it->g = 0;
                    pointcloud_it->b = 0;
                    break;
            }
            ++ pointcloud_it;
        }
    }
    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;
    scan_voxels_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_clusters(const ScanMap& scan)
{
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    const ScanMap::ScanBlockList& scan_blocks = scan.get_scan_blocks();
    for(const ScanMap::ScanBlock& scan_block : scan_blocks)
    {
        for(const ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels)
        {
            if(scan_voxel.dynamic_level == DynamicLevel::STATIC)
                continue;

            pcl::PointXYZRGB pcl_point;
            Eigen::Vector3d voxel_coord = voxel_size * scan_voxel.voxel_idx.cast<double>() + half_voxel_bias;
            pcl_point.x = voxel_coord.x();
            pcl_point.y = voxel_coord.y();
            pcl_point.z = voxel_coord.z();

            switch(scan_voxel.dynamic_level)
            {
                case DynamicLevel::STATIC:
                    pcl_point.r = 255;
                    pcl_point.g = 255;
                    pcl_point.b = 255;
                    break;
                case DynamicLevel::AGGRESSIVE_DYNAMIC:
                    pcl_point.r = 0;
                    pcl_point.g = 255;
                    pcl_point.b = 255;
                    break;
                case DynamicLevel::MODERATE_DYNAMIC:
                    pcl_point.r = 128;
                    pcl_point.g = 255;
                    pcl_point.b = 0;
                    break;
                case DynamicLevel::CONSERVATIVE_DYNAMIC:
                    pcl_point.r = 255;
                    pcl_point.g = 0;
                    pcl_point.b = 0;
                    break;
            }
            pointcloud.emplace_back(pcl_point);
        }
    }
    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;
    clusters_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_scan_map_range(const ScanMap& scan)
{
    visualization_msgs::msg::Marker line;
    line.header.frame_id = map_tf_frame;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.type =  visualization_msgs::msg::Marker::LINE_LIST;
    line.ns = "local_map_range";
    line.id = 0;
    line.scale.x = 0.4;
    line.color.r = 1.0;
    line.color.g = 0.0;
    line.color.b = 0.0;
    line.color.a = 1.0;
    line.pose.orientation.x = 0.0;
    line.pose.orientation.y = 0.0;
    line.pose.orientation.z = 0.0;
    line.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point p[24];
    Eigen::Vector3d min,max;
    min = scan.get_scan_map_min();
    max = min + scan.get_scan_map_size();
    p[0].x = max.x();  p[0].y = max.y();  p[0].z = max.z();
    p[1].x = min.x();  p[1].y = max.y();  p[1].z = max.z();
    p[2].x = max.x();  p[2].y = max.y();  p[2].z = max.z();
    p[3].x = max.x();  p[3].y = min.y();  p[3].z = max.z();
    p[4].x = max.x();  p[4].y = max.y();  p[4].z = max.z();
    p[5].x = max.x();  p[5].y = max.y();  p[5].z = min.z();
    p[6].x = min.x();  p[6].y = min.y();  p[6].z = min.z();
    p[7].x = max.x();  p[7].y = min.y();  p[7].z = min.z();
    p[8].x = min.x();  p[8].y = min.y();  p[8].z = min.z();
    p[9].x = min.x();  p[9].y = max.y();  p[9].z = min.z();
    p[10].x = min.x(); p[10].y = min.y(); p[10].z = min.z();
    p[11].x = min.x(); p[11].y = min.y(); p[11].z = max.z();
    p[12].x = min.x(); p[12].y = max.y(); p[12].z = max.z();
    p[13].x = min.x(); p[13].y = max.y(); p[13].z = min.z();
    p[14].x = min.x(); p[14].y = max.y(); p[14].z = max.z();
    p[15].x = min.x(); p[15].y = min.y(); p[15].z = max.z();
    p[16].x = max.x(); p[16].y = min.y(); p[16].z = max.z();
    p[17].x = max.x(); p[17].y = min.y(); p[17].z = min.z();
    p[18].x = max.x(); p[18].y = min.y(); p[18].z = max.z();
    p[19].x = min.x(); p[19].y = min.y(); p[19].z = max.z();
    p[20].x = max.x(); p[20].y = max.y(); p[20].z = min.z();
    p[21].x = min.x(); p[21].y = max.y(); p[21].z = min.z();
    p[22].x = max.x(); p[22].y = max.y(); p[22].z = min.z();
    p[23].x = max.x(); p[23].y = min.y(); p[23].z = min.z();
    for(int i = 0; i < 24; i++)
    {
  	    line.points.push_back(p[i]);
    }

    scan_map_range_pub->publish(line);
}

void Visualizer::visualize_raycast_enhancement_result(const DepthImage& image)
{
    if(enable_raycast_enhancement && (depth_image_pub->get_subscription_count()>0 || enhanced_depth_image_pub->get_subscription_count()>0))
        visualize_depth_image(image);

    if(enable_raycast_enhancement && enhanced_pointcloud_pub->get_subscription_count()>0)
        visualize_enhanced_pointcloud(image);
}

void Visualizer::visualize_depth_image(const DepthImage& image)
{
    const cv::Mat& depth_image = image.get_depth_image();
    const cv::Mat& raycast_enhance_region = image.get_raycast_enhance_region();
    const cv::Mat& inpainted_image = image.get_inpainted_image();
    const cv::Mat& eroded_raycast_enhance_region = image.get_eroded_raycast_enhance_region();
    const cv::Mat& fov_image = image.get_fov_image();

    cv::Mat colored_depth_image;
    cv::Mat colored_inpainted_image;
    cv::applyColorMap(depth_image, colored_depth_image, cv::COLORMAP_JET);
    cv::applyColorMap(inpainted_image, colored_inpainted_image, cv::COLORMAP_JET);

    colored_depth_image.setTo(cv::Scalar(0, 0, 0), raycast_enhance_region);
    colored_depth_image.setTo(cv::Scalar(0, 0, 0), fov_image == 0);
    sensor_msgs::msg::Image::SharedPtr depth_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colored_depth_image).toImageMsg();

    colored_inpainted_image.copyTo(colored_depth_image, eroded_raycast_enhance_region);
    sensor_msgs::msg::Image::SharedPtr inpainted_depth_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colored_depth_image).toImageMsg();

    depth_image_pub->publish(*depth_image_msg);
    enhanced_depth_image_pub->publish(*inpainted_depth_image_msg);
}

void Visualizer::visualize_enhanced_pointcloud(const DepthImage& image)
{
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    const Points& enhanced_pointcloud = image.get_enhanced_pointcloud();
    for(const Point& point : enhanced_pointcloud)
    {
        pcl::PointXYZ pcl_point(point.x(),point.y(),point.z());
        pointcloud.push_back(pcl_point);
    }

    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;
    enhanced_pointcloud_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_map_removal_result(const MRMap& map)
{
    if(raycasted_blocks_pub->get_subscription_count()>0)
        visualize_raycasted_blocks(map);

    if(raycasted_voxels_pub->get_subscription_count()>0)
        visualize_raycasted_voxels(map);

    if(free_blocks_pub->get_subscription_count()>0)
        visualize_free_blocks(map);

    if(free_voxels_pub->get_subscription_count()>0)
        visualize_free_voxels(map);

    if(static_blocks_pub->get_subscription_count()>0)
        visualize_static_blocks(map);

    if(static_voxels_pub->get_subscription_count()>0)
        visualize_static_voxels(map);

    if(static_subvoxels_pub->get_subscription_count()>0)
        visualize_static_subvoxels(map);

    if(static_pointcloud_pub->get_subscription_count()>0)
        visualize_static_pointcloud(map);

    if(raycast_map_range_pub->get_subscription_count()>0)
        visualize_raycast_map_range(map);

    if(map.local_map_enabled() && local_map_range_pub->get_subscription_count()>0)
        visualize_local_map_range(map);
}

void Visualizer::visualize_raycasted_blocks(const MRMap& map)
{
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    const MRMap::LocalRaycastedFlag& raycasted_flag = map.get_raycasted_flags();

    Eigen::Vector3d raycast_map_bias = map.get_raycast_map_min();
    Eigen::Vector3i raycast_map_idx_size = map.get_raycast_map_idx_size();

    Eigen::Vector3i local_block_idx(0,0,0);

    for(const std::atomic<bool>& flag : raycasted_flag)
    {
        // 若该block被raycast过
        if(flag)
        {
            Eigen::Vector3d block_bias = block_size * local_block_idx.cast<double>();

            Eigen::Vector3d point = raycast_map_bias + block_bias + half_block_bias;
            pointcloud.push_back(pcl::PointXYZ(point.x(),point.y(),point.z()));
        }
        incrementIdx(local_block_idx,raycast_map_idx_size);
    }

    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;
    raycasted_blocks_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_raycasted_voxels(const MRMap& map)
{
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    const MRMap::LocalRaycastedFlag& raycasted_flag = map.get_raycasted_flags();
    const MRMap::LocalRaycastBlockGrid& raycast_blocks = map.get_raycast_blocks();

    auto block_it =  raycast_blocks.begin();

    Eigen::Vector3d raycast_map_bias = map.get_raycast_map_min();
    Eigen::Vector3i raycast_map_idx_size = map.get_raycast_map_idx_size();
    unsigned int block_idx_size = map.getVoxel2blockMultiples();

    Eigen::Vector3i local_block_idx(0,0,0);

    for(const std::atomic<bool>& flag : raycasted_flag)
    {
        // 若该block被raycast过
        if(flag)
        {
            Eigen::Vector3d block_bias = block_size * local_block_idx.cast<double>();

            Eigen::Vector3i local_voxel_idx(0,0,0);
            for(const std::atomic<uint64_t>& voxel : block_it->trversed_voxels)
            {
                for(unsigned int bit = 0; bit < 64; ++bit)
                {
                    // 若该voxel为free
                    if (voxel & (1ULL << bit))
                    {
                        Eigen::Vector3d voxel_bias = voxel_size * local_voxel_idx.cast<double>();
                        Eigen::Vector3d point = raycast_map_bias + block_bias + voxel_bias + half_voxel_bias;
                        pointcloud.push_back(pcl::PointXYZ(point.x(),point.y(),point.z()));
                    }
                    incrementIdx(local_voxel_idx,block_idx_size);
                }
            }
        }

        ++ block_it;
        incrementIdx(local_block_idx,raycast_map_idx_size);
    }

    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;
    raycasted_voxels_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_free_blocks(const MRMap& map)
{
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    for(const auto& block_pair : map.get_free_blocks())
    {
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = block_pair.first.x()*block_size + half_block_size;
        pcl_point.y = block_pair.first.y()*block_size + half_block_size;
        pcl_point.z = block_pair.first.z()*block_size + half_block_size;
        if(block_pair.second.is_free())
        {
            pcl_point.r = 0;
            pcl_point.g = 255;
            pcl_point.b = 0;
        }
        else
        {
            pcl_point.r = 255;
            pcl_point.g = 255;
            pcl_point.b = 255;
        }
        pointcloud.emplace_back(pcl_point);
    }

    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;

    free_blocks_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_free_voxels(const MRMap& map)
{
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    unsigned int block_idx_size = map.getVoxel2blockMultiples();
    unsigned int voxel_num = map.getVoxel2blockMultiplesCubed();

    for(const auto& block_pair : map.get_free_blocks())
    {
        Eigen::Vector3d block_bias = block_size * block_pair.first.cast<double>();
        const FreeBlock& free_block = block_pair.second;

        Eigen::Vector3i local_voxel_idx(0,0,0);
        if(free_block.is_free())
        {
            for(unsigned int i = 0; i < voxel_num; ++i)
            {
                Eigen::Vector3d voxel_bias = voxel_size * local_voxel_idx.cast<double>();
                Eigen::Vector3d point = block_bias + voxel_bias + half_voxel_bias;
                pointcloud.push_back(pcl::PointXYZ(point.x(),point.y(),point.z()));
                incrementIdx(local_voxel_idx,block_idx_size);
            }
        }
        else
        {
            for(unsigned int i = 0; i < voxel_num; ++i)
            {
                if(free_block.getFreeVoxel(i).is_free)
                {
                    Eigen::Vector3d voxel_bias = voxel_size * local_voxel_idx.cast<double>();
                    Eigen::Vector3d point = block_bias + voxel_bias + half_voxel_bias;
                    pointcloud.push_back(pcl::PointXYZ(point.x(),point.y(),point.z()));
                }
                incrementIdx(local_voxel_idx,block_idx_size);
            }
        }
    }

    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;

    free_voxels_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_static_blocks(const MRMap& map)
{
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    for(const auto& block_pair : map.get_static_blocks())
    {
        pcl::PointXYZ pcl_point;
        pcl_point.x = block_pair.first.x()*block_size + half_block_size;
        pcl_point.y = block_pair.first.y()*block_size + half_block_size;
        pcl_point.z = block_pair.first.z()*block_size + half_block_size;
        pointcloud.emplace_back(pcl_point);
    }

    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;

    static_blocks_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_static_voxels(const MRMap& map)
{
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    unsigned int block_idx_size = map.getVoxel2blockMultiples();
    unsigned int voxel_num = map.getVoxel2blockMultiplesCubed();

    for(const auto& block_pair : map.get_static_blocks())
    {
        Eigen::Vector3d block_bias = block_size * block_pair.first.cast<double>();
        const StaticBlock& static_block = block_pair.second;

        Eigen::Vector3i local_voxel_idx(0,0,0);
        for(unsigned int i = 0; i < voxel_num; ++i)
        {
            if(static_block.is_static_voxel_allocated(i))
            {
                const StaticVoxel& static_voxel = block_pair.second.getStaticVoxel(i);
                if(static_voxel.static_occ_count > 0)
                {
                    Eigen::Vector3d voxel_bias = voxel_size * local_voxel_idx.cast<double>();
                    Eigen::Vector3d point = block_bias + voxel_bias + half_voxel_bias;
                    pointcloud.push_back(pcl::PointXYZ(point.x(),point.y(),point.z()));
                }
            }

            incrementIdx(local_voxel_idx,block_idx_size);
        }
    }

    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;

    static_voxels_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_static_subvoxels(const MRMap& map)
{
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    unsigned int block_idx_size = map.getVoxel2blockMultiples();
    unsigned int voxel_idx_size = map.getSubvoxel2voxelMultiples();
    unsigned int voxel_num = map.getVoxel2blockMultiplesCubed();
    unsigned int sub_voxel_num = map.getSubvoxel2voxelMultiplesCubed();

    for(const auto& block_pair : map.get_static_blocks())
    {
        Eigen::Vector3d block_bias = block_size * block_pair.first.cast<double>();
        const StaticBlock& static_block = block_pair.second;

        Eigen::Vector3i local_voxel_idx(0,0,0);
        for(unsigned int i = 0; i < voxel_num; ++i)
        {
            if(!static_block.is_static_voxel_allocated(i))
            {
                incrementIdx(local_voxel_idx,block_idx_size);
                continue;
            }

            const StaticVoxel& static_voxel = block_pair.second.getStaticVoxel(i);

            Eigen::Vector3d voxel_bias = voxel_size * local_voxel_idx.cast<double>();

            Eigen::Vector3i local_subvoxel_idx(0,0,0);
            for(unsigned int j = 0; j < sub_voxel_num; ++j)
            {
                if(static_voxel.scan_in_subvoxel[j] == StaticVoxel::NOT_A_SCAN)
                {
                    incrementIdx(local_subvoxel_idx,voxel_idx_size);
                    continue;
                }

                Eigen::Vector3d subvoxel_bias = sub_voxel_size * local_subvoxel_idx.cast<double>();
                Eigen::Vector3d point = block_bias + voxel_bias + subvoxel_bias + half_sub_voxel_bias;

                pcl::PointXYZRGB pcl_point;
                pcl_point.x = point.x();
                pcl_point.y = point.y();
                pcl_point.z = point.z();

                switch(static_voxel.dynamic_level[j])
                {
                    case DynamicLevel::STATIC:
                        pcl_point.r = 255;
                        pcl_point.g = 255;
                        pcl_point.b = 255;
                        break;
                    case DynamicLevel::AGGRESSIVE_DYNAMIC:
                        pcl_point.r = 0;
                        pcl_point.g = 255;
                        pcl_point.b = 255;
                        break;
                    case DynamicLevel::MODERATE_DYNAMIC:
                        pcl_point.r = 128;
                        pcl_point.g = 255;
                        pcl_point.b = 0;
                        break;
                    case DynamicLevel::CONSERVATIVE_DYNAMIC:
                        pcl_point.r = 255;
                        pcl_point.g = 0;
                        pcl_point.b = 0;
                        break;
                }
                pointcloud.emplace_back(pcl_point);

                incrementIdx(local_subvoxel_idx,voxel_idx_size);
            }

            incrementIdx(local_voxel_idx,block_idx_size);
        }
    }

    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;

    static_subvoxels_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_static_pointcloud(const MRMap& map)
{
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    unsigned int block_idx_size = map.getVoxel2blockMultiples();
    unsigned int voxel_idx_size = map.getSubvoxel2voxelMultiples();
    unsigned int voxel_num = map.getVoxel2blockMultiplesCubed();
    unsigned int sub_voxel_num = map.getSubvoxel2voxelMultiplesCubed();

    for(const auto& block_pair : map.get_static_blocks())
    {
        Eigen::Vector3d block_bias = block_size * block_pair.first.cast<double>();
        const StaticBlock& static_block = block_pair.second;

        Eigen::Vector3i local_voxel_idx(0,0,0);
        for(unsigned int i = 0; i < voxel_num; ++i)
        {
            if(!static_block.is_static_voxel_allocated(i))
            {
                incrementIdx(local_voxel_idx,block_idx_size);
                continue;
            }

            const StaticVoxel& static_voxel = block_pair.second.getStaticVoxel(i);

            Eigen::Vector3d voxel_bias = voxel_size * local_voxel_idx.cast<double>();

            Eigen::Vector3i local_subvoxel_idx(0,0,0);
            for(unsigned int j = 0; j < sub_voxel_num; ++j)
            {
                if( static_voxel.scan_in_subvoxel[j] == StaticVoxel::NOT_A_SCAN ||
                    static_voxel.dynamic_level[j] != DynamicLevel::STATIC)
                {
                    incrementIdx(local_subvoxel_idx,voxel_idx_size);
                    continue;
                }

                Eigen::Vector3d point = block_bias + voxel_bias + static_voxel.points[j].cast<double>();

                pointcloud.emplace_back(point.x(),point.y(),point.z());

                incrementIdx(local_subvoxel_idx,voxel_idx_size);
            }

            incrementIdx(local_voxel_idx,block_idx_size);
        }
    }

    pcl::toROSMsg(pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = map_tf_frame;

    static_pointcloud_pub->publish(pointcloud_msg);
}

void Visualizer::visualize_raycast_map_range(const MRMap& map)
{
    visualization_msgs::msg::Marker line;
    line.header.frame_id = map_tf_frame;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.type =  visualization_msgs::msg::Marker::LINE_LIST;
    line.ns = "raycast_map_range";
    line.id = 0;
    line.scale.x = 0.4;
    line.color.r = 0.0;
    line.color.g = 0.0;
    line.color.b = 1.0;
    line.color.a = 1.0;
    line.pose.orientation.x = 0.0;
    line.pose.orientation.y = 0.0;
    line.pose.orientation.z = 0.0;
    line.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point p[24];
    Eigen::Vector3d min,max;
    min = map.get_raycast_map_min();
    max = map.get_raycast_map_max();
    p[0].x = max.x();  p[0].y = max.y();  p[0].z = max.z();
    p[1].x = min.x();  p[1].y = max.y();  p[1].z = max.z();
    p[2].x = max.x();  p[2].y = max.y();  p[2].z = max.z();
    p[3].x = max.x();  p[3].y = min.y();  p[3].z = max.z();
    p[4].x = max.x();  p[4].y = max.y();  p[4].z = max.z();
    p[5].x = max.x();  p[5].y = max.y();  p[5].z = min.z();
    p[6].x = min.x();  p[6].y = min.y();  p[6].z = min.z();
    p[7].x = max.x();  p[7].y = min.y();  p[7].z = min.z();
    p[8].x = min.x();  p[8].y = min.y();  p[8].z = min.z();
    p[9].x = min.x();  p[9].y = max.y();  p[9].z = min.z();
    p[10].x = min.x(); p[10].y = min.y(); p[10].z = min.z();
    p[11].x = min.x(); p[11].y = min.y(); p[11].z = max.z();
    p[12].x = min.x(); p[12].y = max.y(); p[12].z = max.z();
    p[13].x = min.x(); p[13].y = max.y(); p[13].z = min.z();
    p[14].x = min.x(); p[14].y = max.y(); p[14].z = max.z();
    p[15].x = min.x(); p[15].y = min.y(); p[15].z = max.z();
    p[16].x = max.x(); p[16].y = min.y(); p[16].z = max.z();
    p[17].x = max.x(); p[17].y = min.y(); p[17].z = min.z();
    p[18].x = max.x(); p[18].y = min.y(); p[18].z = max.z();
    p[19].x = min.x(); p[19].y = min.y(); p[19].z = max.z();
    p[20].x = max.x(); p[20].y = max.y(); p[20].z = min.z();
    p[21].x = min.x(); p[21].y = max.y(); p[21].z = min.z();
    p[22].x = max.x(); p[22].y = max.y(); p[22].z = min.z();
    p[23].x = max.x(); p[23].y = min.y(); p[23].z = min.z();
    for(int i = 0; i < 24; i++)
    {
  	    line.points.push_back(p[i]);
    }

    raycast_map_range_pub->publish(line);
}

void Visualizer::visualize_local_map_range(const MRMap& map)
{
    visualization_msgs::msg::Marker line;
    line.header.frame_id = map_tf_frame;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.type =  visualization_msgs::msg::Marker::LINE_LIST;
    line.ns = "local_map_range";
    line.id = 0;
    line.scale.x = 0.4;
    line.color.r = 0.0;
    line.color.g = 1.0;
    line.color.b = 0.0;
    line.color.a = 1.0;
    line.pose.orientation.x = 0.0;
    line.pose.orientation.y = 0.0;
    line.pose.orientation.z = 0.0;
    line.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point p[24];
    Eigen::Vector3d min,max;
    min = map.get_local_map_min();
    max = map.get_local_map_max();
    p[0].x = max.x();  p[0].y = max.y();  p[0].z = max.z();
    p[1].x = min.x();  p[1].y = max.y();  p[1].z = max.z();
    p[2].x = max.x();  p[2].y = max.y();  p[2].z = max.z();
    p[3].x = max.x();  p[3].y = min.y();  p[3].z = max.z();
    p[4].x = max.x();  p[4].y = max.y();  p[4].z = max.z();
    p[5].x = max.x();  p[5].y = max.y();  p[5].z = min.z();
    p[6].x = min.x();  p[6].y = min.y();  p[6].z = min.z();
    p[7].x = max.x();  p[7].y = min.y();  p[7].z = min.z();
    p[8].x = min.x();  p[8].y = min.y();  p[8].z = min.z();
    p[9].x = min.x();  p[9].y = max.y();  p[9].z = min.z();
    p[10].x = min.x(); p[10].y = min.y(); p[10].z = min.z();
    p[11].x = min.x(); p[11].y = min.y(); p[11].z = max.z();
    p[12].x = min.x(); p[12].y = max.y(); p[12].z = max.z();
    p[13].x = min.x(); p[13].y = max.y(); p[13].z = min.z();
    p[14].x = min.x(); p[14].y = max.y(); p[14].z = max.z();
    p[15].x = min.x(); p[15].y = min.y(); p[15].z = max.z();
    p[16].x = max.x(); p[16].y = min.y(); p[16].z = max.z();
    p[17].x = max.x(); p[17].y = min.y(); p[17].z = min.z();
    p[18].x = max.x(); p[18].y = min.y(); p[18].z = max.z();
    p[19].x = min.x(); p[19].y = min.y(); p[19].z = max.z();
    p[20].x = max.x(); p[20].y = max.y(); p[20].z = min.z();
    p[21].x = min.x(); p[21].y = max.y(); p[21].z = min.z();
    p[22].x = max.x(); p[22].y = max.y(); p[22].z = min.z();
    p[23].x = max.x(); p[23].y = min.y(); p[23].z = min.z();
    for(int i = 0; i < 24; i++)
    {
  	    line.points.push_back(p[i]);
    }

    local_map_range_pub->publish(line);
}
}