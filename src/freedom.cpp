#include "freedom/freedom.h"

namespace freedom{
void FreeDOM::set_params(const Config& config)
{
    // set params
    ScanMap::ScanMapConfig scan_config{
        config.sub_voxel_size,
        config.voxel_depth,
        config.block_depth,
        config.enable_local_map,
        config.local_map_range,
        config.local_map_min_z,
        config.local_map_max_z,
        config.num_threads,
        config.sensor_min_range,
        config.sensor_max_range,
        config.sensor_min_z,
        config.sensor_max_z
    };
    scan.set_params(scan_config);
    
    MRMap::MRMapConfig map_config{
        config.sub_voxel_size,
        config.voxel_depth,
        config.block_depth,
        config.enable_local_map,
        config.local_map_range,
        config.local_map_min_z,
        config.local_map_max_z,
        config.num_threads,
        config.sensor_max_range,
        config.sensor_min_z,
        config.sensor_max_z,
        config.raycast_max_range,
        config.raycast_min_z,
        config.raycast_max_z,
        config.counts_to_free,
        config.counts_to_revert,
        config.conservative_connectivity,
        config.aggressive_connectivity
    };
    map.set_params(map_config);

    enable_raycast_enhancement = config.enable_raycast_enhancement;
    if(enable_raycast_enhancement)
    {
        DepthImage::DepthImageConfig depth_image_config{
            config.lidar_horizon_fov,
            config.lidar_vertical_fov_upper,
            config.lidar_vertical_fov_lower,
            config.depth_image_vertical_lines,
            config.depth_image_min_range,
            config.max_raycast_enhancement_range,
            config.raycast_enhancement_depth_margin,
            config.inpaint_size,
            config.erosion_size,
            config.min_raycast_enhancement_area,
            config.depth_image_top_margin,
            config.learn_fov,
            config.enable_fov_mask,
            config.fov_mask_path,
            config.num_threads
        };
        depth_image.set_params(depth_image_config);
    }

    // init variables
    scan_seq = 0;
}

void FreeDOM::set_scan_removal_callback(std::function<void(const ScanMap&)> callback)
{
    scan_removal_callback = callback;
}

void FreeDOM::set_raycast_enhancement_callback(std::function<void(const DepthImage&)> callback)
{
    raycast_enhancement_callback = callback;
}

void FreeDOM::set_map_removal_callback(std::function<void(const MRMap&)> callback)
{
    map_removal_callback = callback;
}

void FreeDOM::pointcloud_integrate(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform)
{
    // 构建scan map,为了快速随机查找与快速遍历
    timer["build scan map"].start();
    scan.build_scan_map(cloud,transform);
    timer["build scan map"].stop();

    // 遍历scan voxel，查找对应的free voxel并设置scan voxel为CONSERVATIVE_DYNAMIC且加入occ_in_freespace
    // 对occ_in_freespace进行聚类并完善其DynamicLevel
    timer["scan removal"].start();
    map.scan_removal(scan);
    timer["scan removal"].stop();

    // 滤除当前帧动态点云后立即触发回调
    if(scan_removal_callback)
        scan_removal_callback(scan);

    if(enable_raycast_enhancement)
    {
        timer["raycast enhancement"].start();
        depth_image.raycast_enhancement(cloud,transform);
        timer["raycast enhancement"].stop();

        raycast_enhancement_callback(depth_image);
    }

    Indices freespace_incremental;
    // FreeSpace estimation
    timer["raycast"].start();
    map.freespace_estimation(scan,depth_image,freespace_incremental);
    timer["raycast"].stop();

    // map clearing
    timer["map removal"].start();
    map.map_removal(freespace_incremental);
    timer["map removal"].stop();

    // map integration
    timer["staticspace integration"].start();
    map.staticspace_integration(scan,scan_seq);
    timer["staticspace integration"].stop();

    timer["remove map out of bound"].start();
    map.remove_map_out_of_bound();
    timer["remove map out of bound"].stop();

    // 集成地图后立即触发回调
    if(map_removal_callback)
        map_removal_callback(map);

    timer["reset"].start();
    scan.reset();
    map.reset();
    timer["reset"].stop();
    ++ scan_seq;
}

void FreeDOM::save_map(const std::string save_map_path)
{
    pcl::PointCloud<pcl::PointXYZ> pointcloud_voxel;
    pcl::PointCloud<pcl::PointXYZ> pointcloud_point;

    unsigned int block_idx_size = map.getVoxel2blockMultiples();
    unsigned int voxel_idx_size = map.getSubvoxel2voxelMultiples();
    unsigned int voxel_num = map.getVoxel2blockMultiplesCubed();
    unsigned int sub_voxel_num = map.getSubvoxel2voxelMultiplesCubed();

    double block_size = map.getBlockSize();
    double voxel_size = map.getVoxelSize();
    double sub_voxel_size = map.getSubVoxelSize();
    PointBias half_sub_voxel_bias = PointBias(sub_voxel_size,sub_voxel_size,sub_voxel_size) / 2.0;

    for(const auto& block_pair : map.get_static_blocks())
    {
        PointBias block_bias = block_size * block_pair.first.cast<double>();
        const StaticBlock& static_block = block_pair.second;

        Index local_voxel_idx(0,0,0);
        for(unsigned int i = 0; i < voxel_num; ++i)
        {
            if(!static_block.is_static_voxel_allocated(i))
            {
                incrementIdx(local_voxel_idx,block_idx_size);
                continue;
            }

            const StaticVoxel& static_voxel = block_pair.second.getStaticVoxel(i);

            PointBias voxel_bias = voxel_size * local_voxel_idx.cast<double>();

            Index local_subvoxel_idx(0,0,0);
            for(unsigned int j = 0; j < sub_voxel_num; ++j)
            {
                if(static_voxel.scan_in_subvoxel[j] == StaticVoxel::NOT_A_SCAN || static_voxel.dynamic_level[j] > DynamicLevel::STATIC)
                {
                    incrementIdx(local_subvoxel_idx,voxel_idx_size);
                    continue;
                }

                PointBias subvoxel_bias = sub_voxel_size * local_subvoxel_idx.cast<double>();
                Point point_voxel = block_bias + voxel_bias + subvoxel_bias + half_sub_voxel_bias;
                Point point = block_bias + voxel_bias + static_voxel.points[j].cast<double>();

                pointcloud_voxel.emplace_back(pcl::PointXYZ(point_voxel.x(),point_voxel.y(),point_voxel.z()));
                pointcloud_point.emplace_back(pcl::PointXYZ(point.x(),point.y(),point.z()));
                incrementIdx(local_subvoxel_idx,voxel_idx_size);
            }
            incrementIdx(local_voxel_idx,block_idx_size);
        }
    }

    pointcloud_voxel.width = pointcloud_voxel.points.size();
    pointcloud_voxel.height = 1;
    pcl::io::savePCDFileASCII(save_map_path + "static_map_voxel.pcd", pointcloud_voxel);

    pointcloud_point.width = pointcloud_point.points.size();
    pointcloud_point.height = 1;
    pcl::io::savePCDFileASCII(save_map_path + "static_map_point.pcd", pointcloud_point);
}
}