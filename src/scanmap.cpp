#include "freedom/scanmap.h"

namespace freedom{
void ScanMap::set_params(const ScanMapConfig& config)
{
    Map::set_params(config);

    sensor_min_range = config.sensor_min_range;
    sensor_max_range = config.sensor_max_range;
    sensor_min_range_squared = sensor_min_range * sensor_min_range;
    sensor_max_range_squared = sensor_max_range * sensor_max_range;

    sensor_min_z = config.sensor_min_z;
    sensor_max_z = config.sensor_max_z;

    // 为了防止在对边界处的voxel聚类过程中查询超出边界的voxel(最多向外聚类3个体素)，将scan map的range设置大3个体素的大小，这样在查找邻居时就无需判断是否在scan map内
    scan_map.set_params(sensor_max_range+3*voxel_size,sensor_min_z-3*voxel_size,sensor_max_z+3*voxel_size,block_size);

    local_blocks.resize(scan_map.total_block_num);
    block_mutexes = std::make_unique<std::vector<std::mutex>>(scan_map.total_block_num);
}

void ScanMap::build_scan_map(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform)
{
    build_scan_blocks(cloud,transform);
    build_scan_voxels();
}

void ScanMap::build_scan_blocks(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform)
{
    // 根据传感器所在位置更新scanmap与local map范围
    sensor_origin = transform.translation();
    scan_map.setLocalMapBound(sensor_origin);

    double point_min_z = sensor_min_z + sensor_origin.z();
    double point_max_z = sensor_max_z + sensor_origin.z();

    // 并行构建scan blocks
    size_t total_size = cloud.size();
    size_t chunk_size = total_size/num_threads + 1;

    std::vector<std::future<ScanBlockList>> threads;

    for (size_t i = 0; i < num_threads; ++i) {
        size_t start = i * chunk_size;
        size_t end = std::min(start + chunk_size, total_size);

        // 并行构建scan block
        threads.emplace_back(std::async(std::launch::async, [&, start, end]() {
            Point transformed_point;
            Index block_idx;
            LinearIndex local_block_linear_idx;

            ScanBlockList thread_scan_blocks;
            for(auto point_it = cloud.begin() + start; point_it < cloud.begin() + end; ++point_it)
            {
                const pcl::PointXYZ& pcl_point = *point_it;
                Point point_pos(pcl_point.x,pcl_point.y,pcl_point.z);

                transformed_point = transform * point_pos;

                double range_squared = (transformed_point - sensor_origin).squaredNorm();

                if( range_squared < sensor_min_range_squared || 
                    range_squared > sensor_max_range_squared ||
                    transformed_point.z() < point_min_z ||
                    transformed_point.z() > point_max_z )
                    continue;

                getBlockIdxFromPoint(transformed_point,block_idx);
                scan_map.getLocalBlockLinearIdxFromBlockIdx(block_idx,local_block_linear_idx);

                // 获取local block并上锁
                LocalScanBlock& local_block = local_blocks[local_block_linear_idx];
                std::unique_lock<std::mutex> local_block_lock((*block_mutexes)[local_block_linear_idx]);
                
                // 若还没有点(第一次遍历)，则在thread_scan_blocks中新增ScanBlock
                if(local_block.points.empty())
                    thread_scan_blocks.emplace_back(block_idx,local_block_linear_idx);

                local_block.points.emplace_back(transformed_point);
                local_block_lock.unlock();
            }

            return thread_scan_blocks;
        }));
    }

    size_t scan_block_linear_idx = 0;
    for (auto& thread : threads)
    {
        ScanBlockList thread_scan_blocks = thread.get();

        scan_blocks.insert(scan_blocks.end(),std::make_move_iterator(thread_scan_blocks.begin()),std::make_move_iterator(thread_scan_blocks.end()));

        for(const ScanBlock& scan_block: thread_scan_blocks)
        {
            local_blocks[scan_block.local_block_linear_idx].scan_block_linear_idx = scan_block_linear_idx;
            ++ scan_block_linear_idx;
        }
    }
}

void ScanMap::build_scan_voxels()
{
    VectorElementGetter<ScanBlock> scan_block_getter(scan_blocks);
    
    std::vector<std::future<void>> threads;
    for (size_t i = 0; i < num_threads; ++i) {
        threads.emplace_back(std::async(std::launch::async, [&]() {
            Index voxel_idx;
            LinearIndex local_voxel_linear_idx;

            ScanBlock* scan_block_ptr;
            // 逐block计算scan_voxel
            while(scan_block_getter.get_ptr(scan_block_ptr))
            {
                ScanBlock& scan_block = *scan_block_ptr;
                LocalScanBlock& local_block = local_blocks[scan_block.local_block_linear_idx];
                // 对于block中的每个点
                size_t point_linear_idx = 0;
                for(const auto& point : local_block.points)
                {
                    getVoxelIdxFromPoint(point,voxel_idx);
                    getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);

                    auto it = scan_block.local_voxel_linear_idx2scan_voxel_linear_idx.find(local_voxel_linear_idx);
                    // 若已经有这个scan voxel则直接填充
                    if(it != scan_block.local_voxel_linear_idx2scan_voxel_linear_idx.end())
                    {
                        ScanVoxel& scan_voxel = scan_block.scan_voxels[it->second];
                        size_t points_size = scan_voxel.points.size();
                        scan_voxel.center = (static_cast<double>(points_size) / (points_size + 1)) * scan_voxel.center + (1.0 / (points_size + 1)) * point;
                        scan_voxel.points.emplace_back(point_linear_idx);
                    }
                    // 若还没有这个scan voxel,则先创建再填充
                    else
                    {
                        scan_block.local_voxel_linear_idx2scan_voxel_linear_idx[local_voxel_linear_idx] = scan_block.scan_voxels.size();
                        scan_block.scan_voxels.emplace_back(point,voxel_idx,local_voxel_linear_idx);

                        ScanVoxel& scan_voxel = scan_block.scan_voxels.back();
                        scan_voxel.points.emplace_back(point_linear_idx);
                    }
                    ++ point_linear_idx;
                }
            }
        }));
    }

    for (auto& thread : threads)
    {
        thread.get();
    }
}

void ScanMap::reset()
{
    // 遍历block_list清除点
    for(const ScanBlock& scan_block : scan_blocks)
    {
        // 释放所有points以避免占用过大内存
        Points().swap(local_blocks[scan_block.local_block_linear_idx].points);
    }
    // scan_blocks的数量通常变化不大，因此不释放内存以避免反复分配
    scan_blocks.resize(0);
}
}