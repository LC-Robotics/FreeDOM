#include "freedom/mrmap.h"

namespace freedom{
unsigned int FreeBlock::voxel2blockMultiples_cubed_;
unsigned int StaticVoxel::subvoxel2voxelMultiples_cubed_;
unsigned int StaticBlock::voxel2blockMultiples_cubed_;

void MRMap::set_params(const MRMapConfig& config)
{
    Map::set_params(config);

    FreeBlock::set_block_param(voxel2blockMultiples_cubed_);
    StaticVoxel::set_voxel_param(subvoxel2voxelMultiples_cubed_);
    StaticBlock::set_block_param(voxel2blockMultiples_cubed_);

    sensor_max_range = config.sensor_max_range;
    sensor_min_z = config.sensor_min_z;
    sensor_max_z = config.sensor_max_z;

    // raycast的实际范围为local map,scan map与raycast limit的交集
    raycast_max_range = std::min({enable_local_map ? config.local_map_range : 1e10,
                                 config.sensor_max_range,
                                 config.raycast_max_range});
    raycast_max_range_squared = raycast_max_range * raycast_max_range;
    raycast_min_z = std::max({enable_local_map ? config.local_map_min_z : -1e10,
                             config.sensor_min_z,
                             config.raycast_min_z});
    raycast_max_z = std::min({enable_local_map ? config.local_map_max_z : 1e10,
                             config.sensor_max_z,
                             config.raycast_max_z});
    raycast_map.set_params(raycast_max_range,raycast_min_z,raycast_max_z,block_size);

    counts_to_free = config.counts_to_free;
    counts_to_revert = config.counts_to_revert;

    conservative_connectivity.set_params(config.conservative_connectivity);
    if(config.aggressive_connectivity <= 26)
    {
        cluster_aggressive_twice = false;
        first_aggressive_connectivity.set_params(config.aggressive_connectivity);
    }
    else if(config.aggressive_connectivity == 80)
    {
        cluster_aggressive_twice = true;
        first_aggressive_connectivity.set_params(26);
        second_aggressive_connectivity.set_params(6);
    }
    else if(config.aggressive_connectivity == 124)
    {
        cluster_aggressive_twice = true;
        first_aggressive_connectivity.set_params(26);
        second_aggressive_connectivity.set_params(26);
    }
    else
    {
        std::cout << "Aggressive connectivity " << config.aggressive_connectivity << " not supproted" << std::endl;
    }

    local_raycast_flag = std::make_unique<LocalRaycastedFlag>(raycast_map.total_block_num);
    for (auto& flag : *local_raycast_flag)
        flag.store(false, std::memory_order_relaxed);

    local_raycast_blocks = std::make_unique<LocalRaycastBlockGrid>();
    local_raycast_blocks->reserve(raycast_map.total_block_num);
    for (unsigned int i = 0; i < raycast_map.total_block_num; ++i)
        local_raycast_blocks->emplace_back((voxel2blockMultiples_cubed_+63)/64);
    
    local_occupied_blocks = std::make_unique<LocalOccupiedBlockGrid>();
    local_occupied_blocks->reserve(raycast_map.total_block_num);
    for (unsigned int i = 0; i < raycast_map.total_block_num; ++i)
        local_occupied_blocks->emplace_back((voxel2blockMultiples_cubed_+63)/64);
}

void MRMap::scan_removal(ScanMap& scan)
{
    sensor_origin = scan.get_sensor_origin();
    getVoxelIdxFromPoint(sensor_origin,sensor_origin_voxel_idx);
    getBlockIdxFromPoint(sensor_origin,sensor_origin_block_idx);
    raycast_map.setLocalMapBound(sensor_origin);
    if(enable_local_map)                            // 如果没有enable_local_map，则不需要维护local map位置
        local_map.setLocalMapBound(sensor_origin);

    // 逐block找出在free space中的voxel,并赋为conservative dynamic
    VectorElementGetter<ScanMap::ScanBlock> scan_block_getter(scan.get_scan_blocks());
    Indices occ_in_freespace;
    Indices voxels_to_revert;

    std::vector<std::future<std::pair<Indices,Indices>>> conservative_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        conservative_threads.emplace_back(std::async(std::launch::async, [&]() {
            Indices thread_occ_in_freespace;
            Indices thread_voxels_to_revert;

            ScanMap::ScanBlock* scan_block_ptr;
            while(scan_block_getter.get_ptr(scan_block_ptr))
            {
                ScanMap::ScanBlock& scan_block = *scan_block_ptr;

                FreeBlock* free_block_ptr = haveFreeBlock(scan_block.block_idx);
                // 若没有该free block，说明不为free space，直接跳过
                if(free_block_ptr == nullptr)
                    continue;
                
                // 则将全部在free space中的scan voxel赋为CONSERVATIVE_DYNAMIC并加入occ_in_freespace
                LinearIndex local_voxel_linear_idx;
                for(ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels)
                {
                    getLocalVoxelLinearIdxFromVoxelIdx(scan_voxel.voxel_idx,local_voxel_linear_idx);
                    FreeVoxel& free_voxel = free_block_ptr->getFreeVoxelFromLocalVoxelLinearIdx(local_voxel_linear_idx);

                    // 增加occ_counter并将free_counter置0
                    ++ free_voxel.occ_counter;
                    free_voxel.free_counter = 0;

                    // 若free_voxel为free，则将scan voxel赋为CONSERVATIVE_DYNAMIC并加入occ_in_freespace
                    if(free_voxel.is_free)
                    {
                        thread_occ_in_freespace.push_back(scan_voxel.voxel_idx);
                        scan_voxel.dynamic_level = DynamicLevel::CONSERVATIVE_DYNAMIC;
                    }

                    // 若超过counts_to_revert则恢复该voxel与附近的所有voxel,由于多线程下可能影响其他voxel的前面的判断，因此附近部分等所有线程结束再计算
                    if(free_voxel.occ_counter >= counts_to_revert)
                    {
                        // 若被连续占据的体素为is free，则取消其is free
                        if(free_voxel.is_free)
                        {
                            free_voxel.is_free = false;
                            free_block_ptr->decreaseFreeCount();
                        }

                        // 将周围voxel的is_free取消，但不需要变更这些voxel的free count
                        // 若将这些voxel的free count变为0，则会进一步影响这些voxel的邻居的is_free判定
                        // 由于is_free判定需要周围体素free count都大于阈值，因此取消周围voxel的is_free也不会在下一次判定就恢复
                        thread_voxels_to_revert.push_back(scan_voxel.voxel_idx);
                    }
                }
            }

            return std::make_pair(std::move(thread_occ_in_freespace), std::move(thread_voxels_to_revert));
        }));
    }

    for (auto& thread : conservative_threads)
    {
        auto pair = thread.get();
        occ_in_freespace.insert(occ_in_freespace.end(),std::make_move_iterator(pair.first.begin()),std::make_move_iterator(pair.first.end()));
        voxels_to_revert.insert(voxels_to_revert.end(),std::make_move_iterator(pair.second.begin()),std::make_move_iterator(pair.second.end()));
    }

    // 根据occ_in_freespace寻找moderate_dynamic_list，由于多线程可能竞争且耗时不多，使用单线程运行
    // 由于scan map已经预先设置大3个voxel，因此聚类过程中无需关心邻居是否在scan map范围内
    Indices moderate_update_list;
    for(const Index& conservative_voxel_idx : occ_in_freespace)
    {
        Index neighbour_voxel_idx;
        for(auto offset : conservative_connectivity.offsets)
        {
            neighbour_voxel_idx = conservative_voxel_idx + offset;

            ScanMap::ScanVoxel* scan_voxel_ptr = scan.have_scan_voxel(neighbour_voxel_idx);
            if(!scan_voxel_ptr)
                continue;

            ScanMap::ScanVoxel& scan_voxel = *scan_voxel_ptr;
            // 若为conservative或moderate，则说明无需任何操作。若为static，则需要变为moderate并加入moderate_update_list
            if(scan_voxel.dynamic_level < DynamicLevel::MODERATE_DYNAMIC)
            {
                scan_voxel.dynamic_level = DynamicLevel::MODERATE_DYNAMIC;
                moderate_update_list.emplace_back(neighbour_voxel_idx);
            }
        }
    }

    // 根据moderate_update_list寻找aggressive_update_list
    Indices aggressive_update_list;
    for(const Index& moderate_voxel_idx : moderate_update_list)
    {
        Index neighbour_voxel_idx;
        for(auto offset : first_aggressive_connectivity.offsets)
        {
            neighbour_voxel_idx = moderate_voxel_idx + offset;

            ScanMap::ScanVoxel* scan_voxel_ptr = scan.have_scan_voxel(neighbour_voxel_idx);
            if(!scan_voxel_ptr)
                continue;

            ScanMap::ScanVoxel& scan_voxel = *scan_voxel_ptr;
            // 若为conservative或moderate或aggressive，则说明无需任何操作。若为static，则需要变为aggressive并加入aggressive_update_list
            if(scan_voxel.dynamic_level < DynamicLevel::AGGRESSIVE_DYNAMIC)
            {
                scan_voxel.dynamic_level = DynamicLevel::AGGRESSIVE_DYNAMIC;
                aggressive_update_list.emplace_back(neighbour_voxel_idx);
            }
        }
    }

    if(cluster_aggressive_twice)
    { 
        // 若aggressive_connectivity>=26，则需要两次cluster
        for(const Index& aggressive_voxel_idx : aggressive_update_list)
        {
            Index neighbour_voxel_idx;
            for(auto offset : second_aggressive_connectivity.offsets)
            {
                neighbour_voxel_idx = aggressive_voxel_idx + offset;

                ScanMap::ScanVoxel* scan_voxel_ptr = scan.have_scan_voxel(neighbour_voxel_idx);
                if(!scan_voxel_ptr)
                    continue;

                ScanMap::ScanVoxel& scan_voxel = *scan_voxel_ptr;
                // 若为conservative或moderate或aggressive，则说明无需任何操作。若为static，则需要变为aggressive并加入aggressive_update_list
                if(scan_voxel.dynamic_level < DynamicLevel::AGGRESSIVE_DYNAMIC)
                {
                    scan_voxel.dynamic_level = DynamicLevel::AGGRESSIVE_DYNAMIC;
                }
            }
        }
    }

    // 取消该voxel与附近的所有voxel的is_free
    for(const Index& revert_voxel_idx : voxels_to_revert)
    {
        Index neighbour_voxel_idx;
        for(auto offset : conservative_connectivity.offsets)
        {
            neighbour_voxel_idx = revert_voxel_idx + offset;

            Index block_idx;
            LinearIndex local_voxel_linear_idx;
            getBlockIdxFromVoxelIdx(neighbour_voxel_idx,block_idx);
            FreeBlock* free_block_ptr = haveFreeBlock(block_idx);

            // 若邻居不在任何free block中，则无需处理
            if(free_block_ptr == nullptr)
                continue;
            
            getLocalVoxelLinearIdxFromVoxelIdx(neighbour_voxel_idx,local_voxel_linear_idx);
            FreeVoxel& free_voxel = free_block_ptr->getFreeVoxelFromLocalVoxelLinearIdx(local_voxel_linear_idx);

            if(free_voxel.is_free)
            {
                free_voxel.is_free = false;
                free_block_ptr->decreaseFreeCount();
            }
        }
    }
}

void MRMap::freespace_estimation(const ScanMap& scan, const DepthImage& depth_image, Indices& freespace_incremental)
{
    constVectorElementGetter<ScanMap::ScanBlock> scan_block_getter(scan.get_scan_blocks());

    // 填充local_occupied_blocks以方便raycast时快速查找
    std::vector<std::future<void>> occ_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        occ_threads.emplace_back(std::async(std::launch::async, [&]() {
            LinearIndex local_block_linear_idx; // raycast map中的linear idx，不是scan map中的

            // 遍历每个scan block
            const ScanMap::ScanBlock* scan_block_ptr;
            while(scan_block_getter.get_ptr(scan_block_ptr))
            {
                const ScanMap::ScanBlock& scan_block = *scan_block_ptr;

                // 若不在raycast map内，则跳过
                if( scan_block.block_idx.x() < raycast_map.min_idx.x() || scan_block.block_idx.x() >= raycast_map.max_idx.x() ||
                    scan_block.block_idx.y() < raycast_map.min_idx.y() || scan_block.block_idx.y() >= raycast_map.max_idx.y() ||
                    scan_block.block_idx.z() < raycast_map.min_idx.z() || scan_block.block_idx.z() >= raycast_map.max_idx.z())
                    continue;

                raycast_map.getLocalBlockLinearIdxFromBlockIdx(scan_block.block_idx,local_block_linear_idx);
                LocalOccupiedBlock& local_occupied_block = (*local_occupied_blocks)[local_block_linear_idx];

                // 遍历每个scan voxel
                for(const ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels)
                {               
                    // 标记穿过的体素为free
                    local_occupied_block.occ(scan_voxel.local_voxel_linear_idx);
                }
            }
        }));
    }

    for (auto& thread : occ_threads)
    {
        thread.get();
    }

    // raycast获取所有被光线穿过的体素（不包括末端体素）
    scan_block_getter.reset();
    std::vector<std::future<void>> raycast_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        raycast_threads.emplace_back(std::async(std::launch::async, [&]() {
            Index voxel_idx;
            Index block_idx;
            LinearIndex local_voxel_linear_idx;
            LinearIndex local_block_linear_idx;

            RayCaster raycaster(sensor_origin,sensor_origin_voxel_idx,voxel_idx,voxel_size);

            // 遍历每个scan block
            const ScanMap::ScanBlock* scan_block_ptr;
            while(scan_block_getter.get_ptr(scan_block_ptr))
            {
                // 遍历每个scan voxel
                const ScanMap::ScanBlock& scan_block = *scan_block_ptr;
                for(const ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels)
                {
                    // 判断是否在raycast 范围内
                    Point raycast_end = closestPointInRaycastRange(scan_voxel.center);
                    Index raycast_end_voxel_idx;
                    getVoxelIdxFromPoint(raycast_end,raycast_end_voxel_idx);

                    raycaster.setRayEnd(raycast_end,raycast_end_voxel_idx);

                    // 对每个center遍历所有经过的voxels
                    do
                    {
                        getBlockIdxFromVoxelIdx(voxel_idx,block_idx);
                        getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);
                        raycast_map.getLocalBlockLinearIdxFromBlockIdx(block_idx,local_block_linear_idx);

                        LocalRaycastBlock& local_raycast_block = (*local_raycast_blocks)[local_block_linear_idx];
                        LocalOccupiedBlock& local_occupied_block = (*local_occupied_blocks)[local_block_linear_idx];
                        std::atomic<bool>& local_raycasted_flag = (*local_raycast_flag)[local_block_linear_idx];

                        // 若raycast到本scan occ的voxel，则停止这条光线的raycast
                        if(local_occupied_block.is_occ(local_voxel_linear_idx))
                            break;
                        
                        // 标记穿过的体素为free
                        local_raycast_block.free(local_voxel_linear_idx);

                        // 标记穿过的block为raycasted
                        if(!local_raycasted_flag.load(std::memory_order_relaxed))
                            local_raycasted_flag.store(true, std::memory_order_relaxed);

                    }while(raycaster.step());
                }
            }
        }));
    }

    for (auto& thread : raycast_threads)
    {
        thread.get();
    }

    // raycast enhancement
    std::vector<std::future<void>> raycast_enhance_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        raycast_enhance_threads.emplace_back(std::async(std::launch::async, [&]() {
            Index voxel_idx;
            Index block_idx;
            LinearIndex local_voxel_linear_idx;
            LinearIndex local_block_linear_idx;

            RayCaster raycaster(sensor_origin,sensor_origin_voxel_idx,voxel_idx,voxel_size);

            for(const Point& point : depth_image.get_enhanced_pointcloud())
            {
                // 判断是否在raycast 范围内
                Point raycast_end = closestPointInRaycastRange(point);
                Index raycast_end_voxel_idx;
                getVoxelIdxFromPoint(raycast_end,raycast_end_voxel_idx);

                raycaster.setRayEnd(raycast_end,raycast_end_voxel_idx);

                // 对每个center遍历所有经过的voxels
                do
                {
                    getBlockIdxFromVoxelIdx(voxel_idx,block_idx);
                    getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);
                    raycast_map.getLocalBlockLinearIdxFromBlockIdx(block_idx,local_block_linear_idx);

                    LocalRaycastBlock& local_raycast_block = (*local_raycast_blocks)[local_block_linear_idx];
                    LocalOccupiedBlock& local_occupied_block = (*local_occupied_blocks)[local_block_linear_idx];
                    std::atomic<bool>& local_raycasted_flag = (*local_raycast_flag)[local_block_linear_idx];

                    // 若raycast到本scan occ的voxel，则停止这条光线的raycast
                    if(local_occupied_block.is_occ(local_voxel_linear_idx))
                        break;
                    
                    // 标记穿过的体素为free
                    local_raycast_block.free(local_voxel_linear_idx);

                    // 标记穿过的block为raycasted
                    if(!local_raycasted_flag.load(std::memory_order_relaxed))
                        local_raycasted_flag.store(true, std::memory_order_relaxed);

                }while(raycaster.step());
            }
        }));
    }

    for (auto& thread : raycast_enhance_threads)
    {
        thread.get();
    }

    // 寻找所有需要更新的free block idx（被遍历且不为free）,预先为其所在的free block分配空间（由于free space使用的unordered map无法多线程操作）
    std::vector<std::pair<LinearIndex, Index>, Eigen::aligned_allocator<std::pair<LinearIndex, Index>>> free_space_update_list;

    LinearIndex raycast_map_linear_block_idx = 0;
    Index raycast_map_block_idx = raycast_map.min_idx;
    for(const auto& flag : *local_raycast_flag)
    {
        // 该block被raycast过才需要更新对应free block
        if(flag)
        {
            // 如果没有该free block则需要创建，若有直接获取
            FreeBlock& free_block = getFreeBlock(raycast_map_block_idx);

            // 若不为free，则需要更新
            if(!free_block.is_free())
                free_space_update_list.emplace_back(std::make_pair(raycast_map_linear_block_idx,raycast_map_block_idx));
        }
        ++ raycast_map_linear_block_idx;
        incrementIdx(raycast_map_block_idx,raycast_map.min_idx, raycast_map.max_idx);
    }

    // 并行逐block更新free space, 将可能更新为free的free voxel加入free_voxel_update_lists
    std::vector<Indices> free_voxel_update_lists(num_threads);
    constVectorElementGetter<std::pair<LinearIndex,Index>> raycasted_block_idx_getter(free_space_update_list);

    std::vector<std::future<void>> free_space_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        free_space_threads.emplace_back(std::async(std::launch::async, [&,i]() {
            
            const std::pair<LinearIndex,Index>* raycasted_block_idx_ptr;
            while(raycasted_block_idx_getter.get_ptr(raycasted_block_idx_ptr))
            {
                const LocalRaycastBlock& raycast_block = (*local_raycast_blocks)[raycasted_block_idx_ptr->first];
                FreeBlock& free_block = free_space[raycasted_block_idx_ptr->second];

                LinearIndex local_voxel_linear_idx = 0;
                Index min_voxel_idx = voxel2blockMultiples_ * raycasted_block_idx_ptr->second;
                Index max_voxel_idx = min_voxel_idx + IndexBias(voxel2blockMultiples_,voxel2blockMultiples_,voxel2blockMultiples_);
                Index voxel_idx = min_voxel_idx;
                for(auto& voxels : raycast_block.trversed_voxels)
                {
                    for(unsigned int bit = 0; bit < 64; ++bit)
                    {
                        // 若该voxel被穿过
                        if (voxels & (1ULL << bit))
                        {
                            FreeVoxel& free_voxel = free_block.getFreeVoxelFromLocalVoxelLinearIdx(local_voxel_linear_idx);

                            free_voxel.free_counter ++;
                            free_voxel.occ_counter = 0;
                            if(!free_voxel.is_free && free_voxel.free_counter >= counts_to_free)
                            {
                                free_voxel_update_lists[i].emplace_back(voxel_idx);
                            }
                        }
                        ++ local_voxel_linear_idx;
                        incrementIdx(voxel_idx,min_voxel_idx,max_voxel_idx);
                    }
                }
            }
       }));
    }

    for (auto& thread : free_space_threads)
    {
        thread.get();
    }

    // 遍历free_voxel_update_list,更新free voxel的is_free（只有conservative_connectivity内所有free_counter都大于等于counts_to_free才更新为free）
    std::vector<std::future<Indices>> free_voxel_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        free_voxel_threads.emplace_back(std::async(std::launch::async, [&,i]() {
            Indices thread_freespace_incremental;
            for(const Index& voxel_idx : free_voxel_update_lists[i])
            {
                bool all_neighbours_are_free = true;

                Index neighbour_voxel_idx;
                Index neighbour_block_idx;
                LinearIndex local_neighbour_voxel_linear_idx;
                for(auto offset : conservative_connectivity.offsets)
                {
                    neighbour_voxel_idx = voxel_idx + offset;
                    getBlockIdxFromVoxelIdx(neighbour_voxel_idx,neighbour_block_idx);
                    FreeBlock* free_block_ptr = haveFreeBlock(neighbour_block_idx);

                    // 虽然voxel一定在一个非is_free的FreeBlock中，但其邻居可能没有FreeBlock，或者有但已经is_free了
                    // 如果getFreeBlock的邻居恰好为isfree()的freeblock，则会出错，故需要判断free_block是否is_free
                    if(free_block_ptr == nullptr)
                    {
                        all_neighbours_are_free = false;
                        break;
                    }

                    getLocalVoxelLinearIdxFromVoxelIdx(neighbour_voxel_idx,local_neighbour_voxel_linear_idx);
                    FreeVoxel& free_voxel = free_block_ptr->getFreeVoxel(local_neighbour_voxel_linear_idx);
                    if(free_voxel.free_counter < counts_to_free)
                    {
                        all_neighbours_are_free = false;
                        break;
                    }
                }

                if(all_neighbours_are_free)
                {
                    Index block_idx;
                    LinearIndex local_voxel_linear_idx;
                    getBlockIdxFromVoxelIdx(voxel_idx,block_idx);
                    getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);
                    FreeBlock& free_block = getFreeBlock(block_idx);
                    FreeVoxel& free_voxel =  free_block.getFreeVoxel(local_voxel_linear_idx);
                    
                    free_voxel.is_free = true;
                    free_block.increaseFreeCount();

                    thread_freespace_incremental.emplace_back(voxel_idx);
                }
            }
            return thread_freespace_incremental;
        }));
    }

    for (auto& thread : free_voxel_threads)
    {
        Indices thread_freespace_incremental = thread.get();
        freespace_incremental.insert(freespace_incremental.end(),std::make_move_iterator(thread_freespace_incremental.begin()),std::make_move_iterator(thread_freespace_incremental.end()));
    }
}

void MRMap::map_removal(const Indices& freespace_incremental)
{
    typedef std::unordered_map<Index,DynamicLevel,IndexHash> DynamicLevelMap;
    std::unordered_map<unsigned int,DynamicLevelMap> scan2map;

    // 遍历所有freespace_incremental的voxel，找出其中的scan并填充scan2map
    for(const Index& voxel_idx : freespace_incremental)
    {
        Index block_idx;
        getBlockIdxFromVoxelIdx(voxel_idx,block_idx);

        StaticBlock* static_block_ptr;
        static_block_ptr = haveStaticBlock(block_idx);

        //若true，说明该idx没有对应的static voxel
        if(static_block_ptr == nullptr)
            continue;

        LinearIndex local_voxel_linear_idx;
        getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);

        //若true，说明该idx没有对应的static voxel
        if(!static_block_ptr->is_static_voxel_allocated(local_voxel_linear_idx))
            continue;

        StaticVoxel& static_voxel = static_block_ptr->getStaticVoxel(local_voxel_linear_idx);

        // 查找static_voxel包含的所有Scan
        std::vector<unsigned int> all_scans;
        for(unsigned int i = 0; i < subvoxel2voxelMultiples_cubed_; ++i)
        {
            unsigned int& scan_seq = static_voxel.scan_in_subvoxel[i];
            if(scan_seq == StaticVoxel::NOT_A_SCAN)
                continue;

            auto it = std::find(all_scans.begin(), all_scans.end(), scan_seq);
            if(it == all_scans.end())
                all_scans.emplace_back(scan_seq);
        }
        
        for(unsigned int i : all_scans)
        {
            scan2map[i][voxel_idx] = DynamicLevel::CONSERVATIVE_DYNAMIC;
        }
    }

    for(auto& [scan_seq,dynamic_level_map] : scan2map)
    {
        // 提取dynamic_level_map中已经有的键(CONSERVATIVE_DYNAMIC)
        Indices conservative_voxel_list;
        Indices moderate_voxel_list;
        Indices aggressive_voxel_list;
        for(auto& [voxel_idx,dynamic_level] : dynamic_level_map)
        {
            conservative_voxel_list.emplace_back(voxel_idx);
        }

        // 在conservative的邻域内寻找moderate
        for(Index& voxel_idx : conservative_voxel_list)
        {
            Index neighbour_voxel_idx;
            Index neighbour_block_idx;
            LinearIndex local_neighbour_voxel_linear_idx;
            for(auto offset : conservative_connectivity.offsets)
            {
                neighbour_voxel_idx = voxel_idx + offset;

                // 若该scan的dynamic_level_map在这个位置已经有值了，这意味着已经遍历过了，因此跳过
                auto it = dynamic_level_map.find(neighbour_voxel_idx);
                if(it != dynamic_level_map.end())
                    continue;

                getBlockIdxFromVoxelIdx(neighbour_voxel_idx,neighbour_block_idx);
                StaticBlock* static_block_ptr = haveStaticBlock(neighbour_block_idx);

                // 若该voxel位置没有对应的static voxel或对应static voxel没有该scan，则跳过
                if(static_block_ptr == nullptr)
                    continue;

                getLocalVoxelLinearIdxFromVoxelIdx(neighbour_voxel_idx,local_neighbour_voxel_linear_idx);
                if(!static_block_ptr->is_static_voxel_allocated(local_neighbour_voxel_linear_idx))
                    continue;
                
                StaticVoxel& static_voxel = static_block_ptr->getStaticVoxel(local_neighbour_voxel_linear_idx);

                if( std::find(static_voxel.scan_in_subvoxel,static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_, scan_seq)
                        == static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_)
                    continue;

                // 若该voxel位置在dynamic_level_map中还没有被遍历过，且该处的static voxel包含该scan
                // 则将dynamic_level_map的该voxel处标记为MODERATE_DYNAMIC，并将该voxel加入moderate_voxel_list
                dynamic_level_map[neighbour_voxel_idx] = DynamicLevel::MODERATE_DYNAMIC;
                moderate_voxel_list.emplace_back(neighbour_voxel_idx);
            }
        }

        // 在moderate的邻域内寻找aggressive
        for(Index& voxel_idx : moderate_voxel_list)
        {
            Index neighbour_voxel_idx;
            Index neighbour_block_idx;
            LinearIndex local_neighbour_voxel_linear_idx;
            for(auto offset : first_aggressive_connectivity.offsets)
            {
                neighbour_voxel_idx = voxel_idx + offset;

                // 若该scan的dynamic_level_map在这个位置已经有值了，这意味着已经遍历过了，因此跳过
                auto it = dynamic_level_map.find(neighbour_voxel_idx);
                if(it != dynamic_level_map.end())
                    continue;

                getBlockIdxFromVoxelIdx(neighbour_voxel_idx,neighbour_block_idx);
                StaticBlock* static_block_ptr = haveStaticBlock(neighbour_block_idx);

                // 若该voxel位置没有对应的static voxel或对应static voxel没有该scan，则跳过
                if(static_block_ptr == nullptr)
                    continue;

                getLocalVoxelLinearIdxFromVoxelIdx(neighbour_voxel_idx,local_neighbour_voxel_linear_idx);
                if(!static_block_ptr->is_static_voxel_allocated(local_neighbour_voxel_linear_idx))
                    continue;
                
                StaticVoxel& static_voxel = static_block_ptr->getStaticVoxel(local_neighbour_voxel_linear_idx);

                if( std::find(static_voxel.scan_in_subvoxel,static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_, scan_seq)
                        == static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_)
                    continue;

                // 若该voxel位置在dynamic_level_map中还没有被遍历过，且该处的static voxel包含该scan
                // 则将dynamic_level_map的该voxel处标记为AGGRESSIVE_DYNAMIC，并将该voxel加入aggressive_voxel_list
                dynamic_level_map[neighbour_voxel_idx] = DynamicLevel::AGGRESSIVE_DYNAMIC;
                aggressive_voxel_list.emplace_back(neighbour_voxel_idx);
            }
        }

        if(!cluster_aggressive_twice)
        {
            // 在aggressive的邻域内继续聚类
            for(Index& voxel_idx : aggressive_voxel_list)
            {
                Index neighbour_voxel_idx;
                Index neighbour_block_idx;
                LinearIndex local_neighbour_voxel_linear_idx;
                for(auto offset : second_aggressive_connectivity.offsets)
                {
                    neighbour_voxel_idx = voxel_idx + offset;

                    // 若该scan的dynamic_level_map在这个位置已经有值了，这意味着已经遍历过了，因此跳过
                    auto it = dynamic_level_map.find(neighbour_voxel_idx);
                    if(it != dynamic_level_map.end())
                        continue;

                    getBlockIdxFromVoxelIdx(neighbour_voxel_idx,neighbour_block_idx);
                    StaticBlock* static_block_ptr = haveStaticBlock(neighbour_block_idx);

                    // 若该voxel位置没有对应的static voxel或对应static voxel没有该scan，则跳过
                    if(static_block_ptr == nullptr)
                        continue;

                    getLocalVoxelLinearIdxFromVoxelIdx(neighbour_voxel_idx,local_neighbour_voxel_linear_idx);
                    if(!static_block_ptr->is_static_voxel_allocated(local_neighbour_voxel_linear_idx))
                        continue;
                    
                    StaticVoxel& static_voxel = static_block_ptr->getStaticVoxel(local_neighbour_voxel_linear_idx);

                    if( std::find(static_voxel.scan_in_subvoxel,static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_, scan_seq)
                            == static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_)
                        continue;

                    // 若该voxel位置在dynamic_level_map中还没有被遍历过，且该处的static voxel包含该scan
                    // 则将dynamic_level_map的该voxel处标记为AGGRESSIVE_DYNAMIC
                    dynamic_level_map[neighbour_voxel_idx] = DynamicLevel::AGGRESSIVE_DYNAMIC;
                }
            }
        }

        // 根据计算好的dynamic_level_map更新其中包含的所有static voxel的该scan的subvoxel
        Index block_idx;
        LinearIndex local_voxel_linear_idx;
        for(auto& [voxel_idx,target_dynamic_level] : dynamic_level_map)
        {
            getBlockIdxFromVoxelIdx(voxel_idx,block_idx);
            getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);

            // 由于这些voxel对应的static voxel一定存在且被分配了，因此无需再次判断
            StaticBlock& static_block = getStaticBlock(block_idx);
            StaticVoxel& static_voxel = static_block.getStaticVoxel(local_voxel_linear_idx);

            // 获取static_voxel中该scan对应的dynamic level
            unsigned int idx = std::find(static_voxel.scan_in_subvoxel,static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_, scan_seq) - static_voxel.scan_in_subvoxel;
            DynamicLevel dynamic_level = static_voxel.dynamic_level[idx];

            // 若之前的dynamic_level小于target_dynamic_level，则无需更新
            if(dynamic_level >= target_dynamic_level)
                continue;

            // 若dynamic_level小于target_dynamic_level，则需要更新dynamic_level
            unsigned int decreased_static_occ_count = 0;
            for(unsigned int i = 0; i < subvoxel2voxelMultiples_cubed_; ++i)
            {
                unsigned int& subvoxel_scan_seq = static_voxel.scan_in_subvoxel[i];
                // 若不是本scan的subvoxel，则跳过
                if(subvoxel_scan_seq != scan_seq)
                    continue;

                // 更新dynamic_level
                static_voxel.dynamic_level[i] = target_dynamic_level;
                
                // 若原来的dynamic_level为STATIC，则需要减小static_occ_count
                if(dynamic_level == DynamicLevel::STATIC)
                    ++ decreased_static_occ_count;

                // // 目标为CONSERVATIVE_DYNAMIC时需要删除该scan
                // if(target_dynamic_level == DynamicLevel::CONSERVATIVE_DYNAMIC)
                //     subvoxel_scan_seq = StaticVoxel::NOT_A_SCAN;
            }

            static_voxel.static_occ_count -= decreased_static_occ_count;
            if(static_voxel.is_free())
            {
                static_block.free_voxel(local_voxel_linear_idx);

                if(static_block.is_free())
                    static_space.erase(block_idx);
            }
        }
    }
}

void MRMap::staticspace_integration(const ScanMap& scan, unsigned int scan_seq)
{
    // 为了避免多线程冲突，在并行插入点前先单线程插入static_block
    for(const ScanMap::ScanBlock& scan_block : scan.get_scan_blocks())
    {
        static_space[scan_block.block_idx];
    }

    // 并行插入点
    constVectorElementGetter<ScanMap::ScanBlock> scan_block_getter(scan.get_scan_blocks());
    
    std::vector<std::future<void>> integrate_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        integrate_threads.emplace_back(std::async(std::launch::async, [&]() {
            const ScanMap::ScanBlock* scan_block_ptr;
            while(scan_block_getter.get_ptr(scan_block_ptr))
            {
                const ScanMap::ScanBlock& scan_block = *scan_block_ptr;
                const ScanMap::LocalScanBlock& local_scan_block = scan.get_local_blocks()[scan_block.local_block_linear_idx];
                StaticBlock& static_block = getStaticBlock(scan_block.block_idx);
                for(const ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels)
                {
                    // // 若dynamic level为CONSERVATIVE_DYNAMIC，则无需集成进地图
                    // if(scan_voxel.dynamic_level == DynamicLevel::CONSERVATIVE_DYNAMIC)
                    //     continue;

                    // 若对应static voxel不存在，则先分配
                    if(!static_block.is_static_voxel_allocated(scan_voxel.local_voxel_linear_idx))
                        static_block.allocate_voxel(scan_voxel.local_voxel_linear_idx);

                    Point scan_voxel_coord = voxel_size * scan_voxel.voxel_idx.cast<double>();
                    StaticVoxel& static_voxel = static_block.getStaticVoxel(scan_voxel.local_voxel_linear_idx);

                    for(const LinearIndex& point_idx : scan_voxel.points)
                    {
                        const Point& point = local_scan_block.points[point_idx];

                        Index subvoxel_idx;
                        LinearIndex local_subvoxel_linear_idx;
                        getSubVoxelIdxFromPoint(point,subvoxel_idx);
                        getLocalSubVoxelLinearIdxFromSubvoxelIdx(subvoxel_idx,local_subvoxel_linear_idx);

                        unsigned int& subvoxel_scan_id = static_voxel.scan_in_subvoxel[local_subvoxel_linear_idx];
                        Pointf& subvoxel_point = static_voxel.points[local_subvoxel_linear_idx];
                        DynamicLevel& subvoxel_dynamic_level = static_voxel.dynamic_level[local_subvoxel_linear_idx];
                        // 若subvoxel已经有占据了，且这一帧还没有处理过
                        if(subvoxel_scan_id != StaticVoxel::NOT_A_SCAN && subvoxel_scan_id != scan_seq)
                        {
                            // 若subvoxel的DynamicLevel更大，则需要让位与DynamicLevel低的点
                            // 如原来是AGGRESSIVE_DYNAMIC，就要让位给新来的STATIC
                            if(subvoxel_dynamic_level > scan_voxel.dynamic_level)
                            {
                                subvoxel_scan_id = scan_seq;
                                subvoxel_point = (point - scan_voxel_coord).cast<float>();
                                subvoxel_dynamic_level = scan_voxel.dynamic_level;

                                //若新的scan_voxel为STATIC
                                if(scan_voxel.dynamic_level < DynamicLevel::AGGRESSIVE_DYNAMIC)
                                    static_voxel.static_occ_count ++;
                            }
                        }
                        // 若这一帧已经处理过，有重复的subvoxel点，取第一个
                        else if(subvoxel_scan_id == scan_seq)
                            ;
                        // 若subvoxel没有占据
                        else
                        {
                            subvoxel_scan_id = scan_seq;
                            subvoxel_point = (point - scan_voxel_coord).cast<float>();
                            subvoxel_dynamic_level = scan_voxel.dynamic_level;
                            static_voxel.occ_count ++;

                            //若新的scan_voxel为STATIC
                                if(scan_voxel.dynamic_level < DynamicLevel::AGGRESSIVE_DYNAMIC)
                                    static_voxel.static_occ_count ++;
                        }
                    }
                }
            }
        }));
    }

    for (auto& thread : integrate_threads)
    {
        thread.get();
    }
}

void MRMap::reset()
{
    for (auto& flag : *local_raycast_flag)
        flag.store(false, std::memory_order_relaxed);

    for(LocalRaycastBlock& block : *local_raycast_blocks)
    {
        block.reset();
    }

    for(LocalOccupiedBlock& block : *local_occupied_blocks)
    {
        block.reset();
    }
}

void MRMap::remove_map_out_of_bound()
{
    // 若enable_local_map，则去除超出范围的free_space与static_space
    if(enable_local_map)
    {
        Indices free_space_remove_list;
        for(const auto& it : free_space)
        {
            if( it.first.x() < local_map.min_idx.x() || it.first.x() >= local_map.max_idx.x() ||
                it.first.y() < local_map.min_idx.y() || it.first.y() >= local_map.max_idx.y() ||
                it.first.z() < local_map.min_idx.z() || it.first.z() >= local_map.max_idx.z())
                free_space_remove_list.emplace_back(it.first);
        }
        for(const auto& idx : free_space_remove_list)
        {
            free_space.erase(idx);
        }

        Indices static_space_remove_list;
        for(const auto& it : static_space)
        {
            if( it.first.x() < local_map.min_idx.x() || it.first.x() >= local_map.max_idx.x() ||
                it.first.y() < local_map.min_idx.y() || it.first.y() >= local_map.max_idx.y() ||
                it.first.z() < local_map.min_idx.z() || it.first.z() >= local_map.max_idx.z())
                static_space_remove_list.emplace_back(it.first);
        }
        for(const auto& idx : static_space_remove_list)
        {
            static_space.erase(idx);
        }
    }
}
}