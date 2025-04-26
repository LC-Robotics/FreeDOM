#ifndef _MRMAP_H
#define _MRMAP_H

#include <Eigen/Eigen>
#include <cmath>
#include <limits>

#include "freedom/map.h"
#include "freedom/scanmap.h"
#include "freedom/utils.h"
#include "freedom/common_types.h"
#include "freedom/raycast.h"
#include "freedom/depth_image.h"

namespace freedom{
class FreeVoxel
{
public:
    bool is_free;                   //是否为自由空间
    unsigned int free_counter;      //连续被观测为free的次数
    unsigned int occ_counter;       //连续被观测为occ的次数

    FreeVoxel():is_free(false),free_counter(0),occ_counter(0){}
};

class FreeBlock
{
public:
    unsigned int free_count;
    FreeVoxel* voxels;

    static unsigned int voxel2blockMultiples_cubed_;

    FreeBlock():free_count(0){allocate_all();}

    ~FreeBlock(){free_all();}

    inline static void set_block_param(unsigned int& voxel2blockMultiples_cubed){voxel2blockMultiples_cubed_ = voxel2blockMultiples_cubed;}

    inline bool is_free() const;    //判断该block是否已经释放

    inline FreeVoxel& getFreeVoxelFromLocalVoxelLinearIdx (const unsigned int local_voxel_linear_idx) const;    //仅当is_free()为false时（即没有free_all()时）能够使用

    //仅当is_free()为false时（即没有free_all()时）能够使用
    inline FreeVoxel& getFreeVoxel(unsigned int local_voxel_linear_idx)
    {
        return voxels[local_voxel_linear_idx];
    }

    //仅当is_free()为false时（即没有free_all()时）能够使用
    inline const FreeVoxel& getFreeVoxel (unsigned int local_voxel_linear_idx) const
    {
        return voxels[local_voxel_linear_idx];
    }

    inline void increaseFreeCount(); //该FreeBlock中的一个FreeVoxel的is_free由false变为true时使用
    inline void decreaseFreeCount(); //该FreeBlock中的一个FreeVoxel的is_free由true变为false时使用

private:
    inline void allocate_all(); //为voxels分配内存
    inline void free_all();     //释放voxels的内存
};

class StaticVoxel
{
public:
    unsigned int* scan_in_subvoxel;     // 每个subvoxel对应点所属的scan seq
    Pointf* points;                     // subvoxel对应点(在voxel内的相对位置，为了节省存储空间使用单精度浮点型)
    DynamicLevel* dynamic_level;        // 每个subvoxel对应点的DynamicLevel

    unsigned int occ_count;             // 含有subvoxel的数量
    unsigned int static_occ_count;      // 含有DynamicLevel为static的subvoxel的数量

    static unsigned int subvoxel2voxelMultiples_cubed_;
    static constexpr unsigned int NOT_A_SCAN = std::numeric_limits<unsigned int>::max();

    StaticVoxel():occ_count(0),static_occ_count(0)
    {
        scan_in_subvoxel = new unsigned int[subvoxel2voxelMultiples_cubed_];
        points = new Pointf[subvoxel2voxelMultiples_cubed_];
        dynamic_level = new DynamicLevel[subvoxel2voxelMultiples_cubed_];
        std::fill(scan_in_subvoxel,scan_in_subvoxel+subvoxel2voxelMultiples_cubed_,NOT_A_SCAN);
    }

    ~StaticVoxel()
    {
        delete[] scan_in_subvoxel;
        delete[] points;
        delete[] dynamic_level;
    }

    inline static void set_voxel_param(unsigned int& subvoxel2voxelMultiples_cubed){subvoxel2voxelMultiples_cubed_ = subvoxel2voxelMultiples_cubed;}

    inline bool is_free(){ return (occ_count == 0); }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class StaticBlock
{
public:
    StaticVoxel** voxels_ptr;
    unsigned int occ_count;     // 已经分配的StaticVoxel数

    static unsigned int voxel2blockMultiples_cubed_;

    StaticBlock():occ_count(0)
    {
        voxels_ptr = new StaticVoxel*[voxel2blockMultiples_cubed_];
        std::fill(voxels_ptr,voxels_ptr+voxel2blockMultiples_cubed_,nullptr);
    }

    ~StaticBlock()
    {
        for(unsigned int i=0; i<voxel2blockMultiples_cubed_; i++)
        {
            //若voxels_ptr[i]为nullptr也不会有问题
            delete voxels_ptr[i];
        }
        delete[] voxels_ptr;
    }

    inline static void set_block_param(unsigned int& voxel2blockMultiples_cubed){voxel2blockMultiples_cubed_ = voxel2blockMultiples_cubed;}

    //StaticVoxel是否已经分配
    inline bool is_static_voxel_allocated(unsigned int local_voxel_linear_idx) const
    {
        return (voxels_ptr[local_voxel_linear_idx] != nullptr);
    }

    //仅当is_static_voxel_allocated(local_voxel_linear_idx)时才能使用
    inline StaticVoxel& getStaticVoxel(unsigned int local_voxel_linear_idx)
    {
        return *voxels_ptr[local_voxel_linear_idx];
    }

    inline const StaticVoxel& getStaticVoxel(unsigned int local_voxel_linear_idx) const
    {
        return *voxels_ptr[local_voxel_linear_idx];
    }

    //判断该block是否为空(为空则应释放该block)
    inline bool is_free() const
    {
        return (occ_count == 0);
    }

    //分配StaticVoxel（仅在!is_static_voxel_allocated(local_voxel_linear_idx)时使用）
    inline void allocate_voxel(unsigned int local_voxel_linear_idx)
    {
        voxels_ptr[local_voxel_linear_idx] = new StaticVoxel();
        occ_count ++;
    }

    //释放StaticVoxel（仅在is_static_voxel_allocated(local_voxel_linear_idx)时使用）
    inline void free_voxel(unsigned int local_voxel_linear_idx)
    {
        delete voxels_ptr[local_voxel_linear_idx];
        voxels_ptr[local_voxel_linear_idx] = nullptr;
        occ_count --;
    }
};

class MRMap : public Map{
public:
    struct MRMapConfig : public Config
    {
        double sensor_max_range;
        double sensor_min_z;
        double sensor_max_z;

        double raycast_max_range;
        double raycast_min_z;
        double raycast_max_z;

        unsigned int counts_to_free;
        unsigned int counts_to_revert;

        unsigned int conservative_connectivity;
        unsigned int aggressive_connectivity;
    };

    // 稠密储存的block单元，保存了该block是否被raycast过，以及使用位压缩储存的trversed_voxels数据
    struct LocalRaycastBlock
    {
        std::vector<std::atomic<uint64_t>> trversed_voxels;

        //由于atomic禁止被拷贝或移动，因此trversed_voxels数量需要在构造时被确定
        LocalRaycastBlock(const unsigned int num_uint64_voxels) : 
            trversed_voxels(num_uint64_voxels){reset();}

        inline void reset();
        inline void free(const unsigned int local_voxel_linear_idx);
    };
    typedef std::vector<std::atomic<bool>,Eigen::aligned_allocator<std::atomic<bool>>> LocalRaycastedFlag;
    typedef std::vector<LocalRaycastBlock,Eigen::aligned_allocator<LocalRaycastBlock>> LocalRaycastBlockGrid;

    struct LocalOccupiedBlock
    {
        std::vector<std::atomic<uint64_t>> occupancy;

        //由于atomic禁止被拷贝或移动，因此trversed_voxels数量需要在构造时被确定
        LocalOccupiedBlock(const unsigned int num_uint64_voxels) : 
            occupancy(num_uint64_voxels){reset();}

        inline void reset();
        inline void occ(const unsigned int local_voxel_linear_idx);
        inline bool is_occ(const unsigned int local_voxel_linear_idx) const;
    };
    typedef std::vector<LocalOccupiedBlock,Eigen::aligned_allocator<LocalOccupiedBlock>> LocalOccupiedBlockGrid;

    MRMap() : Map() {}

    void set_params(const MRMapConfig& config);
    void reset();

    void scan_removal(ScanMap& scan);
    void freespace_estimation(const ScanMap& scan, const DepthImage& depth_image, Indices& freespace_incremental);
    void map_removal(const Indices& freespace_incremental);
    void staticspace_integration(const ScanMap& scan, unsigned int scan_seq);
    void remove_map_out_of_bound();

    inline const LocalRaycastedFlag& get_raycasted_flags() const {return *local_raycast_flag;}
    inline const LocalRaycastBlockGrid& get_raycast_blocks() const {return *local_raycast_blocks;}
    inline const LocalOccupiedBlockGrid& get_occupied_blocks() const {return *local_occupied_blocks;}

    inline const IndexBias& get_raycast_map_min_idx() const {return raycast_map.min_idx;}   // 返回scan map最小block idx
    inline const IndexBias& get_raycast_map_max_idx() const {return raycast_map.max_idx;}   // 返回scan map最大block idx
    inline const IndexBias& get_raycast_map_idx_size() const {return raycast_map.idx_size;} // 返回scan map在xyz的block数量
    inline const PointBias& get_raycast_map_min() const {return raycast_map.min;}     // 返回scan map最小点
    inline const PointBias& get_raycast_map_max() const {return raycast_map.max;}   // 返回scan map最大点
    inline const PointBias& get_raycast_map_size() const {return raycast_map.size;}   // 返回scan map在xyz的长度

    inline const std::unordered_map<Index,FreeBlock,IndexHash>& get_free_blocks() const {return free_space;}
    inline const std::unordered_map<Index,StaticBlock,IndexHash>& get_static_blocks() const {return static_space;}

private:
    // params
    double sensor_max_range;
    double sensor_min_z;
    double sensor_max_z;

    double raycast_max_range;   // raycast的最远距离（相对与LiDAR坐标系）
    double raycast_max_range_squared;   // raycast的最远距离（相对与LiDAR坐标系）
    double raycast_min_z;  // raycast的最小高度（相对与LiDAR坐标系）
    double raycast_max_z;  // raycast的最大高度（相对与LiDAR坐标系）

    unsigned int counts_to_free;
    unsigned int counts_to_revert;

    Neighbours conservative_connectivity;
    bool cluster_aggressive_twice;
    Neighbours first_aggressive_connectivity;
    Neighbours second_aggressive_connectivity;

    // variables
    Point sensor_origin;            // sensor所在位置
    Index sensor_origin_voxel_idx;  // scan map中心(sensor所在voxel)的世界voxel idx
    Index sensor_origin_block_idx;  // scan map中心(sensor所在block)的世界block idx

    // 稠密储存的raycast block
    std::unique_ptr<LocalRaycastedFlag> local_raycast_flag;
    std::unique_ptr<LocalRaycastBlockGrid> local_raycast_blocks;
    std::unique_ptr<LocalOccupiedBlockGrid> local_occupied_blocks;

    RoboCentricMap raycast_map;

    std::unordered_map<Index,FreeBlock,IndexHash> free_space;
    std::unordered_map<Index,StaticBlock,IndexHash> static_space;

    inline Point closestPointInRaycastRange(const Point& end);

    inline FreeBlock* haveFreeBlock(const Index& block_idx);    //返回block指针,若不存在则返回空指针
    inline FreeBlock& getFreeBlock(const Index& block_idx);     //返回block引用,若不存在则创建block

    inline StaticBlock* haveStaticBlock(const Index& block_idx);//返回block指针,若不存在则返回空指针
    inline StaticBlock& getStaticBlock(const Index& block_idx); //返回block引用,若不存在则创建block
};

inline bool FreeBlock::is_free() const
{
    return (free_count == voxel2blockMultiples_cubed_);
}

inline FreeVoxel& FreeBlock::getFreeVoxelFromLocalVoxelLinearIdx (const unsigned int local_voxel_linear_idx) const
{
    return voxels[local_voxel_linear_idx];
}

inline void FreeBlock::increaseFreeCount()
{
    free_count ++;
}

inline void FreeBlock::decreaseFreeCount()
{
    free_count --;
}

inline void FreeBlock::allocate_all()
{
    voxels = new FreeVoxel[voxel2blockMultiples_cubed_];
}

inline void FreeBlock::free_all()
{
    delete[] voxels;
    voxels = nullptr;
}

inline void MRMap::LocalOccupiedBlock::reset()
{
    for (auto& voxel : occupancy)
        voxel.store(0, std::memory_order_relaxed);
}

inline void MRMap::LocalOccupiedBlock::occ(const unsigned int local_voxel_linear_idx)
{
    occupancy[local_voxel_linear_idx/64].fetch_or(1ULL<<(local_voxel_linear_idx%64), std::memory_order_relaxed);
}

inline bool MRMap::LocalOccupiedBlock::is_occ(const unsigned int local_voxel_linear_idx) const
{
    return (occupancy[local_voxel_linear_idx/64].load(std::memory_order_relaxed) & (1ULL << local_voxel_linear_idx%64));
}

inline void MRMap::LocalRaycastBlock::reset()
{
    for (auto& voxel : trversed_voxels)
        voxel.store(0, std::memory_order_relaxed);
}

inline void MRMap::LocalRaycastBlock::free(const unsigned int local_voxel_linear_idx)
{
    trversed_voxels[local_voxel_linear_idx/64].fetch_or(1ULL<<(local_voxel_linear_idx%64), std::memory_order_relaxed);
}

inline Point MRMap::closestPointInRaycastRange(const Point& end)
{
    // 假设光线一秒内行进ray
    PointBias ray = end - sensor_origin;

    // 若在raycast范围内
    if(ray.squaredNorm() <= raycast_max_range_squared && ray.z() > raycast_min_z && ray.z() < raycast_max_z)
        return end;

    // 若在范围外
    double t_r,t_z,min_t;

    // 到达raycast_max_range限制的时间
    t_r = raycast_max_range / ray.norm();

    // 到达raycast_max_z或raycast_min_z限制的时间
    if(ray.z()>0)
        t_z = raycast_max_z / ray.z();
    else if(ray.z()<0)
        t_z = raycast_min_z / ray.z();
    else
        t_z = 1.0;

    // 取最短的时间
    min_t = std::min(t_r,t_z);
    return sensor_origin + ray * min_t;
}

inline FreeBlock* MRMap::haveFreeBlock(const Index& block_idx)
{
    auto it = free_space.find(block_idx);
    if(it != free_space.end())
        return &(it->second);
    else
        return nullptr;
}

inline FreeBlock& MRMap::getFreeBlock(const Index& block_idx)
{
    return free_space[block_idx];
}

inline StaticBlock* MRMap::haveStaticBlock(const Index& block_idx)
{
    auto it = static_space.find(block_idx);
    if(it != static_space.end())
        return &(it->second);
    else
        return nullptr;
}

inline StaticBlock& MRMap::getStaticBlock(const Index& block_idx)
{
    return static_space[block_idx];
}
}
#endif