#ifndef _MAP_H
#define _MAP_H

#include <Eigen/Eigen>
#include <cmath>
#include <future>

#include "freedom/common_types.h"

namespace freedom{
class Map{
public:
    struct Config
    {
        double sub_voxel_size;
        unsigned int voxel_depth;
        unsigned int block_depth;
        
        bool enable_local_map;
        double local_map_range;
        double local_map_min_z;
        double local_map_max_z;

        unsigned int num_threads;
    };

    struct RoboCentricMap
    {
        // params
        double block_size;
        double block_size_inv;
        unsigned int horizontal_block_num;
        unsigned int vertical_block_num;
        unsigned int horizontal_vertical_block_num_multiplied;
        unsigned int total_block_num;

        IndexBias center_min_bias;    // 从map中心所在block到最小与最大block的bias
        IndexBias center_max_bias;
        IndexBias idx_size;       // map在xyz的block数,等于center_max_bias-center_min_bias+1
        PointBias size;

        // variables
        Index center;         // map中心的block
        IndexBias min_idx;    // map最小的block的idx
        IndexBias max_idx;    // map最大的block的idx加上(1,1,1)
        Point min;
        Point max;

        void set_params(const double& range,const double& min_z,const double& max_z,const double& block_size_);
        void setLocalMapBound(const Point& origin);
        inline void getLocalBlockLinearIdxFromBlockIdx(const Index& idx, LinearIndex& linear_idx) const;
    };

    Map(){}

    inline void getSubVoxelIdxFromPoint(const Point& point, Index& idx) const;
    inline void getVoxelIdxFromPoint(const Point& point, Index& idx) const;
    inline void getBlockIdxFromPoint(const Point& point, Index& idx) const;

    inline void getVoxelIdxFromSubVoxelIdx(const Index& subvoxel_idx, Index& voxel_idx) const;
    inline void getBlockIdxFromSubVoxelIdx(const Index& subvoxel_idx, Index& block_idx) const;
    inline void getBlockIdxFromVoxelIdx(const Index& voxel_idx, Index& block_idx) const;

    inline void getLocalSubVoxelLinearIdxFromSubvoxelIdx(const Index& subvoxel_idx, LinearIndex& local_subvoxel_linear_idx) const;
    inline void getLocalVoxelLinearIdxFromVoxelIdx(const Index& voxel_idx, LinearIndex& local_voxel_linear_idx) const;

    inline double getSubVoxelSize() const {return sub_voxel_size;}
    inline double getVoxelSize() const {return voxel_size;}
    inline double getBlockSize() const {return block_size;}

    inline unsigned int getSubvoxel2voxelMultiples() const {return subvoxel2voxelMultiples_;}
    inline unsigned int getVoxel2blockMultiples() const {return voxel2blockMultiples_;}
    inline unsigned int getSubvoxel2voxelMultiplesCubed() const {return subvoxel2voxelMultiples_cubed_;}
    inline unsigned int getVoxel2blockMultiplesCubed() const {return voxel2blockMultiples_cubed_;}

    inline bool local_map_enabled() const {return enable_local_map;}
    inline const IndexBias& get_local_map_min_idx() const {return local_map.min_idx;}    // 返回local map最小block idx
    inline const IndexBias& get_local_map_max_idx() const {return local_map.max_idx;}    // 返回local map最大block idx
    inline const IndexBias& get_local_map_idx_size() const {return local_map.idx_size;}      // 返回scan map在xyz的block数量
    inline const PointBias& get_local_map_min() const {return local_map.min;}            // 返回scan map最小点
    inline const PointBias& get_local_map_max() const {return local_map.max;}            // 返回scan map最小点
    inline const PointBias& get_local_map_size() const {return local_map.size;}   // 返回scan map在xyz的长度

protected:
    unsigned int voxel_depth;
    unsigned int block_depth;
    unsigned int voxel_to_block_depth;

    unsigned int voxel_depth_doubled;
    unsigned int voxel_to_block_depth_doubled;

    double sub_voxel_size;
    double voxel_size;
    double block_size;

    double sub_voxel_size_inv;
    double voxel_size_inv;
    double block_size_inv;

    unsigned int subvoxel2voxelMultiples_;
    unsigned int subvoxel2voxelMultiples_squared_;
    unsigned int subvoxel2voxelMultiples_cubed_;

    unsigned int voxel2blockMultiples_;
    unsigned int voxel2blockMultiples_squared_;
    unsigned int voxel2blockMultiples_cubed_;

    unsigned int local_subvoxel_mask;
    unsigned int local_voxel_mask;

    bool enable_local_map;
    RoboCentricMap local_map;

    unsigned int num_threads;

    void set_params(const Config& config);
};

inline void Map::RoboCentricMap::getLocalBlockLinearIdxFromBlockIdx(const Index& idx, LinearIndex& linear_idx) const
{
    Index local_idx = idx - min_idx;
    linear_idx = local_idx.x() * horizontal_vertical_block_num_multiplied + local_idx.y() * vertical_block_num + local_idx.z();
}

inline void Map::getSubVoxelIdxFromPoint(const Point& point, Index& idx) const
{
    idx.x() = std::floor(point.x()*sub_voxel_size_inv);
    idx.y() = std::floor(point.y()*sub_voxel_size_inv);
    idx.z() = std::floor(point.z()*sub_voxel_size_inv);
}

inline void Map::getVoxelIdxFromPoint(const Point& point, Index& idx) const
{
    idx.x() = std::floor(point.x()*voxel_size_inv);
    idx.y() = std::floor(point.y()*voxel_size_inv);
    idx.z() = std::floor(point.z()*voxel_size_inv);
}

inline void Map::getBlockIdxFromPoint(const Point& point, Index& idx) const
{
    idx.x() = std::floor(point.x()*block_size_inv);
    idx.y() = std::floor(point.y()*block_size_inv);
    idx.z() = std::floor(point.z()*block_size_inv);
}

inline void Map::getVoxelIdxFromSubVoxelIdx(const Index& subvoxel_idx, Index& voxel_idx) const
{
    voxel_idx.x() = subvoxel_idx.x() >> voxel_depth;
    voxel_idx.y() = subvoxel_idx.y() >> voxel_depth;
    voxel_idx.z() = subvoxel_idx.z() >> voxel_depth;
}

inline void Map::getBlockIdxFromSubVoxelIdx(const Index& subvoxel_idx, Index& block_idx) const
{
    block_idx.x() = subvoxel_idx.x() >> block_depth;
    block_idx.y() = subvoxel_idx.y() >> block_depth;
    block_idx.z() = subvoxel_idx.z() >> block_depth;
}

inline void Map::getBlockIdxFromVoxelIdx(const Index& voxel_idx, Index& block_idx) const
{
    block_idx.x() = voxel_idx.x() >> voxel_to_block_depth;
    block_idx.y() = voxel_idx.y() >> voxel_to_block_depth;
    block_idx.z() = voxel_idx.z() >> voxel_to_block_depth;
}

inline void Map::getLocalSubVoxelLinearIdxFromSubvoxelIdx(const Index& subvoxel_idx, LinearIndex& local_subvoxel_linear_idx) const
{
    local_subvoxel_linear_idx = ((subvoxel_idx.x() & local_subvoxel_mask) << voxel_depth_doubled) + 
                                ((subvoxel_idx.y() & local_subvoxel_mask) << voxel_depth) + 
                                (subvoxel_idx.z() & local_subvoxel_mask);
}

inline void Map::getLocalVoxelLinearIdxFromVoxelIdx(const Index& voxel_idx, LinearIndex& local_voxel_linear_idx) const
{
    local_voxel_linear_idx =    ((voxel_idx.x() & local_voxel_mask) << voxel_to_block_depth_doubled) + 
                                ((voxel_idx.y() & local_voxel_mask) << voxel_to_block_depth) + 
                                (voxel_idx.z() & local_voxel_mask);
}
}
#endif