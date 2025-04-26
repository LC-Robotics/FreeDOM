#include "freedom/map.h"

namespace freedom{
void Map::set_params(const Config& config)
{
    voxel_depth = config.voxel_depth;
    block_depth = config.block_depth;
    voxel_to_block_depth = block_depth - voxel_depth;

    voxel_depth_doubled = 2 * voxel_depth;
    voxel_to_block_depth_doubled = 2 * voxel_to_block_depth;

    sub_voxel_size = config.sub_voxel_size;
    voxel_size = sub_voxel_size * pow(2,voxel_depth);
    block_size = sub_voxel_size * pow(2,block_depth);

    sub_voxel_size_inv = 1 / sub_voxel_size;
    voxel_size_inv = 1 / voxel_size;
    block_size_inv = 1 / block_size;

    subvoxel2voxelMultiples_ = pow(2,voxel_depth);
    subvoxel2voxelMultiples_squared_ = subvoxel2voxelMultiples_ * subvoxel2voxelMultiples_;
    subvoxel2voxelMultiples_cubed_ = subvoxel2voxelMultiples_ * subvoxel2voxelMultiples_ * subvoxel2voxelMultiples_;

    voxel2blockMultiples_ = pow(2,voxel_to_block_depth);
    voxel2blockMultiples_squared_ = voxel2blockMultiples_ * voxel2blockMultiples_;
    voxel2blockMultiples_cubed_ = voxel2blockMultiples_ * voxel2blockMultiples_ * voxel2blockMultiples_;

    local_subvoxel_mask = subvoxel2voxelMultiples_ - 1;
    local_voxel_mask = voxel2blockMultiples_ - 1;

    enable_local_map = config.enable_local_map;
    if(enable_local_map)
        local_map.set_params(config.local_map_range,config.local_map_min_z,config.local_map_max_z,block_size);

    num_threads = config.num_threads;
}

void Map::RoboCentricMap::set_params(const double& range,const double& min_z,const double& max_z,const double& block_size_)
{
    block_size = block_size_;
    block_size_inv = 1/block_size_;
    int block_bias_horizontal = std::ceil(range/block_size + 0.25);
    int block_bias_vertical_min = std::floor(min_z/block_size - 0.25);
    int block_bias_vertical_max = std::ceil(max_z/block_size + 0.25);
    horizontal_block_num = 2 * block_bias_horizontal + 1;

    vertical_block_num = block_bias_vertical_max - block_bias_vertical_min + 1;
    horizontal_vertical_block_num_multiplied = horizontal_block_num * vertical_block_num;
    total_block_num = horizontal_block_num * horizontal_block_num * vertical_block_num;

    center_min_bias = IndexBias(-block_bias_horizontal,-block_bias_horizontal,block_bias_vertical_min);
    center_max_bias = IndexBias( block_bias_horizontal, block_bias_horizontal,block_bias_vertical_max);
    idx_size = IndexBias( horizontal_block_num , horizontal_block_num ,vertical_block_num);
    size = block_size * idx_size.cast<double>();

    center = Index(0,0,0);
    min_idx = center + center_min_bias;
    max_idx = center + center_max_bias + IndexBias(1,1,1);
    min = block_size * min_idx.cast<double>();
    max = block_size * max_idx.cast<double>();
}

void Map::RoboCentricMap::setLocalMapBound(const Point& origin)
{
    if((origin.x() < (center.x() - 0.24)*block_size || origin.x() > (center.x() + 1.24)*block_size ||
        origin.y() < (center.y() - 0.24)*block_size || origin.y() > (center.y() + 1.24)*block_size ||
        origin.z() < (center.z() - 0.24)*block_size || origin.z() > (center.z() + 1.24)*block_size))
    {
        center = Index(std::floor(origin.x()*block_size_inv),std::floor(origin.y()*block_size_inv),std::floor(origin.z()*block_size_inv));
        min_idx = center + center_min_bias;
        max_idx = center + center_max_bias + IndexBias(1,1,1);
        min = block_size * min_idx.cast<double>();
        max = block_size * max_idx.cast<double>();
    }
}
}

