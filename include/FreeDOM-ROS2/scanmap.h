//
// Created by ZhiangQi on 25-5-5.
//

#ifndef SCANMAP_H
#define SCANMAP_H
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <atomic>

#include <FreeDOM-ROS2/common_types.h>
#include <FreeDOM-ROS2/map.h>
#include <FreeDOM-ROS2/utils.h>

namespace freedom{
class ScanMap : public Map{
public:
    struct ScanMapConfig : public Config
    {
        double sensor_min_range;
        double sensor_max_range;
        double sensor_min_z;
        double sensor_max_z;
    };

    // 以voxel为单位保存的该voxel信息与其中的点的list
    struct ScanVoxel
    {
        Point center;
        Index voxel_idx;
        LinearIndex local_voxel_linear_idx;
        DynamicLevel dynamic_level = DynamicLevel::STATIC;
        std::vector<LinearIndex> points;        // 在LocalScanBlock中points的索引

        ScanVoxel() = default;

        ScanVoxel(const Point& point_,const Index& voxel_idx_,const LinearIndex& local_voxel_linear_idx_) :
            center(point_),voxel_idx(voxel_idx_),local_voxel_linear_idx(local_voxel_linear_idx_){}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    typedef std::vector<ScanVoxel,Eigen::aligned_allocator<ScanVoxel>> ScanVoxelList;

    // 以block为单位保存的该block信息与其中的scan voxel的map和list
    struct ScanBlock
    {
        Index block_idx;
        LinearIndex local_block_linear_idx; // 在local_blocks中的linear idx
        ScanVoxelList scan_voxels;     // 该block中所有scan voxel
        LinearIndex2LinearIndexMap local_voxel_linear_idx2scan_voxel_linear_idx;   // local voxel linear idx到scan voxel linear idx的hash map

        ScanBlock() = default;

        ScanBlock(const Index& block_idx_,const LinearIndex& local_block_linear_idx_) :
            block_idx(block_idx_),local_block_linear_idx(local_block_linear_idx_){}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    typedef std::vector<ScanBlock,Eigen::aligned_allocator<ScanBlock>> ScanBlockList;

    // 稠密储存的block单元，保存了到对应ScanBlock的linear idx和其中的point(为了并行插入point而不储存在ScanBlock内)
    struct LocalScanBlock
    {
        LinearIndex scan_block_linear_idx;  // 只有points不为空这个idx才有效
        Points points;

        LocalScanBlock() = default;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    typedef std::vector<LocalScanBlock,Eigen::aligned_allocator<LocalScanBlock>> LocalScanBlockGrid;

    ScanMap() : Map() {}
    void set_params(const ScanMapConfig& config);
    void reset();

    void build_scan_map(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform);

    // for visualization
    inline const Point& get_sensor_origin() const {return sensor_origin;}

    inline const ScanBlockList& get_scan_blocks() const {return scan_blocks;}
    inline ScanBlockList& get_scan_blocks() {return scan_blocks;}

    inline const LocalScanBlockGrid& get_local_blocks() const {return local_blocks;}

    // inline const ScanVoxel* have_scan_voxel(const Index& voxel_idx) const;
    inline ScanVoxel* have_scan_voxel(const Index& voxel_idx);

    inline const IndexBias& get_scan_map_min_idx() const {return scan_map.min_idx;}   // 返回scan map最小block idx
    inline const IndexBias& get_scan_map_max_idx() const {return scan_map.max_idx;}   // 返回scan map最大block idx
    inline const IndexBias& get_scan_map_idx_size() const {return scan_map.idx_size;} // 返回scan map在xyz的block数量
    inline const PointBias& get_scan_map_min() const {return scan_map.min;}     // 返回scan map最小点
    inline const PointBias& get_scan_map_max() const {return scan_map.max;}   // 返回scan map最大点
    inline const PointBias& get_scan_map_size() const {return scan_map.size;}   // 返回scan map在xyz的长度

private:
    void build_scan_blocks(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform);
    void build_scan_voxels();

    // params
    double sensor_min_range;
    double sensor_max_range;
    double sensor_min_range_squared;
    double sensor_max_range_squared;
    double sensor_min_z;
    double sensor_max_z;

    unsigned int scan_map_horizontal_block_num;
    unsigned int scan_map_vertical_block_num;
    unsigned int scan_map_horizontal_vertical_block_num_multiplied;
    unsigned int scan_map_total_block_num;

    IndexBias scan_map_center_min_bias;      // 从传感器所在block到最小与最大block的bias
    IndexBias scan_map_center_max_bias;
    IndexBias scan_map_idx_size;        // scan_map在xyz的block数,等于center_max_bias-center_min_bias+1
    PointBias scan_map_size;

    // variables
    Point sensor_origin;            // sensor所在位置

    // 稠密储存的scan block map
    LocalScanBlockGrid local_blocks;
    std::unique_ptr<std::vector<std::mutex>> block_mutexes;

    // 所有scan block的列表
    ScanBlockList scan_blocks;

    RoboCentricMap scan_map;
};

// inline const ScanMap::ScanVoxel* ScanMap::have_scan_voxel(const Index& voxel_idx) const
// {
//     ;
// }

inline ScanMap::ScanVoxel* ScanMap::have_scan_voxel(const Index& voxel_idx)
{
    Index block_idx;
    LinearIndex local_voxel_linear_idx;
    LinearIndex local_block_linear_idx;
    getBlockIdxFromVoxelIdx(voxel_idx,block_idx);
    getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);
    scan_map.getLocalBlockLinearIdxFromBlockIdx(block_idx,local_block_linear_idx);

    // 若没有points，说明没有对应scan voxel
    if(local_blocks[local_block_linear_idx].points.empty())
        return nullptr;

    ScanBlock& scan_block = scan_blocks[local_blocks[local_block_linear_idx].scan_block_linear_idx];
    auto it = scan_block.local_voxel_linear_idx2scan_voxel_linear_idx.find(local_voxel_linear_idx);
    if(it != scan_block.local_voxel_linear_idx2scan_voxel_linear_idx.end())
        return &scan_block.scan_voxels[it->second];
    else
        return nullptr;
}
}
#endif //SCANMAP_H
