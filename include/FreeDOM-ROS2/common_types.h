//
// Created by ZhiangQi on 25-5-5.
//

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <Eigen/Eigen>

namespace freedom{
    enum class DynamicLevel : uint8_t
    {
        STATIC = 0,
        AGGRESSIVE_DYNAMIC = 1,
        MODERATE_DYNAMIC = 2,
        CONSERVATIVE_DYNAMIC = 3
    };

    enum Label
    {
        LABEL_ERROR = 0,
        LABEL_STATIC = 9,
        LABEL_DYNAMIC = 251
    };

    typedef Eigen::Vector3f Pointf;
    typedef Eigen::Vector3d Point;
    typedef Eigen::Vector3d PointBias;
    typedef Eigen::Vector3i Index;
    typedef Eigen::Vector3i IndexBias;
    typedef size_t LinearIndex;
    typedef std::vector<Point,Eigen::aligned_allocator<Point>> Points;
    typedef std::vector<Index,Eigen::aligned_allocator<Index>> Indices;
    typedef std::vector<LinearIndex> LinearIndices;

    class IndexHash
    {
    public:
        std::size_t operator()(const Index& v) const{
            return v.x()*73856093 ^ v.y()*19349663 ^ v.z()*83492791;
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    typedef std::unordered_map<Index,LinearIndex,IndexHash> Index2LinearIndexMap;
    typedef std::unordered_map<LinearIndex,LinearIndex> LinearIndex2LinearIndexMap;
}
#endif //COMMON_TYPES_H
