#ifndef _POINT_TYPES_H
#define _POINT_TYPES_H

#include <pcl/point_types.h>

namespace ouster_pcl
{
struct Point
{
  PCL_ADD_POINT4D;
  float         intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;

  double getTimestamp() const{return static_cast<double>(t);}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}
EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_pcl::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint32_t, t, t)
                                  (std::uint16_t, reflectivity, reflectivity))

namespace velodyne_pcl
{
struct Point
{
  PCL_ADD_POINT4D;
  float         intensity;
  std::uint16_t ring;
  float         time;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}
EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pcl::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (float, time, time))

namespace robosense_pcl
{
struct Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  double timestamp;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}
EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_pcl::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (double, timestamp, timestamp))

namespace evaluate_pcl
{
struct Point
{
  PCL_ADD_POINT4D;
  std::uint16_t label;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}
EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(evaluate_pcl::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (std::uint16_t, label, label))

#endif