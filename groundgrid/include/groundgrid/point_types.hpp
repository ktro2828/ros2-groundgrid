//  Copyright (C) 2011, 2012 Austin Robot Technology
//
//  License: Modified BSD Software License Agreement
//
//  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
//

#ifndef GROUNDGRID__POINT_TYPES_HPP_
#define GROUNDGRID__POINT_TYPES_HPP_

#include <Eigen/Core>
#include <pcl/impl/point_types.hpp>

#include <Eigen/src/Core/util/ConfigureVectorization.h>
#include <pcl/point_types.h>

#include <cstdint>

namespace groundgrid
{
/**
 * @brief Simple 3D point with intensity and label.
 */
struct EIGEN_ALIGN16 PointXYZIL
{
  // Constructors
  PointXYZIL() : x(0.0f), y(0.0f), z(0.0f), intensity(0.0f), label(0) {}

  PointXYZIL(float _x, float _y, float _z, float _intensity, std::uint16_t _label)
  : x(_x), y(_y), z(_z), intensity(_intensity), label(_label)
  {
  }

  // Member variables
  float x;
  float y;
  float z;
  float intensity;
  uint16_t label;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Extended 3D point with intensity, ring number, distance, and azimuth.
 */
struct EIGEN_ALIGN16 PointXYZIDRA
{
  // Constructors
  PointXYZIDRA() : x(0.0f), y(0.0f), z(0.0f), intensity(0.0f), distance(0.0f), ring(0), azimuth(0)
  {
  }

  PointXYZIDRA(
    float _x, float _y, float _z, float _intensity, float _distance, std::uint16_t _ring,
    std::uint16_t _azimuth)
  : x(_x), y(_y), z(_z), intensity(_intensity), distance(_distance), ring(_ring), azimuth(_azimuth)
  {
  }

  // Member variables
  float x;
  float y;
  float z;
  float intensity;
  float distance;
  std::uint16_t ring;
  std::uint16_t azimuth;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};  // namespace groundgrid

POINT_CLOUD_REGISTER_POINT_STRUCT(
  groundgrid::PointXYZIL,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, label, label))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  groundgrid::PointXYZIDRA,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, distance, distance)(
    std::uint16_t, ring, ring)(std::uint16_t, azimuth, azimuth))

#endif  // GROUNDGRID__POINT_TYPES_HPP_
