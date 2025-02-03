// Copyright 2023 Dahlem Center for Machine Learning and Robotics, Freie
// Universität Berlin
//
// Redistribution and use in source and binary forms, with or without
// modification,
// are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this
// list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
// OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef GROUNDGRID__SEGMENTATION_HPP_
#define GROUNDGRID__SEGMENTATION_HPP_

#include "groundgrid/config.hpp"
#include "groundgrid/forward.hpp"
#include "groundgrid/point_types.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <utility>
#include <vector>

namespace groundgrid
{
class GroundSegmentation
{
public:
  explicit GroundSegmentation(const GroundGridConfig & config);

  void init(const size_t dimension, const float & resolution);

  [[nodiscard]] PCLPointPtr filterPointcloud(
    const PCLPointPtr cloud, const PointT & cloud_origin, const TransformStamped & map_to_base,
    GridMap & map);

private:
  void insertPointcloud(
    const PCLPointPtr cloud, const size_t start, const size_t end, const PointT & cloud_origin,
    std::vector<std::pair<size_t, GridMapIndex>> & point_index,
    std::vector<std::pair<size_t, GridMapIndex>> & ignored, std::vector<size_t> & outliers,
    GridMap & map);

  void detectGroundPatches(GridMap & grid_map, size_t section) const;

  template <int S>
  void detectGroundPatch(GridMap & grid_map, size_t i, size_t j) const;

  void spiralGroundInterpolation(GridMap & grid_map, const TransformStamped & map_to_base) const;

  void interpolateCell(GridMap & grid_map, const size_t x, const size_t y) const;

  GroundGridConfig config_;
  GridMapMatrix expected_points_;

  const float verticalPointAngDist = 0.00174532925 * 2;
  const float minDistSquared = 12.0f;
};
}  // namespace groundgrid
#endif  // GROUNDGRID__SEGMENTATION_HPP_
