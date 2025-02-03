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

#ifndef GROUNDGRID__GROUNDGRID_HPP_
#define GROUNDGRID__GROUNDGRID_HPP_

#include "groundgrid/config.hpp"
#include "groundgrid/forward.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace groundgrid
{
class GroundGrid
{
public:
  explicit GroundGrid(rclcpp::Node * node_ptr);

  void initGroundGrid(const Odometry::ConstSharedPtr in_odometry);

  [[nodiscard]] GridMapPtr update(const Odometry::ConstSharedPtr in_odometry);

  [[nodiscard]] const GroundGridConfig & config() const noexcept { return config_; }

  const float mResolution = .33f;
  const float mDimension = 120.0f;

private:
  rclcpp::Node * node_ptr_;
  /// dynamic config attribute
  GroundGridConfig config_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double detection_radius_ = 60.0;
  GridMapPtr grid_map_ptr_;
  TransformStamped tf_position_, tf_lux_, tf_utm_, tf_map_;
  PoseWithCovarianceStamped last_pose_;
};
}  // namespace groundgrid
#endif  // GROUNDGRID__GROUNDGRID_HPP_
