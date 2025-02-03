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

#include "groundgrid/groundgrid.hpp"

#include "groundgrid/forward.hpp"

#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>

namespace groundgrid
{
GroundGrid::GroundGrid(rclcpp::Node * node_ptr)
: node_ptr_(node_ptr),
  config_(node_ptr),
  tf_buffer_(node_ptr->get_clock()),
  tf_listener_(tf_buffer_)
{
}

void GroundGrid::initGroundGrid(const Odometry::ConstSharedPtr in_odometry)
{
  PoseWithCovarianceStamped odometry_pose, map_pose;

  grid_map_ptr_ = std::make_shared<GridMap, const std::vector<std::string>>(
    {"points", "ground", "ground_patch", "minGroundHeight", "maxGroundHeight"});
  GridMap & map = *grid_map_ptr_;
  map.setFrameId("map");
  map.setGeometry(
    GridMapLength(mDimension, mDimension), mResolution,
    GridMapPosition(in_odometry->pose.pose.position.x, in_odometry->pose.pose.position.y));

  odometry_pose.pose = in_odometry->pose;
  odometry_pose.header = in_odometry->header;
  std::vector<grid_map::BufferRegion> damage;
  map.move(
    GridMapPosition(odometry_pose.pose.pose.position.x, odometry_pose.pose.pose.position.y),
    damage);
  grid_map::BufferRegion region(
    GridMapIndex(0, 0), map.getSize(), grid_map::BufferRegion::Quadrant(0));

  map["points"].setZero();
  map["ground"].setConstant(in_odometry->pose.pose.position.z);
  map["ground_patch"].setConstant(0.0000001);
  map["minGroundHeight"].setConstant(100.0);
  map["maxGroundHeight"].setConstant(-100.0);

  last_pose_ = odometry_pose;
}

GridMapPtr GroundGrid::update(const Odometry::ConstSharedPtr in_odometry)
{
  if (!grid_map_ptr_) {
    initGroundGrid(in_odometry);
    return grid_map_ptr_;
  }

  grid_map::GridMap & map = *grid_map_ptr_;

  PoseWithCovarianceStamped pose_diff;
  pose_diff.pose.pose.position.x =
    in_odometry->pose.pose.position.x - last_pose_.pose.pose.position.x;
  pose_diff.pose.pose.position.y =
    in_odometry->pose.pose.position.y - last_pose_.pose.pose.position.y;
  std::vector<grid_map::BufferRegion> damage;
  map.move(
    GridMapPosition(in_odometry->pose.pose.position.x, in_odometry->pose.pose.position.y), damage);

  // static so if the new transform is not yet available, we can use the last
  // one
  static TransformStamped base_to_map;

  try {
    base_to_map = tf_buffer_.lookupTransform("base_link", "map", in_odometry->header.stamp);
  } catch (tf2::LookupException & e) {
    // potentially degraded performance
    RCLCPP_WARN_STREAM(node_ptr_->get_logger(), "no transform? -> error: " << e.what());
  } catch (tf2::ExtrapolationException & e) {
    // can happen when new transform has not yet been published, we can use the
    // old one instead
    RCLCPP_DEBUG_STREAM(
      node_ptr_->get_logger(), "need to extrapolate a transform? -> error: " << e.what());
  }

  PointStamped ps;
  ps.header = in_odometry->header;
  ps.header.frame_id = "map";
  GridMapPosition pos;

  for (auto region : damage) {
    for (auto it = grid_map::SubmapIterator(map, region); !it.isPastEnd(); ++it) {
      const auto & idx = *it;

      map.getPosition(idx, pos);
      ps.point.x = pos(0);
      ps.point.y = pos(1);
      ps.point.z = 0;
      tf2::doTransform(ps, ps, base_to_map);
      map.at("ground", idx) = -ps.point.z;
      map.at("ground_patch", idx) = 0.0;
    }
  }

  // We haven't moved so we have nothing to do
  if (damage.empty()) {
    return grid_map_ptr_;
  }

  last_pose_.pose = in_odometry->pose;
  last_pose_.header = in_odometry->header;

  map.convertToDefaultStartIndex();
  return grid_map_ptr_;
}
}  // namespace groundgrid
