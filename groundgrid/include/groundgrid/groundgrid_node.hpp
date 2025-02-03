// Copyright 2025 Kotaro Uetake.
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

#ifndef GROUNDGRID__GROUNDGRID_NODE_HPP_
#define GROUNDGRID__GROUNDGRID_NODE_HPP_

#include "groundgrid/forward.hpp"
#include "groundgrid/groundgrid.hpp"
#include "groundgrid/segmentation.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace groundgrid
{
class GroundGridNode : public rclcpp::Node
{
public:
  explicit GroundGridNode(const rclcpp::NodeOptions & options);

private:
  void onOdometry(const Odometry::ConstSharedPtr odometry_msg);

  void onPointcloud(const PointCloud2::ConstSharedPtr in_cloud_msg);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  GroundGridPtr groundgrid_ptr_;
  GroundSegmentationPtr ground_segmentation_ptr_;
  GridMapPtr grid_map_ptr_;

  rclcpp::Subscription<Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_pub_;
};
}  // namespace groundgrid
#endif  // GROUNDGRID__GROUNDGRID_NODE_HPP_
