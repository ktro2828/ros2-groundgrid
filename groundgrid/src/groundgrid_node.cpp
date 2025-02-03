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

#include "groundgrid/groundgrid_node.hpp"

#include "groundgrid/forward.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/convert.hpp>
#include <tf2/exceptions.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <functional>
#include <memory>

namespace groundgrid
{
GroundGridNode::GroundGridNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("groundgrid", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  {
    groundgrid_ptr_ = std::make_shared<GroundGrid>(this);
    ground_segmentation_ptr_ = std::make_shared<GroundSegmentation>(groundgrid_ptr_->config());
  }

  {
    odometry_sub_ = create_subscription<Odometry>(
      "~/input/odometry", rclcpp::QoS{1},
      std::bind(&GroundGridNode::onOdometry, this, std::placeholders::_1));
    pointcloud_sub_ = create_subscription<PointCloud2>(
      "~/input/pointcloud", rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&GroundGridNode::onPointcloud, this, std::placeholders::_1));
    pointcloud_pub_ =
      create_publisher<PointCloud2>("~/output/pointcloud", rclcpp::SensorDataQoS().keep_last(5));
  }
}

void GroundGridNode::onOdometry(const Odometry::ConstSharedPtr odometry_msg)
{
  grid_map_ptr_ = groundgrid_ptr_->update(odometry_msg);
}

void GroundGridNode::onPointcloud(const PointCloud2::ConstSharedPtr in_cloud_msg)
{
  PCLPointPtr cloud = std::make_shared<PCLPoint>();
  pcl::fromROSMsg(*in_cloud_msg, *cloud);

  if (!grid_map_ptr_) {
    RCLCPP_WARN(get_logger(), "GridMapPtr has not been initialized yet...");
    return;
  }

  TransformStamped map_to_base, cloud_origin;
  try {
    map_to_base = tf_buffer_.lookupTransform("map", "base_link", in_cloud_msg->header.stamp);
    cloud_origin =
      tf_buffer_.lookupTransform("map", in_cloud_msg->header.frame_id, in_cloud_msg->header.stamp);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_STREAM(get_logger(), "Tf2 transformation exception: " << e.what());
    return;
  }

  PointStamped origin;
  origin.header = in_cloud_msg->header;
  tf2::doTransform(origin, origin, cloud_origin);

  if (in_cloud_msg->header.frame_id != "map") {
    PCLPointPtr transformed_cloud = std::make_shared<PCLPoint>();
    transformed_cloud->header = cloud->header;
    transformed_cloud->header.frame_id = "map";
    transformed_cloud->points.reserve(cloud->points.size());

    TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_.lookupTransform(
        "map", in_cloud_msg->header.frame_id, in_cloud_msg->header.stamp);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_STREAM(get_logger(), "Tf2 transformation exception: " << e.what());
      return;
    }

    PointStamped in_point;
    in_point.header = in_cloud_msg->header;
    in_point.header.frame_id = "map";
    for (const auto & point : cloud->points) {
      in_point.point.x = point.x;
      in_point.point.y = point.y;
      in_point.point.z = point.z;

      tf2::doTransform(in_point, in_point, tf_stamped);

      PointT & transformed_point = transformed_cloud->points.emplace_back(point);
      transformed_point.x = in_point.point.x;
      transformed_point.y = in_point.point.y;
      transformed_point.z = in_point.point.z;
    }

    cloud = transformed_cloud;
  }

  PointT origin_point;
  origin_point.x = origin.point.x;
  origin_point.y = origin.point.y;
  origin_point.z = origin.point.z;
  const auto segment_cloud_ptr =
    ground_segmentation_ptr_->filterPointcloud(cloud, origin_point, map_to_base, *grid_map_ptr_);

  PointCloud2 out_cloud_msg;
  pcl::toROSMsg(*segment_cloud_ptr, out_cloud_msg);
  out_cloud_msg.header = in_cloud_msg->header;
  out_cloud_msg.header.frame_id = "map";
  pointcloud_pub_->publish(out_cloud_msg);
}
}  // namespace groundgrid

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(groundgrid::GroundGridNode)
