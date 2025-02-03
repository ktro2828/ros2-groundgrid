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

#include "groundgrid/config.hpp"

#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/parameter.hpp>

#include <cstddef>
#include <functional>
#include <vector>

namespace groundgrid
{
GroundGridConfig::GroundGridConfig(rclcpp::Node * node_ptr)
{
  {
    point_count_cell_variance_threshold =
      node_ptr->declare_parameter<int>("point_count_cell_variance_threshold");
    max_ring = node_ptr->declare_parameter<int>("max_ring");
    ground_patch_detection_minimum_threshold =
      node_ptr->declare_parameter<double>("ground_patch_detection_minimum_threshold");
    distance_factor = node_ptr->declare_parameter<double>("distance_factor");
    minimum_distance_factor = node_ptr->declare_parameter<double>("minimum_distance_factor");
    minimum_point_height_threshold =
      node_ptr->declare_parameter<double>("minimum_point_height_threshold");
    minimum_point_height_obstacle_threshold =
      node_ptr->declare_parameter<double>("minimum_point_height_obstacle_threshold");
    outlier_tolerance = node_ptr->declare_parameter<double>("outlier_tolerance");
    ground_patch_detection_minimum_point_count_threshold =
      node_ptr->declare_parameter<double>("ground_patch_detection_minimum_point_count_threshold");
    patch_size_change_distance = node_ptr->declare_parameter<double>("patch_size_change_distance");
    occupied_cells_decrease_factor =
      node_ptr->declare_parameter<double>("occupied_cells_decrease_factor");
    occupied_cells_point_count_factor =
      node_ptr->declare_parameter<double>("occupied_cells_point_count_factor");
    min_outlier_detection_ground_confidence =
      node_ptr->declare_parameter<double>("min_outlier_detection_ground_confidence");
    thread_count = static_cast<size_t>(node_ptr->declare_parameter<int>("thread_count"));
  }

  set_params_res_ = node_ptr->add_on_set_parameters_callback(
    std::bind(&GroundGridConfig::onParameter, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult GroundGridConfig::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    const auto & name = param.get_name();
    if (name == "point_count_cell_variance_threshold") {
      point_count_cell_variance_threshold = param.as_int();
    } else if (name == "max_ring") {
      max_ring = param.as_int();
    } else if (name == "ground_patch_detection_minimum_threshold") {
      ground_patch_detection_minimum_threshold = param.as_double();
    } else if (name == "distance_factor") {
      distance_factor = param.as_double();
    } else if (name == "minimum_distance_factor") {
      minimum_distance_factor = param.as_double();
    } else if (name == "minimum_point_height_threshold") {
      minimum_point_height_threshold = param.as_double();
    } else if (name == "minimum_point_height_height_obstacle_threshold") {
      minimum_point_height_obstacle_threshold = param.as_double();
    } else if (name == "outlier_tolerance") {
      outlier_tolerance = param.as_double();
    } else if (name == "ground_patch_detection_minimum_point_count_threshold") {
      ground_patch_detection_minimum_point_count_threshold = param.as_double();
    } else if (name == "patch_size_change_distance") {
      patch_size_change_distance = param.as_double();
    } else if (name == "occupied_cells_decrease_factor") {
      occupied_cells_decrease_factor = param.as_double();
    } else if (name == "occupied_cells_point_count_factor") {
      occupied_cells_point_count_factor = param.as_double();
    } else if (name == "min_outlier_detection_ground_confidence") {
      min_outlier_detection_ground_confidence = param.as_double();
    } else if (name == "thread_count") {
      thread_count = static_cast<size_t>(param.as_int());
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}
}  // namespace groundgrid
