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

#ifndef GROUNDGRID__CONFIG_HPP_
#define GROUNDGRID__CONFIG_HPP_

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>

#include <cstddef>
#include <vector>

namespace groundgrid
{
class GroundGridConfig
{
public:
  explicit GroundGridConfig(rclcpp::Node * node_ptr);

  int point_count_cell_variance_threshold;
  int max_ring;
  double ground_patch_detection_minimum_threshold;
  double distance_factor;
  double minimum_distance_factor;
  double minimum_point_height_threshold;
  double minimum_point_height_obstacle_threshold;
  double outlier_tolerance;
  double ground_patch_detection_minimum_point_count_threshold;
  double patch_size_change_distance;
  double occupied_cells_decrease_factor;
  double occupied_cells_point_count_factor;
  double min_outlier_detection_ground_confidence;
  size_t thread_count;

private:
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_params_res_;
};
}  // namespace groundgrid
#endif  // GROUNDGRID__CONFIG_HPP_
