// Copyright 2021 TierIV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VEHICLE_VELOCITY_CONVERTER_HPP_
#define VEHICLE_VELOCITY_CONVERTER_HPP_

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <string>
#include <vector>

namespace autoware::vehicle_velocity_converter
{

template <typename T>
using PollingSubscriber = autoware_utils_rclcpp::InterProcessPollingSubscriber<T>;
class VehicleVelocityConverter : public rclcpp::Node
{
public:
  explicit VehicleVelocityConverter(const rclcpp::NodeOptions & options);

private:
  void callback_velocity_report(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg);

  void update_speed_scale_factor(const double current_speed);

  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_report_sub_;
  PollingSubscriber<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    estimated_speed_scale_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_covariance_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    current_speed_scale_factor_pub_;

  std::string frame_id_;
  double stddev_vx_;
  double stddev_wz_;
  double speed_scale_factor_;
  bool enable_online_speed_scale_factor_calibration_;
  double estimated_speed_scale_factor_;
  std::vector<double> acceptable_speed_scale_factor_range_;
  double stop_speed_threshold_;
};
}  // namespace autoware::vehicle_velocity_converter

#endif  // VEHICLE_VELOCITY_CONVERTER_HPP_
