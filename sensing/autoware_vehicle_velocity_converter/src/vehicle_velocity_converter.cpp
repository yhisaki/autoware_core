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

#include "vehicle_velocity_converter.hpp"

#include <algorithm>
#include <string>

namespace autoware::vehicle_velocity_converter
{
VehicleVelocityConverter::VehicleVelocityConverter(const rclcpp::NodeOptions & options)
: rclcpp::Node("vehicle_velocity_converter", options),
  frame_id_(declare_parameter<std::string>("frame_id")),
  stddev_vx_(declare_parameter<double>("velocity_stddev_xx")),
  stddev_wz_(declare_parameter<double>("angular_velocity_stddev_zz")),
  speed_scale_factor_(declare_parameter<double>("speed_scale_factor")),
  enable_online_speed_scale_factor_calibration_(
    declare_parameter<bool>("enable_online_speed_scale_factor_calibration")),
  estimated_speed_scale_factor_(declare_parameter<double>("estimated_speed_scale_factor")),
  acceptable_speed_scale_factor_range_(
    declare_parameter<std::vector<double>>("acceptable_speed_scale_factor_range")),
  stop_speed_threshold_(declare_parameter<double>("stop_speed_threshold"))
{
  vehicle_report_sub_ = create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "velocity_status", rclcpp::QoS{100},
    std::bind(&VehicleVelocityConverter::callback_velocity_report, this, std::placeholders::_1));

  twist_with_covariance_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rclcpp::QoS{10});

  if (enable_online_speed_scale_factor_calibration_) {
    estimated_speed_scale_sub_ =
      PollingSubscriber<autoware_internal_debug_msgs::msg::Float32Stamped>::create_subscription(
        this, "estimated_speed_scale_factor");
    if (acceptable_speed_scale_factor_range_.size() != 2) {
      throw std::runtime_error("acceptable_speed_scale_factor_range must be a vector of size 2");
    }
    current_speed_scale_factor_pub_ =
      create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
        "debug/current_speed_scale_factor", rclcpp::QoS{10});
  }
}

void VehicleVelocityConverter::callback_velocity_report(
  const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
{
  if (msg->header.frame_id != frame_id_) {
    RCLCPP_WARN(get_logger(), "frame_id is not base_link.");
  }

  // set twist with covariance msg from vehicle report msg
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance_msg;
  twist_with_covariance_msg.header = msg->header;
  twist_with_covariance_msg.twist.twist.linear.x = msg->longitudinal_velocity * speed_scale_factor_;
  twist_with_covariance_msg.twist.twist.linear.y = msg->lateral_velocity;
  twist_with_covariance_msg.twist.twist.angular.z = msg->heading_rate;
  twist_with_covariance_msg.twist.covariance[0 + 0 * 6] = stddev_vx_ * stddev_vx_;
  twist_with_covariance_msg.twist.covariance[1 + 1 * 6] = 10000.0;
  twist_with_covariance_msg.twist.covariance[2 + 2 * 6] = 10000.0;
  twist_with_covariance_msg.twist.covariance[3 + 3 * 6] = 10000.0;
  twist_with_covariance_msg.twist.covariance[4 + 4 * 6] = 10000.0;
  twist_with_covariance_msg.twist.covariance[5 + 5 * 6] = stddev_wz_ * stddev_wz_;

  twist_with_covariance_pub_->publish(twist_with_covariance_msg);

  if (enable_online_speed_scale_factor_calibration_) {
    update_speed_scale_factor(twist_with_covariance_msg.twist.twist.linear.x);
  }
}

void VehicleVelocityConverter::update_speed_scale_factor(const double current_speed)
{
  auto data = estimated_speed_scale_sub_->take_data();
  estimated_speed_scale_factor_ = data->data;
  bool is_stopping = std::abs(current_speed) < stop_speed_threshold_;
  if (is_stopping) {
    speed_scale_factor_ = std::clamp(
      estimated_speed_scale_factor_, acceptable_speed_scale_factor_range_[0],
      acceptable_speed_scale_factor_range_[1]);
  }
  autoware_internal_debug_msgs::msg::Float32Stamped current_speed_scale_factor_msg;
  current_speed_scale_factor_msg.stamp = get_clock()->now();
  current_speed_scale_factor_msg.data = static_cast<float>(speed_scale_factor_);
  current_speed_scale_factor_pub_->publish(current_speed_scale_factor_msg);
}
}  // namespace autoware::vehicle_velocity_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::vehicle_velocity_converter::VehicleVelocityConverter)
