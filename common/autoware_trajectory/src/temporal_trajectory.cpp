// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory/temporal_trajectory.hpp"

#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/threshold.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{
namespace
{
double to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return rclcpp::Duration(duration).seconds();
}

builtin_interfaces::msg::Duration to_duration_msg(const double seconds)
{
  return rclcpp::Duration::from_seconds(seconds);
}

void set_zero_velocity(autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  point.longitudinal_velocity_mps = 0.0F;
}

bool has_same_time(const double lhs, const double rhs)
{
  return std::abs(lhs - rhs) <= k_epsilon;
}
}  // namespace

TemporalTrajectory::TemporalTrajectory()
{
  Builder::defaults(this);
}

TemporalTrajectory::TemporalTrajectory(const TemporalTrajectory & rhs)
: spatial_trajectory_(rhs.spatial_trajectory_),
  time_to_distance_(rhs.time_to_distance_ ? rhs.time_to_distance_->clone() : nullptr),
  time_bases_(rhs.time_bases_),
  distance_bases_(rhs.distance_bases_)
{
}

TemporalTrajectory & TemporalTrajectory::operator=(const TemporalTrajectory & rhs)
{
  if (this != &rhs) {
    spatial_trajectory_ = rhs.spatial_trajectory_;
    time_to_distance_ = rhs.time_to_distance_ ? rhs.time_to_distance_->clone() : nullptr;
    time_bases_ = rhs.time_bases_;
    distance_bases_ = rhs.distance_bases_;
  }
  return *this;
}

interpolator::InterpolationResult TemporalTrajectory::build(const std::vector<PointType> & points)
{
  if (!time_to_distance_) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"time_to_distance interpolator is nullptr"});
  }

  if (const auto result = spatial_trajectory_.build(points); !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"failed to interpolate temporal spatial trajectory"} +
      result.error());
  }

  time_bases_.clear();
  time_bases_.reserve(points.size());
  for (const auto & point : points) {
    time_bases_.push_back(to_seconds(point.time_from_start));
  }

  if (time_bases_.empty()) {
    return tl::unexpected(interpolator::InterpolationFailure{"cannot interpolate 0 size points"});
  }
  if (!std::is_sorted(time_bases_.begin(), time_bases_.end())) {
    return tl::unexpected(
      interpolator::InterpolationFailure{"time_from_start must be sorted in ascending order"});
  }

  distance_bases_ = spatial_trajectory_.get_underlying_bases();

  std::vector<double> sanitized_time_bases;
  std::vector<double> sanitized_distance_bases;
  sanitized_time_bases.reserve(time_bases_.size());
  sanitized_distance_bases.reserve(distance_bases_.size());

  for (size_t i = 0; i < time_bases_.size(); ++i) {
    if (
      !sanitized_time_bases.empty() &&
      std::abs(time_bases_[i] - sanitized_time_bases.back()) <= k_epsilon) {
      if (std::abs(distance_bases_[i] - sanitized_distance_bases.back()) > k_epsilon) {
        return tl::unexpected(interpolator::InterpolationFailure{
          "time_from_start contains duplicate timestamps with different distances"});
      }
      sanitized_time_bases.back() = time_bases_[i];
      sanitized_distance_bases.back() = distance_bases_[i];
      continue;
    }
    sanitized_time_bases.push_back(time_bases_[i]);
    sanitized_distance_bases.push_back(distance_bases_[i]);
  }

  if (const auto result = time_to_distance_->build(sanitized_time_bases, sanitized_distance_bases);
      !result) {
    return tl::unexpected(
      interpolator::InterpolationFailure{
        "failed to interpolate TemporalTrajectory::time_to_distance"} +
      result.error());
  }

  return interpolator::InterpolationSuccess{};
}

double TemporalTrajectory::length() const
{
  return spatial_trajectory_.length();
}

double TemporalTrajectory::duration() const
{
  return time_bases_.empty() ? 0.0 : time_bases_.back() - time_bases_.front();
}

std::vector<double> TemporalTrajectory::get_underlying_time_bases() const
{
  return time_bases_;
}

std::vector<double> TemporalTrajectory::get_underlying_distance_bases() const
{
  return distance_bases_;
}

TemporalTrajectory::PointType TemporalTrajectory::compute_from_time(const double t) const
{
  const auto t_clamped = clamp_time(t, true);
  auto point = spatial_trajectory_.compute(time_to_distance_->compute(t_clamped));
  point.time_from_start = to_duration_msg(t_clamped);
  return point;
}

std::vector<TemporalTrajectory::PointType> TemporalTrajectory::compute_from_time(
  const std::vector<double> & ts) const
{
  std::vector<PointType> points;
  points.reserve(ts.size());
  for (const auto t : ts) {
    points.push_back(compute_from_time(t));
  }
  return points;
}

TemporalTrajectory::PointType TemporalTrajectory::compute_from_distance(const double s) const
{
  auto point = spatial_trajectory_.compute(s);
  const auto t = distance_to_time(s);
  if (t.has_value()) {
    point.time_from_start = to_duration_msg(*t);
  }
  return point;
}

std::vector<TemporalTrajectory::PointType> TemporalTrajectory::compute_from_distance(
  const std::vector<double> & ss) const
{
  std::vector<PointType> points;
  points.reserve(ss.size());
  for (const auto s : ss) {
    points.push_back(compute_from_distance(s));
  }
  return points;
}

double TemporalTrajectory::time_to_distance(const double t) const
{
  return time_to_distance_->compute(clamp_time(t, true));
}

std::vector<double> TemporalTrajectory::time_to_distance(const std::vector<double> & ts) const
{
  std::vector<double> distances;
  distances.reserve(ts.size());
  for (const auto t : ts) {
    distances.push_back(time_to_distance(t));
  }
  return distances;
}

std::optional<double> TemporalTrajectory::distance_to_time(const double s) const
{
  const auto s_clamped = std::clamp(s, 0.0, length());

  if (
    distance_bases_.empty() || time_bases_.empty() ||
    distance_bases_.size() != time_bases_.size()) {
    return std::nullopt;
  }

  for (size_t i = 0; i + 1 < distance_bases_.size(); ++i) {
    const auto s0 = distance_bases_[i];
    const auto s1 = distance_bases_[i + 1];
    if (s_clamped < s0 - k_epsilon || s_clamped > s1 + k_epsilon) {
      continue;
    }

    const auto t0 = time_bases_[i];
    const auto t1 = time_bases_[i + 1];
    if (std::abs(s1 - s0) <= k_epsilon) {
      return t0;
    }

    const auto ratio = (s_clamped - s0) / (s1 - s0);
    return t0 + ratio * (t1 - t0);
  }

  if (std::abs(s_clamped - distance_bases_.back()) <= k_epsilon) {
    return time_bases_.back();
  }

  return std::nullopt;
}

std::vector<TemporalTrajectory::PointType> TemporalTrajectory::restore(
  const size_t min_points) const
{
  std::vector<double> time_samples = time_bases_;
  if (min_points > 1 && time_samples.size() < min_points && !time_samples.empty()) {
    const auto step = duration() / static_cast<double>(min_points - 1);
    time_samples.clear();
    time_samples.reserve(min_points);
    for (size_t i = 0; i < min_points; ++i) {
      time_samples.push_back(time_bases_.front() + step * static_cast<double>(i));
    }
  }
  return compute_from_time(time_samples);
}

void TemporalTrajectory::set_stopline(const double arc_length)
{
  const auto stop_time = distance_to_time(arc_length);
  if (!stop_time.has_value()) {
    return;
  }

  auto points = compute_from_time(time_bases_);
  auto stop_point = compute_from_distance(arc_length);
  set_zero_velocity(stop_point);

  const auto insert_index = static_cast<size_t>(std::distance(
    time_bases_.begin(), std::lower_bound(time_bases_.begin(), time_bases_.end(), *stop_time)));

  if (insert_index == time_bases_.size() || !has_same_time(time_bases_[insert_index], *stop_time)) {
    points.insert(points.begin() + static_cast<std::ptrdiff_t>(insert_index), stop_point);
  } else {
    points[insert_index] = stop_point;
  }

  for (size_t i = insert_index; i < points.size(); ++i) {
    points[i].pose = stop_point.pose;
    set_zero_velocity(points[i]);
  }

  (void)build(points);
}

void TemporalTrajectory::set_stopline(const double arc_length, const double time)
{
  const auto stop_time = distance_to_time(arc_length);
  if (!stop_time.has_value()) {
    return;
  }

  const auto target_time = std::max(time, *stop_time);
  auto points = compute_from_time(time_bases_);
  auto stop_point = compute_from_distance(arc_length);
  set_zero_velocity(stop_point);

  const auto insert_index = static_cast<size_t>(std::distance(
    time_bases_.begin(), std::lower_bound(time_bases_.begin(), time_bases_.end(), *stop_time)));

  if (insert_index == time_bases_.size() || !has_same_time(time_bases_[insert_index], *stop_time)) {
    points.insert(points.begin() + static_cast<std::ptrdiff_t>(insert_index), stop_point);
  } else {
    points[insert_index] = stop_point;
  }

  const auto stop_duration = target_time - *stop_time;
  if (stop_duration > k_epsilon) {
    auto delayed_stop_point = stop_point;
    delayed_stop_point.time_from_start = to_duration_msg(target_time);
    points.insert(
      points.begin() + static_cast<std::ptrdiff_t>(insert_index + 1), delayed_stop_point);

    for (size_t i = insert_index + 2; i < points.size(); ++i) {
      const auto shifted_time = to_seconds(points[i].time_from_start) + stop_duration;
      points[i].time_from_start = to_duration_msg(shifted_time);
    }
  }

  (void)build(points);
}

const TemporalTrajectory::SpatialTrajectory & TemporalTrajectory::spatial_trajectory() const
{
  return spatial_trajectory_;
}

double TemporalTrajectory::clamp_time(const double t, const bool show_warning) const
{
  if (time_bases_.empty()) {
    return t;
  }

  constexpr double eps = 1e-5;
  const auto start = time_bases_.front();
  const auto end = time_bases_.back();
  if (show_warning && (t < start - eps || t > end + eps)) {
    RCLCPP_WARN(
      rclcpp::get_logger("TemporalTrajectory"),
      "The time %f is out of the trajectory duration range [%f, %f]", t, start, end);
  }
  return std::clamp(t, start, end);
}

TemporalTrajectory::Builder::Builder() : trajectory_(std::make_unique<TemporalTrajectory>())
{
  defaults(trajectory_.get());
}

void TemporalTrajectory::Builder::defaults(TemporalTrajectory * trajectory)
{
  trajectory->time_to_distance_ = std::make_shared<interpolator::Linear>();
}

tl::expected<TemporalTrajectory, interpolator::InterpolationFailure>
TemporalTrajectory::Builder::build(const std::vector<PointType> & points)
{
  const auto spatial_trajectory_result = spatial_trajectory_builder_.build(points);
  if (!spatial_trajectory_result) {
    return tl::unexpected(spatial_trajectory_result.error());
  }

  trajectory_->spatial_trajectory_ = spatial_trajectory_result.value();
  if (const auto result = trajectory_->build(points); !result) {
    return tl::unexpected(result.error());
  }
  return std::move(*trajectory_);
}

}  // namespace autoware::experimental::trajectory
