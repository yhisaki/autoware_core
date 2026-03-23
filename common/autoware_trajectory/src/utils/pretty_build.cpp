// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory/utils/pretty_build.hpp"

#include <rclcpp/duration.hpp>

#include <string>
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

template <typename PointType>
std::vector<double> extract_times(const std::vector<PointType> & points)
{
  std::vector<double> times;
  times.reserve(points.size());
  for (const auto & point : points) {
    times.push_back(to_seconds(point.time_from_start));
  }
  return times;
}

template <typename PointType>
void assign_times(
  std::vector<PointType> & points, const std::vector<double> & original_bases,
  const std::vector<double> & original_times, const std::vector<double> & new_bases)
{
  interpolator::Linear time_interpolator;
  const auto result = time_interpolator.build(original_bases, original_times);
  if (!result) {
    return;
  }

  for (size_t i = 0; i < points.size(); ++i) {
    points[i].time_from_start = to_duration_msg(time_interpolator.compute(new_bases[i]));
  }
}

tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string>
populate3_temporal(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs)
{
  const auto populated = detail::populate3(inputs);
  if (!populated) {
    return tl::unexpected(populated.error());
  }
  if (inputs.size() >= 3) {
    return populated.value();
  }

  auto points = populated.value();
  const auto original_bases = std::vector<double>{
    0.0,
    autoware_utils_geometry::calc_distance3d(inputs[0].pose.position, inputs[1].pose.position)};
  const auto new_bases =
    std::vector<double>{0.0, original_bases.back() / 2.0, original_bases.back()};
  assign_times(points, original_bases, extract_times(inputs), new_bases);
  return points;
}

tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string>
populate4_temporal(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs)
{
  if (inputs.size() >= 4) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected(std::string("cannot populate4() from less than 1 points!"));
  }

  const auto try_inputs3 = populate3_temporal(inputs);
  if (!try_inputs3) {
    return tl::unexpected(try_inputs3.error());
  }
  const auto & inputs3 = inputs.size() == 2 ? try_inputs3.value() : inputs;

  using Builder = typename Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>::Builder;
  const auto interpolation_result = Builder{}
                                      .set_xy_interpolator<interpolator::Linear>()
                                      .set_z_interpolator<interpolator::Linear>()
                                      .build(inputs3);
  if (!interpolation_result) {
    return tl::unexpected(std::string("failed Linear interpolation in populate4()!"));
  }

  const auto & interpolation = interpolation_result.value();
  const auto new_bases =
    detail::insert_middle_into_largest_interval(interpolation.get_underlying_bases());
  auto points = interpolation.compute(new_bases);
  assign_times(points, interpolation.get_underlying_bases(), extract_times(inputs3), new_bases);
  return points;
}

tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string>
populate5_temporal(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs)
{
  if (inputs.size() >= 5) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected(std::string("cannot populate5() from less than 1 points!"));
  }

  const auto try_inputs4 = populate4_temporal(inputs);
  if (!try_inputs4) {
    return tl::unexpected(try_inputs4.error());
  }
  const auto & inputs4 = inputs.size() == 4 ? inputs : try_inputs4.value();

  using Builder = typename Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>::Builder;
  const auto interpolation_result = Builder{}.build(inputs4);
  if (!interpolation_result) {
    return tl::unexpected(std::string("failed Linear interpolation in populate4()!"));
  }

  const auto & interpolation = interpolation_result.value();
  const auto new_bases =
    detail::insert_middle_into_largest_interval(interpolation.get_underlying_bases());
  auto points = interpolation.compute(new_bases);
  assign_times(points, interpolation.get_underlying_bases(), extract_times(inputs4), new_bases);
  return points;
}
}  // namespace

namespace detail
{

tl::expected<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate3(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs)
{
  if (inputs.size() >= 3) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected(std::string("cannot populate3() from less than 1 points!"));
  }

  const auto & p1 =
    get_geometry_msgs_pose<autoware_internal_planning_msgs::msg::PathPointWithLaneId>(inputs.at(0));
  const auto & pos1 = p1.position;
  const auto & p2 =
    get_geometry_msgs_pose<autoware_internal_planning_msgs::msg::PathPointWithLaneId>(inputs.at(1));
  const auto & pos2 = p2.position;
  const auto l = autoware_utils_geometry::calc_distance3d(pos1, pos2);

  const auto quat_result =
    interpolator::SphericalLinear::Builder{}
      .set_bases(std::vector<double>{0.0, l})
      .set_values(std::vector<geometry_msgs::msg::Quaternion>({p1.orientation, p2.orientation}))
      .build();
  // LCOV_EXCL_START
  if (!quat_result) {
    // this never happens because two values are given
    return tl::unexpected(std::string("failed to interpolate orientation"));
  }
  // LCOV_EXCL_END

  const geometry_msgs::msg::Point mid_position = geometry_msgs::build<geometry_msgs::msg::Point>()
                                                   .x((pos1.x + pos2.x) / 2.0)
                                                   .y((pos1.y + pos2.y) / 2.0)
                                                   .z((pos1.z + pos2.z) / 2.0);
  const auto mid_quat = quat_result->compute(l / 2.0);

  autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
  point.point.pose.position = mid_position;
  point.point.pose.orientation = mid_quat;
  point.point.longitudinal_velocity_mps = inputs.at(0).point.longitudinal_velocity_mps;
  point.point.lateral_velocity_mps = inputs.at(0).point.lateral_velocity_mps;
  point.point.heading_rate_rps = inputs.at(0).point.heading_rate_rps;
  point.lane_ids = inputs.at(0).lane_ids;

  return std::vector{inputs[0], point, inputs[1]};
}

tl::expected<std::vector<autoware_planning_msgs::msg::PathPoint>, std::string> populate3(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & inputs)
{
  if (inputs.size() >= 3) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected(std::string("cannot populate3() from less than 1 points!"));
  }

  const auto & p1 = get_geometry_msgs_pose<autoware_planning_msgs::msg::PathPoint>(inputs.at(0));
  const auto & pos1 = p1.position;
  const auto & p2 = get_geometry_msgs_pose<autoware_planning_msgs::msg::PathPoint>(inputs.at(1));
  const auto & pos2 = p2.position;
  const auto l = autoware_utils_geometry::calc_distance3d(pos1, pos2);

  const auto quat_result =
    interpolator::SphericalLinear::Builder{}
      .set_bases(std::vector<double>{0.0, l})
      .set_values(std::vector<geometry_msgs::msg::Quaternion>({p1.orientation, p2.orientation}))
      .build();
  // LCOV_EXCL_START
  if (!quat_result) {
    // this never happens because two values are given
    return tl::unexpected(std::string("failed to interpolate orientation"));
  }
  // LCOV_EXCL_END

  const geometry_msgs::msg::Point mid_position = geometry_msgs::build<geometry_msgs::msg::Point>()
                                                   .x((pos1.x + pos2.x) / 2.0)
                                                   .y((pos1.y + pos2.y) / 2.0)
                                                   .z((pos1.z + pos2.z) / 2.0);
  const auto mid_quat = quat_result->compute(l / 2.0);

  autoware_planning_msgs::msg::PathPoint point;
  point.pose.position = mid_position;
  point.pose.orientation = mid_quat;
  point.longitudinal_velocity_mps = inputs.at(0).longitudinal_velocity_mps;
  point.lateral_velocity_mps = inputs.at(0).lateral_velocity_mps;
  point.heading_rate_rps = inputs.at(0).heading_rate_rps;

  return std::vector{inputs[0], point, inputs[1]};
}

tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string> populate3(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs)
{
  if (inputs.size() >= 3) {
    return inputs;
  }
  if (inputs.size() < 2) {
    return tl::unexpected(std::string("cannot populate3() from less than 1 points!"));
  }

  const auto & p1 =
    get_geometry_msgs_pose<autoware_planning_msgs::msg::TrajectoryPoint>(inputs.at(0));
  const auto & pos1 = p1.position;
  const auto & p2 =
    get_geometry_msgs_pose<autoware_planning_msgs::msg::TrajectoryPoint>(inputs.at(1));
  const auto & pos2 = p2.position;
  const auto l = autoware_utils_geometry::calc_distance3d(pos1, pos2);

  const auto quat_result =
    interpolator::SphericalLinear::Builder{}
      .set_bases(std::vector<double>{0.0, l})
      .set_values(std::vector<geometry_msgs::msg::Quaternion>({p1.orientation, p2.orientation}))
      .build();
  // LCOV_EXCL_START
  if (!quat_result) {
    // this never happens because two values are given
    return tl::unexpected(std::string("failed to interpolate orientation"));
  }
  // LCOV_EXCL_END

  const geometry_msgs::msg::Point mid_position = geometry_msgs::build<geometry_msgs::msg::Point>()
                                                   .x((pos1.x + pos2.x) / 2.0)
                                                   .y((pos1.y + pos2.y) / 2.0)
                                                   .z((pos1.z + pos2.z) / 2.0);
  const auto mid_quat = quat_result->compute(l / 2.0);

  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose.position = mid_position;
  point.pose.orientation = mid_quat;
  point.longitudinal_velocity_mps = inputs.at(0).longitudinal_velocity_mps;
  point.lateral_velocity_mps = inputs.at(0).lateral_velocity_mps;
  point.heading_rate_rps = inputs.at(0).heading_rate_rps;
  point.acceleration_mps2 = inputs.at(0).acceleration_mps2;
  point.front_wheel_angle_rad = inputs.at(0).front_wheel_angle_rad;
  point.rear_wheel_angle_rad = inputs.at(0).rear_wheel_angle_rad;

  return std::vector{inputs[0], point, inputs[1]};
}

//
template tl::expected<
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate4(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs);

template tl::expected<std::vector<autoware_planning_msgs::msg::PathPoint>, std::string> populate4(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & inputs);

template tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string>
populate4(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs);

//
template tl::expected<
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
populate5(const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & inputs);

template tl::expected<std::vector<autoware_planning_msgs::msg::PathPoint>, std::string> populate5(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & inputs);

template tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string>
populate5(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & inputs);
}  // namespace detail

//
template std::optional<Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>
pretty_build(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const bool use_akima = false);

template std::optional<Trajectory<autoware_planning_msgs::msg::PathPoint>> pretty_build(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const bool use_akima = false);
template std::optional<Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>> pretty_build(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const bool use_akima = false);

std::optional<TemporalTrajectory> pretty_build_temporal(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const bool use_akima)
{
  if (use_akima) {
    const auto try_input5 = populate5_temporal(points);
    if (!try_input5) {
      return std::nullopt;
    }
    const auto & points_get5 = try_input5.value();

    const auto try_trajectory =
      TemporalTrajectory::Builder{}.set_xy_interpolator<interpolator::AkimaSpline>().build(
        points_get5);
    if (!try_trajectory) {
      return std::nullopt;
    }
    return try_trajectory.value();
  }

  const auto try_input4 = populate4_temporal(points);
  if (!try_input4) {
    return std::nullopt;
  }
  const auto & points_get4 = try_input4.value();

  const auto try_trajectory = TemporalTrajectory::Builder{}.build(points_get4);
  if (!try_trajectory) {
    return std::nullopt;
  }
  return try_trajectory.value();
}

}  // namespace autoware::experimental::trajectory
