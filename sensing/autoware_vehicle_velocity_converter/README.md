# autoware_vehicle_velocity_converter

## Purpose

This package converts autoware_vehicle_msgs::msg::VehicleReport message to geometry_msgs::msg::TwistWithCovarianceStamped for gyro odometer node.

## Inputs / Outputs

### Input

| Name                           | Type                                                | Description                  |
| ------------------------------ | --------------------------------------------------- | ---------------------------- |
| `velocity_status`              | `autoware_vehicle_msgs::msg::VehicleReport`         | vehicle velocity             |
| `estimated_speed_scale_factor` | `autoware_internal_debug_msgs::msg::Float32Stamped` | estimated speed scale factor |

### Output

| Name                               | Type                                                | Description                                        |
| ---------------------------------- | --------------------------------------------------- | -------------------------------------------------- |
| `twist_with_covariance`            | `geometry_msgs::msg::TwistWithCovarianceStamped`    | twist with covariance converted from VehicleReport |
| `debug/current_speed_scale_factor` | `autoware_internal_debug_msgs::msg::Float32Stamped` | current speed scale factor                         |

## Parameters

| Name                                           | Type   | Description                                              |
| ---------------------------------------------- | ------ | -------------------------------------------------------- |
| `speed_scale_factor`                           | double | speed scale factor (ideal value is 1.0)                  |
| `frame_id`                                     | string | frame id for output message                              |
| `velocity_stddev_xx`                           | double | standard deviation for vx                                |
| `angular_velocity_stddev_zz`                   | double | standard deviation for yaw rate                          |
| `enable_online_speed_scale_factor_calibration` | bool   | enable online calibration of speed scale factor          |
| `acceptable_speed_scale_factor_range`          | array  | acceptable range for speed scale factor [min, max]       |
| `stop_speed_threshold`                         | double | speed threshold to determine if vehicle is stopped [m/s] |
