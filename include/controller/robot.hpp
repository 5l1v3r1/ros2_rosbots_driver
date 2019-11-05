/*
 * This file is part of ROSbots ROS Drivers.
 *
 * Copyright
 *
 *     Copyright (C) 2019 Jack Pien <jack@rosbots.com>
 *
 * License
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU Lesser General Public License as published
 *     by the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU Lesser General Public License for more details at
 *     <http://www.gnu.org/licenses/lgpl-3.0-standalone.html>
 *
 * Documentation
 *
 *     http://www.rosbots.com
 */

#ifndef _ROSBOTS_ROBOT_HPP
#define _ROSBOTS_ROBOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace _rosbots_ns {

class Robot {

public:
  Robot(rclcpp::Node* parent_ros_node);

private:
  rclcpp::Node* p_parent_ros_node_;
  double wheelbase_;
  double wheel_radius_;

  double wheel_speed_min;
  double wheel_speed_mid;
  double wheel_speed_max;
  std_msgs::msg::Float32 cur_wheel_power_right;
  std_msgs::msg::Float32 cur_wheel_power_left;
};

} // namespace _rosbots_ns

#endif // _ROSBOTS_ROBOT_HPP
