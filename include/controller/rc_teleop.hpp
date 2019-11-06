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

#ifndef __ROSBOTS_RC_TELEOP_HPP
#define __ROSBOTS_RC_TELEOP_HPP
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "controller/controller.hpp"

namespace _rosbots_ns {

class RCTeleop : public Controller {
public:
  RCTeleop(rclcpp::Node* parent_ros_node);

  ControllerOutput execute();
private:
  double v_;  // Meters per second
  double w_;  // Radians per second

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  void twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
};

} // namespace _rosbots_ns

#endif // __ROSBOTS_RC_TELEOP_HPP
