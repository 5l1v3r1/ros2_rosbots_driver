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

#ifndef __ROSBOTS_CONTROLLER_HPP
#define __ROSBOTS_CONTROLLER_HPP
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace _rosbots_ns {

struct ControllerOutput {
  double v;  // Meters per second
  double w;  // Radians per second
};

class Controller {
public:
  static const unsigned int type_rc_teleop = 1;

  Controller(rclcpp::Node *parent_ros_node)
      : p_parent_ros_node_(parent_ros_node) {}

  virtual ControllerOutput execute() = 0;

protected:
  rclcpp::Node *p_parent_ros_node_;
};

} // namespace _rosbots_ns

#endif // __ROSBOTS_CONTROLLER_HPP
