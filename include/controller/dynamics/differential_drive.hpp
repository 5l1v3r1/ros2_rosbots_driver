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

#ifndef _ROSBOTS_DIFFERENTIAL_DRIVE_HPP
#define _ROSBOTS_DIFFERENTIAL_DRIVE_HPP

#include "rclcpp/rclcpp.hpp"

namespace _rosbots_ns {

struct WheelVelocity {
  double vl;
  double vr;
};

class DifferentialDrive {
public:
  DifferentialDrive(rclcpp::Node *parent_ros_node, double wheelbase,
                    double wheel_radius);

  WheelVelocity uni_to_diff(double v, double w) const;

private:
  rclcpp::Node* p_parent_ros_node_;
  double wheelbase_;
  double wheel_radius_;
};

} // namespace _rosbots_ns

#endif // _ROSBOTS_DIFFERENTIAL_DRIVE_HPP
