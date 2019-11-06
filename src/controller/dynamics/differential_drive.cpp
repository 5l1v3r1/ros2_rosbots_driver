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

#include "controller/dynamics/differential_drive.hpp"

using namespace _rosbots_ns;

DifferentialDrive::DifferentialDrive(rclcpp::Node *parent_ros_node,
                                     double wheelbase, double wheel_radius)
    : p_parent_ros_node_(parent_ros_node), wheelbase_(wheelbase),
      wheel_radius_(wheel_radius) {
  // Wheelbase / wheel_radius - In meters per radian
  // L is the radius of the circle drawn from turning one wheel while
  // holding the other one still - happens to also be the wheelbase
}

WheelVelocity DifferentialDrive::uni_to_diff(double v, double w) const {
  // Return radians per sec wheel velocities

  // In meters per radian
  double L = this->wheelbase_;
  double R = this->wheel_radius_;

  
  // w is angular velocity counter clockwise - radians/sec
  // v - m/s
  // L - wheelbase - meter/radian
  // R - wheel radius - meter/radian
  double vr = ((2.0 * v) + (w * L)) / (2.0 * R);
  double vl = ((2.0 * v) + (-1.0 * w * L)) / (2.0 * R);

  // In radians per sec
  WheelVelocity wv = {vl, vr};
  return wv;
}
