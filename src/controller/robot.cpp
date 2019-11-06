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

#include <algorithm>
#include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "controller/robot.hpp"

using namespace _rosbots_ns;

Robot::Robot(rclcpp::Node *parent_ros_node)
    : p_parent_ros_node_(parent_ros_node) {
  rclcpp::Node *pnode = this->p_parent_ros_node_;

  // Diff drive robot attributes can be stored in parameter server
  // but otherwise a ROSbots dimensions are measured as the defaults
  // wheelbase of 140mm and wheel diameter of 70mm
  pnode->get_parameter_or<double>("wheelbase", this->wheelbase_, 0.14);
  pnode->get_parameter_or<double>("wheel_radius", this->wheel_radius_, 0.035);

  // Wheel min and max no-load velocities in radians per sec
  pnode->get_parameter_or<double>("wheel_speed/min", this->wheel_speed_min_,
                                  3.1);
  pnode->get_parameter_or<double>("wheel_speed/mid", this->wheel_speed_mid_,
                                  4.4);
  pnode->get_parameter_or<double>("wheel_speed/max", this->wheel_speed_max_,
                                  5.48);
  pnode->get_parameter_or<double>("wheel_speed/min_power",
                                  this->wheel_speed_min_power_, 0.5);
  pnode->get_parameter_or<double>("wheel_speed/mid_power",
                                  this->wheel_speed_mid_power_, 0.75);
  pnode->get_parameter_or<double>("wheel_speed/max_power",
                                  this->wheel_speed_max_power_, 1.0);

  RCLCPP_INFO(pnode->get_logger(),
              "Robot: Init: Using parameters wheelbase %f, wheel_radius %f, "
              "wheel speed min/mid/max (%f, %f, %f), wheel speed power "
              "min/mid/max (%f, %f, %f)",
              this->wheelbase_, this->wheel_radius_, this->wheel_speed_min_,
              this->wheel_speed_mid_, this->wheel_speed_max_,
              this->wheel_speed_min_power_, this->wheel_speed_mid_power_,
              this->wheel_speed_max_power_);

  // Publish out initial wheel power
  this->cur_wheel_power_right_.data = 0.0;
  this->cur_wheel_power_left_.data = 0.0;
  this->pub_power_right_ =
      pnode->create_publisher<std_msgs::msg::Float32>("/wheel_power_right", 10);
  this->pub_power_left_ =
      pnode->create_publisher<std_msgs::msg::Float32>("/wheel_power_left", 10);
  this->pub_power_right_->publish(this->cur_wheel_power_right_);
  this->pub_power_left_->publish(this->cur_wheel_power_left_);
}

double Robot::velocity_to_power(double v) {
  // Input v is in radians per second
  auto av = fabs(v);

  // If velocity is below minimum velocity turnable by PWM, then
  // just set to zero since the wheels won't spin anyway
  if (av < this->wheel_speed_min_) {
    return 0.0;
  }

  double a=0.0, b=0.0, a_pow=0.0, b_pow=0.0, nnn=0.0;
  if (av >= this->wheel_speed_min_ && av < this->wheel_speed_mid_) {
    a = this->wheel_speed_min_;
    a_pow = this->wheel_speed_min_power_;
    b = this->wheel_speed_mid_;
    b_pow = this->wheel_speed_mid_power_;
  } else if (av >= this->wheel_speed_mid_ && av <= this->wheel_speed_max_) {
    a = this->wheel_speed_mid_;
    a_pow = this->wheel_speed_mid_power_;
    b = this->wheel_speed_max_;
    b_pow = this->wheel_speed_max_power_;
  }

  // Linearly interpolate a and b
  nnn = ((av - a) / (b - a));
  auto wheel_power = ((nnn * (b_pow - a_pow)) + a_pow);

  if (v < 0) {
    wheel_power *= -1.0;
  }

  return wheel_power;
}

void Robot::set_wheel_speed(double vr, double vl) {
  // The inputs vr and vl are in radians per second, as is wheel_speed_xxx_
  // Clamp the wheel speeds to actuator limits
  vr = std::max(std::min(vr, this->wheel_speed_max_),
                this->wheel_speed_max_ * -1.0);
  vl = std::max(std::min(vl, this->wheel_speed_max_),
                this->wheel_speed_max_ * -1.0);

  // Convert to power norms
  this->cur_wheel_power_right_.data = float(this->velocity_to_power(vr));
  this->cur_wheel_power_left_.data = float(this->velocity_to_power(vl));

  // Publish out
  this->pub_power_right_->publish(this->cur_wheel_power_right_);
  this->pub_power_left_->publish(this->cur_wheel_power_left_);
}
