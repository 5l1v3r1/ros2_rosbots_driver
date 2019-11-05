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

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "controller/rc_teleop.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace _rosbots_ns;

RCTeleop::RCTeleop(rclcpp::Node *parent_ros_node)
    : Controller(parent_ros_node), v_(0.0), w_(0.0) {
  rclcpp::Node *pnode = this->p_parent_ros_node_;
  RCLCPP_INFO(pnode->get_logger(),
              "RCTeleop: Init node name %s, namespace %s...", pnode->get_name(),
              pnode->get_namespace());

  // Subscribe to twist topic
  std::string topic_name = "topic";
  this->twist_sub_ = pnode->create_subscription<geometry_msgs::msg::Twist>(
      topic_name, 10,
      std::bind(&RCTeleop::twist_cb, this, std::placeholders::_1));
}

ControllerOutput RCTeleop::execute() {
  ControllerOutput output = {this->v_, this->w_};
  return output;
}

void RCTeleop::twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  const rclcpp::Node *pnode = this->p_parent_ros_node_;
  RCLCPP_INFO(pnode->get_logger(),
              "I heard twist msg: linear.x (%f), angular.z(%f)", msg->linear.x,
              msg->angular.z);

  this->v_ = msg->linear.x;
  this->w_ = msg->angular.z;
}
