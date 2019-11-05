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

#include "rclcpp/rclcpp.hpp"

#include "controller/robot.hpp"

using namespace _rosbots_ns;

Robot::Robot(rclcpp::Node* parent_ros_node) : p_parent_ros_node_(parent_ros_node) {
  rclcpp::Node *pnode = this->p_parent_ros_node_;
  RCLCPP_INFO(pnode->get_logger(),
              "Robot: Init: node name %s, namespace %s...", pnode->get_name(),
              pnode->get_namespace());

   
}
