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

#ifndef __ROSBOTS_SUPERVISOR_HPP
#define __ROSBOTS_SUPERVISOR_HPP
#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "controller/controller.hpp"
#include "controller/robot.hpp"
#include "controller/dynamics/differential_drive.hpp"

namespace _rosbots_ns {

class Supervisor : public rclcpp::Node {
public:
  // Supervisor() : Node("rosbots_supervisor") {
  //}
  Supervisor();

private:
  void execute_cb() const;

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::map<unsigned int, std::shared_ptr<Controller>> controllers_;
  int current_state_;
  std::shared_ptr<Controller> current_controller_;

  std::shared_ptr<Robot> robot_;

  std::shared_ptr<DifferentialDrive> dd_;
};

} // namespace _rosbots_ns

#endif // __ROSBOTS_SUPERVISOR_HPP
