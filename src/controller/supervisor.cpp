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

#include <chrono>
#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "controller/controller.hpp"
#include "controller/rc_teleop.hpp"
#include "controller/supervisor.hpp"

using namespace _rosbots_ns;
using namespace std::chrono_literals;

const unsigned int Controller::type_rc_teleop;

Supervisor::Supervisor() : Node("rosbots_supervisor") {
  // Create controllers and initialize
  auto p_rcteleop_controller =
      std::make_shared<RCTeleop>(this);
  this->controllers_.insert(
      std::pair<unsigned int, std::shared_ptr<Controller>>(
          Controller::type_rc_teleop, p_rcteleop_controller));

  // Initial state is RC Teleop
  this->current_state_ = Controller::type_rc_teleop;
  this->current_controller_ = this->controllers_[this->current_state_];

  // Create execution timer
  this->timer_ =
      this->create_wall_timer(500ms, std::bind(&Supervisor::execute_cb, this));
}

void Supervisor::execute_cb() const {
  RCLCPP_INFO(this->get_logger(), "Supervisor executing...");

  // Get commands in unicycle model
  auto ctrl_output = this->current_controller_->execute();
  RCLCPP_INFO(this->get_logger(), "Control Output %f, %f", ctrl_output.v,
              ctrl_output.w);
}
