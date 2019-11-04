#ifndef __ROSBOTS_RC_TELEOP_HPP
#define __ROSBOTS_RC_TELEOP_HPP
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "controller/controller.hpp"

namespace _rosbots_ns {

class RCTeleop : public Controller {
public:
  RCTeleop() {}

  virtual ControllerOutput execute() { return {2.0, 4.0}; }
};

} // namespace _rosbots_ns

#endif // __ROSBOTS_RC_TELEOP_HPP
