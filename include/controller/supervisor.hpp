#ifndef __ROSBOTS_SUPERVISOR_HPP
#define __ROSBOTS_SUPERVISOR_HPP
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace rosbots {
  
class Supervisor : public rclcpp::Node {
public:
  Supervisor() : Node("rosbots_supervisor") {
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const { 
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

} // namespace rosbots

#endif // __ROSBOTS_SUPERVISOR_HPP
