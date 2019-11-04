#ifndef __ROSBOTS_SUPERVISOR_HPP
#define __ROSBOTS_SUPERVISOR_HPP
#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "controller/controller.hpp"

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
};

} // namespace _rosbots_ns

#endif // __ROSBOTS_SUPERVISOR_HPP
