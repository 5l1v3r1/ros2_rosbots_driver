#ifndef __ROSBOTS_CONTROLLER_HPP
#define __ROSBOTS_CONTROLLER_HPP
#include <memory>

namespace _rosbots_ns {

struct ControllerOutput {
  float v;
  float w;
};

class Controller {
public:
  static const unsigned int type_rc_teleop = 1;

  virtual ControllerOutput execute() = 0;
};

} // namespace _rosbots_ns

#endif // __ROSBOTS_CONTROLLER_HPP
