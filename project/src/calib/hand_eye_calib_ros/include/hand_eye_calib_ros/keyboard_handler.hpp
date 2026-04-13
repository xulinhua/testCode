#ifndef HAND_EYE_CALIB_ROS__KEYBOARD_HANDLER_HPP_
#define HAND_EYE_CALIB_ROS__KEYBOARD_HANDLER_HPP_

#include <termios.h>

namespace handeyecalib_ros {

class KeyboardHandler
{
public:
    KeyboardHandler();
    ~KeyboardHandler();
    int readOne();

private:
    struct termios initial_settings_;
    struct termios new_settings_;
};

}  // namespace handeyecalib_ros

#endif  // HAND_EYE_CALIB_ROS__KEYBOARD_HANDLER_HPP_