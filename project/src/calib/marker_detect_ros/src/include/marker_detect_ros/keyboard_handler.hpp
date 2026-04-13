#ifndef MARKER_DETECT_ROS__KEYBOARD_HANDLER_HPP_
#define MARKER_DETECT_ROS__KEYBOARD_HANDLER_HPP_

#include <termios.h>

namespace marker_detect_ros {

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

}  // namespace marker_detect_ros

#endif  // MARKER_DETECT_ROS__KEYBOARD_HANDLER_HPP_
