#include <sensor_msgs/msg/joint_state.hpp>

namespace rby1_ros2{
    using JointState = sensor_msgs::msg::JointState;
    enum ErrorType{
        NONE,
        IDLE,
        ENABLE,
        EXECUTING,
        SWITCHING,
        MAJOR,
        MINOR
    };

    struct RobotState{
        bool is_power_on = false;
        bool is_servo_on = false;
        bool is_ready = false;
        ErrorType error_type = IDLE;
        JointState joint_torso;
        JointState joint_right_arm;
        JointState joint_left_arm;
        JointState joint_head;
        

    };
}
