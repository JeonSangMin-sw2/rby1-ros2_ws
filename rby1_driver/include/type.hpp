#include <sensor_msgs/msg/joint_state.hpp>

namespace rby1_ros2{
    using JointState = sensor_msgs::msg::JointState;
    enum RobotState{
        NONE,
        IDLE,
        ENABLE,
        EXECUTING,
        SWITCHING,
        MAJOR_FAULT,
        MINOR_FAULT
    };

    struct RobotInfo{
        bool is_power_on = false;
        bool is_servo_on = false;
        bool is_ready = false;
        std::string address;
        std::string model;
        RobotState robot_state = IDLE;
    };

    struct RobotJoint{
        JointState joint_torso;
        JointState joint_right_arm;
        JointState joint_left_arm;
        JointState joint_head;
    }

    struct RobotParameter{
        std::string joint_topic_name;
        std::vector<int64_t> power_on_list;
        std::vector<std::string> servo_on_list;
        double minimum_time;
        double angular_velocity_limit;
        double linear_velocity_limit;
        double acceleration_limit;
        double stop_orientation_tracking_error;
        double stop_position_tracking_error;
    }
}
