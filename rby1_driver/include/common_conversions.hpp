#include <sensor_msgs/msg/joint_state.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot_command_builder.h"

namespace rby1_ros2 {

    void convertToProto(const sensor_msgs::msg::JointState& ros_msg, rby1::api::JointState& proto);
    void convertToRos(const rby1::api::JointState& proto, sensor_msgs::msg::JointState& ros_msg);

}