#include "common_conversions.hpp"

namespace rby1_ros2 {
    void convertToProto(const sensor_msgs::msg::JointState& ros_msg, rby1::api::JointState& proto) {
        for (size_t i = 0; i < ros_msg.name.size(); i++) {
            proto.add_name(ros_msg.name[i]);
            proto.add_position(ros_msg.position[i]);
            proto.add_velocity(ros_msg.velocity[i]);
            proto.add_effort(ros_msg.effort[i]);
        }
    }
    
    void convertToRos(const rby1::api::JointState& proto, sensor_msgs::msg::JointState& ros_msg) {
        ros_msg.name.clear();
        ros_msg.position.clear();
        ros_msg.velocity.clear();
        ros_msg.effort.clear();
        for (size_t i = 0; i < proto.name_size(); i++) {
            ros_msg.name.push_back(proto.name(i));
            ros_msg.position.push_back(proto.position(i));
            ros_msg.velocity.push_back(proto.velocity(i));
            ros_msg.effort.push_back(proto.effort(i));
        }
    }
}
