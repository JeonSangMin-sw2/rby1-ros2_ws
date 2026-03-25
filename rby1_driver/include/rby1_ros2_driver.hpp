#pragma once

#include <mutex>
#include <chrono>
#include <thread>
//ros2
#include "rclcpp/rclcpp.hpp"
//sdk
#include "rby1-sdk/robot.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot_command_builder.h"
//local header file
#include "type.hpp"

namespace rby1_ros2{
    template <typename ModelType>
    class RBY1_ROS2_DRIVER : public rclcpp::Node {
        private:
            //robot
            RobotParameter robot_parameter_;
            RobotJoint robot_joint_;
            RobotState robot_state_;
            std::shared_ptr<rb::Robot<ModelType>> robot_;

            //utility
            std::mutex mutex_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_torso_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_right_arm_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_left_arm_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_head_;
        public:
            RBY1_ROS2_DRIVER();
            ~RBY1_ROS2_DRIVER();
            bool power_on(std::vector<int64_t> power_list = {5,12,24,48});
            bool power_off(std::vector<int64_t> power_list = {5,12,24,48});
            bool servo_on(std::vector<std::string> servo_list = {"all"});
            bool servo_on(std::string servo_name = "all");
            // bool servo_off(std::vector<std::string> servo_list = {"all"});
            // bool servo_off(std::string servo_name);

            bool check_controll_manager();
            void read_joint_state();
            // std::vector<double> get_joint_velocity(std::string joint_space);
            // std::vector<double> get_joint_effort(std::string joint_space);

        private:
            void get_parameters();
            void init_joint_states();
    };
}