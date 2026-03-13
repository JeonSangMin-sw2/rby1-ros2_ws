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
            std::string address;
            std::string model;
            std::string power_list_str;
            std::string servo_list_str;
            std::string joint_topic_name;

            std::vector<int64_t> power_on_list_;
            std::vector<std::string> servo_on_list_;

            double minimum_time;
            double angular_velocity_limit;
            double linear_velocity_limit;
            double acceleration_limit;
            double stop_orientation_tracking_error;
            double stop_position_tracking_error;

            bool fault_reset_trigger;
            bool node_power_off_trigger;

            RobotState robot_state_;
            rb::RobotInfo info_;
            std::shared_ptr<rb::Robot<ModelType>> robot_;

            //utility
            std::mutex mutex_;

            //ros2
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torso_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_pub_;

            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_sub_;
            // Timer for 100Hz publishing
            rclcpp::TimerBase::SharedPtr joint_state_timer_;
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
            void position_command(std::string joint_space, std::vector<double> position);
            void position_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        private:
            void init_parameter();
            void resize_joint_states();
            void categorize_joints();
            void publish_joint_states();        
    };
}