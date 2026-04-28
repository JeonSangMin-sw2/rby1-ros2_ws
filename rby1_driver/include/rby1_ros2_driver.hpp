#pragma once

#include <mutex>
#include <chrono>
#include <thread>
#include <optional>
//ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rby1_msgs/srv/state_on_off.hpp"
#include "rby1_msgs/action/multi_joint_command.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rby1_msgs/action/single_joint_command.hpp"
//sdk
#include "rby1-sdk/robot.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot_command_builder.h"
//local header file
#include "type.hpp"

namespace rby1_ros2{
    struct BuilderConfig {
        std::string control_mode; // "joint_position", "joint_impedance", "cartesian_impedance"
        
        // Cartesian Impedance Parameters
        std::string ref_link;
        std::string target_link;
        std::vector<double> translation_weight;
        std::vector<double> rotation_weight;
        
        // Joint Impedance Parameters
        std::vector<double> joint_stiffness;
        
        // Common Impedance Parameters
        double damping_ratio;
    };

    template <typename ModelType>
    class RBY1_ROS2_DRIVER : public rclcpp::Node {
        private:
            //robot
            RobotParameter robot_parameter_;
            RobotJoint robot_joint_;
            RobotState robot_state_;
            rb::RobotInfo info_;

            std::shared_ptr<rb::Robot<ModelType>> robot_;

            std::string address;
            std::string model;
            std::string state_topic_name;
            std::string servo_list_str;
            std::string power_list_str;
            bool fault_reset_trigger;
            bool node_power_off_trigger;

            //utility
            std::mutex mutex_;

            //ros2
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torso_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_pub_;
            rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_control_state_;

            // Timer for 100Hz publishing
            rclcpp::TimerBase::SharedPtr joint_state_timer_;

            using MultiJointCommand = rby1_msgs::action::MultiJointCommand;
            using SingleJointCommand = rby1_msgs::action::SingleJointCommand;
            
            rclcpp_action::Server<MultiJointCommand>::SharedPtr multi_position_action_server_;
            rclcpp_action::Server<SingleJointCommand>::SharedPtr single_position_action_server_;
            
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr power_service_;
            rclcpp::Service<rby1_msgs::srv::StateOnOff>::SharedPtr servo_service_;

            BuilderConfig torso_builder_;
            BuilderConfig right_arm_builder_;
            BuilderConfig left_arm_builder_;
            BuilderConfig head_builder_;
        public:
            RBY1_ROS2_DRIVER();
            ~RBY1_ROS2_DRIVER();
            bool check_controll_manager();
            void read_joint_state();
            std::string finish_code_to_string(rb::RobotCommandFeedback::FinishCode code);
            
            void apply_builder(
                rb::BodyComponentBasedCommandBuilder& body_comp, 
                rb::ComponentBasedCommandBuilder& comp,
                const std::string& part_name,
                const std::vector<double>& goal_data,
                double min_time);
            void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

        private:
            void init_parameter();
            void resize_joint_states();
            void publish_joint_states();
            
            void power_control(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                               std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response);
            void servo_control(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                               std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response);

            // Multi-joint Action Handlers
            rclcpp_action::GoalResponse handle_multi_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MultiJointCommand::Goal> goal);
            rclcpp_action::CancelResponse handle_multi_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle);
            void handle_multi_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle);
            void execute_multi_command(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle);

            // Single-joint Action Handlers
            rclcpp_action::GoalResponse handle_single_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SingleJointCommand::Goal> goal);
            rclcpp_action::CancelResponse handle_single_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle);
            void handle_single_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle);
            void execute_single_command(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle);
    };
}