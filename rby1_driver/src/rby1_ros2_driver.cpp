#include "rby1_ros2_driver.hpp"
namespace rby1_ros2{
    //using namespace rb;


    template <typename ModelType>
    RBY1_ROS2_DRIVER<ModelType>::RBY1_ROS2_DRIVER()
        : Node("rby1_ros2_driver"){
            
            //declare parameter from yaml
            init_parameter();
            try{
                robot_ = rb::Robot<ModelType>::Create(address);
                if(robot_->Connect()){
                    robot_->SetParameter("default.acceleration_limit_scaling", std::to_string(robot_parameter_.acceleration_limit));
                    robot_->SetParameter("joint_position_command.cutoff_frequency", std::to_string(robot_parameter_.angular_velocity_limit));
                    robot_->SetParameter("cartesian_command.cutoff_frequency", std::to_string(robot_parameter_.linear_velocity_limit));
                    robot_->SetParameter("default.linear_acceleration_limit", std::to_string(robot_parameter_.acceleration_limit));
                    // Fetch robot info once and cache it
                    info_ = robot_->GetRobotInfo();
                    RCLCPP_INFO(this->get_logger(), "Robot Info: Model=%s, Version=%s", info_.robot_model_name.c_str(), info_.robot_model_version.c_str());
                    RCLCPP_INFO(this->get_logger(), "Joint counts: torso=%zu, right_arm=%zu, left_arm=%zu, head=%zu, mobility=%zu", 
                                info_.torso_joint_idx.size(), info_.right_arm_joint_idx.size(), 
                                info_.left_arm_joint_idx.size(), info_.head_joint_idx.size(), 
                                info_.mobility_joint_idx.size());
                    resize_joint_states();
                }
                
                torso_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/torso", 10);
                right_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/right_arm", 10);
                left_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/left_arm", 10);
                head_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_name + "/head", 10);
                pub_control_state_ = this->create_publisher<std_msgs::msg::Int32>(state_topic_name + "/control_state", 10);
                
                // Timer for 100Hz publishing (10ms)
                joint_state_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(robot_parameter_.get_state_period*1000.0)), std::bind(&RBY1_ROS2_DRIVER<ModelType>::read_joint_state, this));
                
                using namespace std::placeholders;
                power_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
                    "robot_power", std::bind(&RBY1_ROS2_DRIVER<ModelType>::power_control, this, _1, _2));
                servo_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
                    "robot_servo", std::bind(&RBY1_ROS2_DRIVER<ModelType>::servo_control, this, _1, _2));

                multi_position_action_server_ = rclcpp_action::create_server<MultiJointCommand>(
                    this,
                    state_topic_name + "/multi_position_command",
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_multi_goal, this, _1, _2),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_multi_cancel, this, _1),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_multi_accepted, this, _1));

                single_position_action_server_ = rclcpp_action::create_server<SingleJointCommand>(
                    this,
                    state_topic_name + "/single_position_command",
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_single_goal, this, _1, _2),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_single_cancel, this, _1),
                    std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_single_accepted, this, _1));
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "에러 발생: %s", e.what());
            }
    }

    template <typename ModelType>
    RBY1_ROS2_DRIVER<ModelType>::~RBY1_ROS2_DRIVER(){
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::init_parameter(){
        RCLCPP_INFO(this->get_logger(), "Declaring parameters...");
        this->declare_parameter<std::string>("robot_ip", "127.0.0.1:50051");
        this->declare_parameter<std::string>("model", "a");
        this->declare_parameter<std::string>("state_topic_name", "joint_states");
        this->declare_parameter<std::string>("control_mode.torso", "joint");
        this->declare_parameter<std::string>("control_mode.right_arm", "joint");
        this->declare_parameter<std::string>("control_mode.left_arm", "joint");
        this->declare_parameter<std::string>("control_mode.head", "joint");
        this->declare_parameter<std::vector<std::string>>("power_on", {"all"});
        this->declare_parameter<std::vector<std::string>>("servo_on", {"all"});

        this->declare_parameter<double>("get_state_period", 0.01);
        this->declare_parameter<double>("minimum_time", 2.0);
        this->declare_parameter<double>("angular_velocity_limit", 4.712388);
        this->declare_parameter<double>("linear_velocity_limit", 1.5);
        this->declare_parameter<double>("acceleration_limit", 1.0);
        this->declare_parameter<double>("stop_orientation_tracking_error", 1e-5);
        this->declare_parameter<double>("stop_position_tracking_error", 1e-5);
        
        this->declare_parameter<bool>("fault_reset_trigger", false);
        this->declare_parameter<bool>("node_power_off_trigger",false);
        
        this->get_parameter("robot_ip", address);
        this->get_parameter("model", model);
        this->get_parameter("state_topic_name", state_topic_name);
        
        torso_builder_.control_mode = this->get_parameter("control_mode.torso").as_string();
        right_arm_builder_.control_mode = this->get_parameter("control_mode.right_arm").as_string();
        left_arm_builder_.control_mode = this->get_parameter("control_mode.left_arm").as_string();
        head_builder_.control_mode = this->get_parameter("control_mode.head").as_string();

        this->get_parameter("power_on", robot_parameter_.power_on_list);
        this->get_parameter("servo_on", robot_parameter_.servo_on_list);
        this->get_parameter("get_state_period", robot_parameter_.get_state_period);
        this->get_parameter("minimum_time", robot_parameter_.minimum_time);
        this->get_parameter("angular_velocity_limit", robot_parameter_.angular_velocity_limit);
        this->get_parameter("linear_velocity_limit", robot_parameter_.linear_velocity_limit);
        this->get_parameter("acceleration_limit", robot_parameter_.acceleration_limit);
        this->get_parameter("stop_orientation_tracking_error", robot_parameter_.stop_orientation_tracking_error);
        this->get_parameter("stop_position_tracking_error", robot_parameter_.stop_position_tracking_error);
        this->get_parameter("fault_reset_trigger", fault_reset_trigger);
        this->get_parameter("node_power_off_trigger", node_power_off_trigger);

        auto init_bcfg = [](BuilderConfig& bcfg, const std::string& mode, const std::string& ref, const std::string& target, int dof) {
            bcfg.control_mode = mode;
            bcfg.ref_link = ref;
            bcfg.target_link = target;
            bcfg.translation_weight = {1000.0, 1000.0, 1000.0};
            bcfg.rotation_weight = {100.0, 100.0, 100.0};
            bcfg.joint_stiffness = std::vector<double>(dof, 1000.0);
            bcfg.damping_ratio = 0.85;
        };

        init_bcfg(torso_builder_, this->get_parameter("control_mode.torso").as_string(), "base", "link_torso_5", 6);
        init_bcfg(right_arm_builder_, this->get_parameter("control_mode.right_arm").as_string(), "link_torso_5", "ee_right", 7);
        init_bcfg(left_arm_builder_, this->get_parameter("control_mode.left_arm").as_string(), "link_torso_5", "ee_left", 7);
        init_bcfg(head_builder_, this->get_parameter("control_mode.head").as_string(), "base", "ee_head", 2);

        if (address == "" || model == ""){
            RCLCPP_ERROR(this->get_logger(), "address or model isn't declared");
            rclcpp::shutdown();
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::power_control(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                                                    std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
        
        // Parse parameters string
        std::vector<std::string> param_list;
        std::stringstream ss(request->parameters);
        std::string token;
        while(std::getline(ss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (!token.empty()) param_list.push_back(token);
        }
        if (param_list.empty()) param_list.push_back("all");

        std::string power_list_str = "";
        if (address == "127.0.0.1:50051"){
            power_list_str = ".*";
        }else{
            for (size_t i = 0; i < param_list.size(); i++) {
                if (param_list[i] == "all" || param_list[i] == ".*") {
                    power_list_str = ".*";
                    break;
                }
                power_list_str += param_list[i];
                if (param_list[i].find('v') == std::string::npos && param_list[i] != ".*") {
                    power_list_str += "v";
                }
                if (i != param_list.size() - 1){
                    power_list_str += "|";
                }
            }
        }

        if (request->state) {
            RCLCPP_INFO(this->get_logger(), "power on [%s]", power_list_str.c_str());
            if (!robot_->IsPowerOn(power_list_str) && !robot_->PowerOn(power_list_str)) {
                response->success = false;
                response->message = "Failed to power on";
                return;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "power off [%s]", power_list_str.c_str());
            if (!robot_->PowerOff(power_list_str)) {
                response->success = false;
                response->message = "Failed to power off";
                return;
            }
        }
        response->success = true;
        response->message = "Power control success";
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::servo_control(const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
                                                    std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
        // Parse parameters string
        std::vector<std::string> param_list;
        std::stringstream ss(request->parameters);
        std::string token;
        while(std::getline(ss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (!token.empty()) param_list.push_back(token);
        }
        if (param_list.empty()) param_list.push_back("all");

        std::string servo_list_str = "";
        for (size_t i = 0; i < param_list.size(); i++) {
            std::string name = param_list[i];
            if(name == "right")      servo_list_str += "^right_arm_.*";
            else if(name == "left")  servo_list_str += "^left_arm_.*";
            else if(name == "head")  servo_list_str += "^head_.*";
            else if(name == "torso") servo_list_str += "^torso_.*";
            else if(name == "all" || name == ".*") {
                servo_list_str = ".*";
                break;
            } else {
                servo_list_str += name;
            }
            if (i != param_list.size() - 1){
                servo_list_str += "|";
            }
        }

        if (request->state) {
            RCLCPP_INFO(this->get_logger(), "servo on [%s]", servo_list_str.c_str());
            if (!robot_->IsServoOn(servo_list_str) && !robot_->ServoOn(servo_list_str)) {
                response->success = false;
                response->message = "Failed to servo on";
                return;
            }
            
            { // sync info 
                std::lock_guard<std::mutex> lock(mutex_);
                info_ = robot_->GetRobotInfo();
                resize_joint_states();
            }

            if (!check_controll_manager()) {
                response->success = false;
                response->message = "Failed to enable control manager";
                return;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "servo off [%s]", servo_list_str.c_str());
            robot_->DisableControlManager();
            if (!robot_->ServoOff(servo_list_str)) {
                response->success = false;
                response->message = "Failed to servo off";
                return;
            }
        }
        auto state = robot_->GetState();
        std::vector<std::string> servo_on_joints;
        for (size_t i = 0; i < info_.joint_infos.size(); ++i) {
            if (state.is_ready[i]) {
                servo_on_joints.push_back(info_.joint_infos[i].name);
            }
        }
        // 출력해보기
        std::cout << "--- Servo On Joints ---" << std::endl;
        for (const auto& name : servo_on_joints) {
            std::cout << name << std::endl;
        }
    }

    // template <typename ModelType>
    // bool RBY1_ROS2_DRIVER<ModelType>::servo_on(std::string servo_name){
    //     if(servo_name == "all" || servo_name == ".*"){
    //         servo_list_str = ".*";
    //     }else if(servo_name == "right"){
    //         servo_list_str = "^right_arm_.*";
    //     }else if(servo_name == "left"){
    //         servo_list_str = "^left_arm_.*";
    //     }else if(servo_name == "head"){
    //         servo_list_str = "^head_.*";
    //     }else if(servo_name == "torso"){
    //         servo_list_str = "^torso_.*";
    //     }else{
    //         servo_list_str = servo_name;
    //     }
    //     if (!robot_->IsServoOn(servo_list_str) && !robot_->ServoOn(servo_list_str)) return false;
        
    //     {
    //         std::lock_guard<std::mutex> lock(mutex_);
    //         info_ = robot_->GetRobotInfo();
    //         resize_joint_states();
    //     }

    //     auto state = robot_->GetState();
    //     std::vector<std::string> servo_on_joints;
    //     // T::kRobotDOF는 모델의 총 관절 개수를 의미합니다 (보통 info.joint_infos.size()와 동일)
    //     for (size_t i = 0; i < info_.joint_infos.size(); ++i) {
    //         if (state.is_ready[i]) { // 해당 인덱스의 관절이 서보온 상태(ready) 라면
    //             servo_on_joints.push_back(info_.joint_infos[i].name);
    //         }
    //     }
    //     // 출력해보기
    //     std::cout << "--- Servo On Joints ---" << std::endl;
    //     for (const auto& name : servo_on_joints) {
    //         std::cout << name << std::endl;
    //     }
    //     return true;
    // }

    template <typename ModelType>
    bool RBY1_ROS2_DRIVER<ModelType>::check_controll_manager(){
        robot_->ResetFaultControlManager();
        if (!robot_->EnableControlManager()) return false;
        
        const auto& control_manager_state = robot_->GetControlManagerState();

        if (control_manager_state.state == rb::ControlManagerState::State::kMajorFault ||
            control_manager_state.state == rb::ControlManagerState::State::kMinorFault)
        {
            RCLCPP_ERROR(this->get_logger(), "Detected a %s fault in the Control Manager.", 
                    (control_manager_state.state == rb::ControlManagerState::State::kMajorFault ? "Major" : "Minor"));
        
            if(fault_reset_trigger){
                RCLCPP_INFO(this->get_logger(), "Attempting to reset the fault...");
                if (!robot_->ResetFaultControlManager()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to reset the fault in the Control Manager.");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Fault reset successfully.");
            }else{
                return false;
            }
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Control Manager state is normal. No faults detected.");
        }
        
        RCLCPP_INFO(this->get_logger(), "Enabling Control Manager...");
        if (!robot_->EnableControlManager()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable the Control Manager.");
            return false;
        }

        // Wait for control ready to ensure SendCommand doesn't fail with kUnknown immediately
        if (!robot_->WaitForControlReady(2000)) {
            RCLCPP_WARN(this->get_logger(), "Control Manager enabled, but timed out waiting for Control Ready status.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Control Manager enabled and ready.");
        }
        
        return true;
    }

    template <typename ModelType>
    std::string RBY1_ROS2_DRIVER<ModelType>::finish_code_to_string(rb::RobotCommandFeedback::FinishCode code) {
        switch (code) {
            case rb::RobotCommandFeedback::FinishCode::kUnknown: return "kUnknown (0)";
            case rb::RobotCommandFeedback::FinishCode::kOk: return "kOk (1)";
            case rb::RobotCommandFeedback::FinishCode::kCanceled: return "kCanceled (2)";
            case rb::RobotCommandFeedback::FinishCode::kPreempted: return "kPreempted (3)";
            case rb::RobotCommandFeedback::FinishCode::kInitializationFailed: return "kInitializationFailed (4)";
            case rb::RobotCommandFeedback::FinishCode::kControlManagerIdle: return "kControlManagerIdle (5)";
            case rb::RobotCommandFeedback::FinishCode::kControlManagerFault: return "kControlManagerFault (6)";
            case rb::RobotCommandFeedback::FinishCode::kUnexpectedState: return "kUnexpectedState (7)";
            default: return "Unknown (" + std::to_string((int)code) + ")";
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::read_joint_state(){
        if (info_.joint_infos.empty()) return; // info가 아직 오지 않았으면 리턴
        
        auto state = robot_->GetState();
        auto cm_state = robot_->GetControlManagerState();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto now = this->now();
            
            if (cm_state.state == rb::ControlManagerState::State::kMinorFault) {
                robot_state_.state = MINOR_FAULT;
            } else if (cm_state.state == rb::ControlManagerState::State::kMajorFault) {
                robot_state_.state = MAJOR_FAULT;
            } else if (cm_state.state == rb::ControlManagerState::State::kIdle) {
                robot_state_.state = IDLE;
            } else if (cm_state.state == rb::ControlManagerState::State::kEnabled) {
                if (cm_state.control_state == rb::ControlManagerState::ControlState::kExecuting) {
                    robot_state_.state = EXECUTING;
                } else {
                    robot_state_.state = ENABLE;
                }
            } else {
                robot_state_.state = NONE;
            }
            
            auto fill = [&](JointState& js, const std::vector<unsigned int>& idx_vec){
                js.header.stamp = now;
                for (size_t i = 0; i < idx_vec.size(); ++i) {
                    unsigned int idx = idx_vec[i];
                    js.name[i]     = info_.joint_infos[idx].name;
                    js.position[i] = state.position[idx];
                    js.velocity[i] = state.velocity[idx];
                    js.effort[i]   = state.torque[idx];
                }
            };

            fill(robot_joint_.joint_torso,      info_.torso_joint_idx);
            fill(robot_joint_.joint_right_arm,  info_.right_arm_joint_idx);
            fill(robot_joint_.joint_left_arm,   info_.left_arm_joint_idx);
            fill(robot_joint_.joint_head,       info_.head_joint_idx);
            //fill(robot_joint_.joint_wheel,      info_.mobility_joint_idx);

            torso_pub_->publish(robot_joint_.joint_torso);
            right_arm_pub_->publish(robot_joint_.joint_right_arm);
            left_arm_pub_->publish(robot_joint_.joint_left_arm);
            head_pub_->publish(robot_joint_.joint_head);
            
            std_msgs::msg::Int32 state_msg;
            state_msg.data = static_cast<int32_t>(robot_state_.state);
            pub_control_state_->publish(state_msg);
           // wheel_pub_->publish(robot_joint_.joint_wheel);
        }
    }


    // --- MultiJointCommand Handlers ---
    template <typename ModelType>
    rclcpp_action::GoalResponse RBY1_ROS2_DRIVER<ModelType>::handle_multi_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MultiJointCommand::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received MultiJointCommand request");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    template <typename ModelType>
    rclcpp_action::CancelResponse RBY1_ROS2_DRIVER<ModelType>::handle_multi_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel MultiJointCommand goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::handle_multi_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_multi_command, this, _1), goal_handle}.detach();
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::apply_builder(
        rb::BodyComponentBasedCommandBuilder& body_comp, 
        rb::ComponentBasedCommandBuilder& comp,
        const std::string& part_name,
        const std::vector<double>& goal_data,
        double min_time) {
        
        BuilderConfig* bcfg = nullptr;
        if (part_name == "torso") bcfg = &torso_builder_;
        else if (part_name == "right_arm") bcfg = &right_arm_builder_;
        else if (part_name == "left_arm") bcfg = &left_arm_builder_;
        else if (part_name == "head") bcfg = &head_builder_;
        else return;

        if (bcfg->control_mode == "joint_position" || bcfg->control_mode == "joint") {
            auto b = rb::JointPositionCommandBuilder();
            b.SetMinimumTime(min_time);
            Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(goal_data.data(), goal_data.size());
            b.SetPosition(q);
            if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
            else if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
            else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
            else if (part_name == "head") comp.SetHeadCommand(rb::HeadCommandBuilder(b));
        } else if (bcfg->control_mode == "joint_impedance") {
            auto b = rb::JointImpedanceControlCommandBuilder();
            b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(min_time));
            Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(goal_data.data(), goal_data.size());
            b.SetPosition(q);
            Eigen::VectorXd stiffness = Eigen::Map<const Eigen::VectorXd>(bcfg->joint_stiffness.data(), bcfg->joint_stiffness.size());
            if (stiffness.size() == q.size()) {
                b.SetStiffness(stiffness);
            }
            b.SetDampingRatio(bcfg->damping_ratio);
            
            if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
            else if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
            else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
            else if (part_name == "head") {
                RCLCPP_WARN(this->get_logger(), "Head does not support joint impedance control.");
            }
        } else if (bcfg->control_mode == "cartesian_impedance" || bcfg->control_mode == "impedance") {
            auto b = rb::ImpedanceControlCommandBuilder();
            b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(min_time));
            b.SetReferenceLinkName(bcfg->ref_link);
            b.SetLinkName(bcfg->target_link);
            
            Eigen::Vector3d t_weight(bcfg->translation_weight[0], bcfg->translation_weight[1], bcfg->translation_weight[2]);
            Eigen::Vector3d r_weight(bcfg->rotation_weight[0], bcfg->rotation_weight[1], bcfg->rotation_weight[2]);
            b.SetTranslationWeight(t_weight);
            b.SetRotationWeight(r_weight);
            b.SetDampingRatio(bcfg->damping_ratio);
            
            if (goal_data.size() == 16) {
                Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(goal_data.data());
                b.SetTransformation(T);
            } else if (goal_data.size() == 7) {
                Eigen::Quaterniond q(goal_data[6], goal_data[3], goal_data[4], goal_data[5]);
                Eigen::Vector3d t(goal_data[0], goal_data[1], goal_data[2]);
                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                T.block<3,3>(0,0) = q.toRotationMatrix();
                T.block<3,1>(0,3) = t;
                b.SetTransformation(T);
            } else {
                RCLCPP_WARN(this->get_logger(), "Cartesian Impedance %s requires 7 or 16 float elements, got %zu", part_name.c_str(), goal_data.size());
                return;
            }
            if (part_name == "torso") body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
            else if (part_name == "right_arm") body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
            else if (part_name == "left_arm") body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::execute_multi_command(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiJointCommand>> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MultiJointCommand::Result>();

        bool use_torso = !goal->torso.empty();
        bool use_right_arm = !goal->right_arm.empty();
        bool use_left_arm = !goal->left_arm.empty();
        bool use_head = !goal->head.empty();

        if (!use_torso && !use_right_arm && !use_left_arm && !use_head) {
            RCLCPP_WARN(this->get_logger(), "Received empty MultiJointCommand goal.");
            result->success = false;
            result->finish_code = "Empty arrays";
            goal_handle->abort(result);
            return;
        }

        if (!robot_->HasEstablishedTimeSync()) robot_->SyncTime();
        double min_time = (goal->minimum_time > 0.01) ? goal->minimum_time : robot_parameter_.minimum_time;

        rb::ComponentBasedCommandBuilder component_cmd_builder;
        rb::BodyComponentBasedCommandBuilder body_comp;

        if (use_torso) apply_builder(body_comp, component_cmd_builder, "torso", goal->torso, min_time);
        if (use_right_arm) apply_builder(body_comp, component_cmd_builder, "right_arm", goal->right_arm, min_time);
        if (use_left_arm) apply_builder(body_comp, component_cmd_builder, "left_arm", goal->left_arm, min_time);
        if (use_head) apply_builder(body_comp, component_cmd_builder, "head", goal->head, min_time);

        if (use_torso || use_right_arm || use_left_arm) {
            component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));
        }

        auto cmd_handler = robot_->SendCommand(rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
        
        rclcpp::Rate rate(10);
        while (rclcpp::ok() && !cmd_handler->IsDone()) {
            if (goal_handle->is_canceling()) {
                cmd_handler->Cancel();
                result->success = false;
                result->finish_code = "kCanceled";
                goal_handle->canceled(result);
                return;
            }

            auto cm_state = robot_->GetControlManagerState();
            if (cm_state.state == rb::ControlManagerState::State::kMajorFault ||
                cm_state.state == rb::ControlManagerState::State::kMinorFault) {
                cmd_handler->Cancel();
                result->success = false;
                result->finish_code = "Fault Detected";
                goal_handle->abort(result);
                return;
            }

            auto feedback = std::make_shared<MultiJointCommand::Feedback>();
            feedback->current_state = "excuting";
            goal_handle->publish_feedback(feedback);
            rate.sleep();
        }

        if (rclcpp::ok()) {
            auto rv = cmd_handler->Get();
            result->success = (rv.finish_code() == rb::RobotCommandFeedback::FinishCode::kOk);
            result->finish_code = this->finish_code_to_string(rv.finish_code());
            if (result->success) goal_handle->succeed(result);
            else goal_handle->abort(result);
        }
    }

    // --- SingleJointCommand Handlers ---
    template <typename ModelType>
    rclcpp_action::GoalResponse RBY1_ROS2_DRIVER<ModelType>::handle_single_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SingleJointCommand::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received SingleJointCommand request");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    template <typename ModelType>
    rclcpp_action::CancelResponse RBY1_ROS2_DRIVER<ModelType>::handle_single_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel SingleJointCommand goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::handle_single_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_single_command, this, _1), goal_handle}.detach();
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::execute_single_command(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SingleJointCommand>> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<SingleJointCommand::Result>();

        if (goal->position.empty() || goal->target_name.empty()) {
            RCLCPP_WARN(this->get_logger(), "Invalid SingleJointCommand data.");
            result->success = false;
            result->finish_code = "Invalid arguments";
            goal_handle->abort(result);
            return;
        }

        if (!robot_->HasEstablishedTimeSync()) robot_->SyncTime();

        double min_time = (goal->minimum_time > 0.01) ? goal->minimum_time : robot_parameter_.minimum_time;

        if (goal->target_name != "torso" && goal->target_name != "right_arm" && 
            goal->target_name != "left_arm" && goal->target_name != "head") {
            RCLCPP_WARN(this->get_logger(), "Unknown target name in SingleJointCommand: %s", goal->target_name.c_str());
            result->success = false;
            result->finish_code = "Unknown target name";
            goal_handle->abort(result);
            return;
        }

        rb::ComponentBasedCommandBuilder component_cmd_builder;
        rb::BodyComponentBasedCommandBuilder body_comp;
        
        apply_builder(body_comp, component_cmd_builder, goal->target_name, goal->position, min_time);

        if (goal->target_name != "head") {
            component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));
        }

        auto cmd_handler = robot_->SendCommand(rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
        
        rclcpp::Rate rate(10);
        while (rclcpp::ok() && !cmd_handler->IsDone()) {
            if (goal_handle->is_canceling()) {
                cmd_handler->Cancel();
                result->success = false;
                result->finish_code = "kCanceled";
                goal_handle->canceled(result);
                return;
            }

            auto cm_state = robot_->GetControlManagerState();
            if (cm_state.state == rb::ControlManagerState::State::kMajorFault ||
                cm_state.state == rb::ControlManagerState::State::kMinorFault) {
                cmd_handler->Cancel();
                result->success = false;
                result->finish_code = "Fault Detected";
                goal_handle->abort(result);
                return;
            }

            auto feedback = std::make_shared<SingleJointCommand::Feedback>();
            feedback->current_state = "excuting";
            goal_handle->publish_feedback(feedback);
            rate.sleep();
        }

        if (rclcpp::ok()) {
            auto rv = cmd_handler->Get();
            result->success = (rv.finish_code() == rb::RobotCommandFeedback::FinishCode::kOk);
            result->finish_code = this->finish_code_to_string(rv.finish_code());
            if (result->success) goal_handle->succeed(result);
            else goal_handle->abort(result);
        }
    }



    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::resize_joint_states(){
        // 고정 DOF 기준으로 resize
        constexpr size_t kTorsoDOF    = 6;
        constexpr size_t kArmDOF      = 7;
        constexpr size_t kHeadDOF     = 2;
        // 바퀴 수: model A → 2, model M → 4
        // constexpr size_t kWheelDOF =
        //     std::is_same_v<ModelType, rb::y1_model::A> ? 2 : 4;

        auto resize_js = [](JointState& js, size_t n) {
            js.name.assign(n, "");
            js.position.assign(n, 0.0);
            js.velocity.assign(n, 0.0);
            js.effort.assign(n, 0.0);
        };

        resize_js(robot_joint_.joint_torso,     kTorsoDOF);
        resize_js(robot_joint_.joint_right_arm,  kArmDOF);
        resize_js(robot_joint_.joint_left_arm,   kArmDOF);
        resize_js(robot_joint_.joint_head,       kHeadDOF);
        //resize_js(robot_joint_.joint_wheel,      kWheelDOF);
    }


    /**
     * @brief 테스트용 함수. 레시피로 저장하고 함수 하나가 그걸 조합해서 커멘드를 짜는 구조
     */
    // struct CommandRecipe {
    //     std::string command_type; 
    //     double minimum_time = 2.0;

    //     std::map<std::string, Eigen::VectorXd> joint_positions;     
    //     std::map<std::string, Eigen::VectorXd> joint_velocities;   
    // };

    // template <typename ModelType>
    // rb::RobotCommandBuilder RBY1_ROS2_DRIVER<ModelType>::create_robot_command(const CommandRecipe& recipe) {
    //     rb::ComponentBasedCommandBuilder component_builder;
    //     rb::BodyComponentBasedCommandBuilder body_builder;

    //     if (recipe.command_type == "joint_position") {
    //         for (auto const& [part, pos] : recipe.joint_positions) {
    //             rb::JointPositionCommandBuilder jpc;
    //             jpc.SetMinimumTime(recipe.minimum_time).SetPosition(pos);
                
    //             if (part == "torso") body_builder.SetTorsoCommand(rb::TorsoCommandBuilder(jpc));
    //             else if (part == "right_arm") body_builder.SetRightArmCommand(rb::ArmCommandBuilder(jpc));
    //             else if (part == "left_arm") body_builder.SetLeftArmCommand(rb::ArmCommandBuilder(jpc));
    //             else if (part == "head") component_builder.SetHeadCommand(rb::HeadCommandBuilder(jpc));
    //         }
    //         component_builder.SetBodyCommand(rb::BodyCommandBuilder(body_builder));
    //     }
    //     else if (recipe.command_type == "joint_velocity") {
    //         //예시
    //     }

    //     return rb::RobotCommandBuilder().SetCommand(component_builder);
    // }

    // Explicit template instantiations
    template class RBY1_ROS2_DRIVER<rb::y1_model::A>;
    template class RBY1_ROS2_DRIVER<rb::y1_model::M>;
}


/*
################### CAUTION ###################
# CAUTION:
# Ensure that the robot has enough surrounding clearance before running this example.
###############################################

# Motion Demo
# This example connects to an RB-Y1 robot, configures the control manager,
# and runs joint position, Cartesian, impedance, optimal control,
# and mixed command demos in sequence. See --help for arguments.
#
# Usage example:
#     python 24_demo_motion.py --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.


import rby1_sdk as rby
import numpy as np
import argparse
from typing import Iterable
import importlib

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej

D2R = np.pi / 180  # Degree to Radian conversion factor
MINIMUM_TIME = 2
LINEAR_VELOCITY_LIMIT = 1.5
ANGULAR_VELOCITY_LIMIT = np.pi * 1.5
ACCELERATION_LIMIT = 1.0
STOP_ORIENTATION_TRACKING_ERROR = 1e-4
STOP_POSITION_TRACKING_ERROR = 1e-3
WEIGHT = 1
STOP_COST = WEIGHT * WEIGHT * 2e-3
MIN_DELTA_COST = WEIGHT * WEIGHT * 2e-3
PATIENCE = 10

def make_transform(r: np.ndarray, t: Iterable[float]) -> np.ndarray:
    """Build a 4x4 homogeneous transform from rotation and translation.

    Args:
        r: 3x3 rotation matrix.
        t: Iterable of 3 floats [x, y, z].

    Returns:
        4x4 homogeneous transform.
    """
    T = np.eye(4)
    T[:3, :3] = r
    T[:3, 3] = np.asarray(t, dtype=float)
    return T

def move_to_pre_control_pose(robot):
    """Move to the pre-control pose before starting the motion."""
    torso = np.array([0.0, 0.1, -0.2, 0.1, 0.0, 0.0])
    right_arm = np.array([0.2, -0.2, 0.0, -1.0, 0, 0.7, 0.0])
    left_arm = np.array([0.2, 0.2, 0.0, -1.0, 0, 0.7, 0.0])
    if not movej(robot, torso=torso, right_arm=right_arm, left_arm=left_arm, minimum_time=5.0):
        exit(1)

def rot_y(angle_rad: float) -> np.ndarray:
    """Rotation matrix about Y-axis.

    Args:
        angle_rad: Rotation angle in radians.

    Returns:
        3x3 rotation numpy array.
    """
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle_rad: float) -> np.ndarray:
    """Rotation matrix about Z-axis.

    Args:
        angle_rad: Rotation angle in radians.

    Returns:
        3x3 rotation numpy array.
    """
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])



def example_joint_position_command_1(robot):
    print("joint position command example 1")

    # Initialize joint positions
    q_joint_torso = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    q_joint_right_arm[3] = -90 * D2R
    q_joint_left_arm[3] = -90 * D2R

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_torso_command(
                rby.JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_torso)
            )
            .set_right_arm_command(
                rby.JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_right_arm)
            )
            .set_left_arm_command(
                rby.JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_left_arm)
            )
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_joint_position_command_2(robot):
    print("joint position command example 2")

    # Define joint positions
    q_joint_torso = np.array([0, 30, -60, 30, 0, 0]) * D2R

    q_joint_right_arm = np.array([-45, -30, 0, -90, 0, 45, 0]) * D2R
    q_joint_left_arm = np.array([-45, 30, 0, -90, 0, 45, 0]) * D2R

    # Combine joint positions
    q = np.concatenate([q_joint_torso, q_joint_right_arm, q_joint_left_arm])

    # Build command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyCommandBuilder().set_command(
                rby.JointPositionCommandBuilder()
                .set_position(q)
                .set_minimum_time(MINIMUM_TIME)
            )
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0

def example_joint_position_command_3(robot):
    print("Joint position command example 3")

    # Define joint angles in degrees and convert to radians
    q_joint_torso = np.array([0, 30, -60, 30, 0, 0]) * D2R

    q_joint_right_arm = np.array([-45, -30, 0, -90, 0, 45, 0]) * D2R
    q_joint_left_arm = np.array([-45, 30, 0, -90, 0, 45, 0]) * D2R

    # Concatenate joint positions
    q = np.concatenate((q_joint_torso, q_joint_right_arm, q_joint_left_arm))

    # Build joint position command
    joint_position_command = (
        rby.JointPositionCommandBuilder().set_position(q).set_minimum_time(MINIMUM_TIME)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(joint_position_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0

def example_cartesian_command_1(robot):
    print("Cartesian command example 1")

    # Define transformation matrices
    T_torso = make_transform(np.eye(3), [0, 0, 1])

    angle = -np.pi / 4
    T_right = make_transform(rot_y(angle), [0.5, -0.3, 1.0])
    T_left = make_transform(rot_y(angle), [0.5, 0.3, 1.0])

    target_link = "link_torso_5"


    # Build command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_torso_command(
                rby.CartesianCommandBuilder()
                .add_target(
                    "base",
                    target_link,
                    T_torso,
                    LINEAR_VELOCITY_LIMIT,
                    ANGULAR_VELOCITY_LIMIT,
                    ACCELERATION_LIMIT,
                )
                .set_minimum_time(MINIMUM_TIME * 2)
                .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
                .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
            )
            .set_right_arm_command(
                rby.CartesianCommandBuilder()
                .add_target(
                    "base",
                    "ee_right",
                    T_right,
                    LINEAR_VELOCITY_LIMIT,
                    ANGULAR_VELOCITY_LIMIT,
                    ACCELERATION_LIMIT,
                )
                .set_minimum_time(MINIMUM_TIME)
                .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(3))
                .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
                .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
            )
            .set_left_arm_command(
                rby.CartesianCommandBuilder()
                .add_target(
                    "base",
                    "ee_left",
                    T_left,
                    LINEAR_VELOCITY_LIMIT,
                    ANGULAR_VELOCITY_LIMIT,
                    ACCELERATION_LIMIT,
                )
                .set_minimum_time(MINIMUM_TIME)
                .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
                .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
            )
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_cartesian_command_2(robot):
    print("Cartesian command example 2")

    # Define transformation matrices
    angle = np.pi / 6
    T_torso = make_transform(rot_y(angle), [0.1, 0, 1.1])
    angle = -np.pi / 2
    T_right = make_transform(rot_y(angle), [0.5, -0.4, 1.2])
    T_left = make_transform(rot_y(angle), [0.5, 0.4, 1.2])

    target_link = "link_torso_5"

    # Build command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.CartesianCommandBuilder()
            .add_target(
                "base",
                target_link,
                T_torso,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_target(
                "base",
                "ee_right",
                T_right,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_target(
                "base",
                "ee_left",
                T_left,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_joint_position_target("right_arm_1", -np.pi / 3)
            .add_joint_position_target("left_arm_1", np.pi / 3)
            .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
            .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
            .set_minimum_time(MINIMUM_TIME)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_cartesian_command_3(robot):
    print("Cartesian command example 3")

    # Define transformation matrices
    angle = np.pi / 6
    T_torso = make_transform(rot_y(angle), [0.1, 0, 1.2])

    angle = -np.pi / 4
    T_right = make_transform(rot_y(angle), [0.35, -0.4, -0.2])
    T_left = make_transform(rot_y(angle), [0.35, 0.4, -0.2])

    target_link = "link_torso_5"

    # Build command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.CartesianCommandBuilder()
            .add_target(
                "base",
                target_link,
                T_torso,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_target(
                "link_torso_5",
                "ee_right",
                T_right,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_target(
                "link_torso_5",
                "ee_left",
                T_left,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_joint_position_target("right_arm_1", -np.pi / 3)
            .add_joint_position_target("left_arm_1", np.pi / 3)
            .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
            .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
            .set_minimum_time(MINIMUM_TIME)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_impedance_control_command_1(robot):
    print("Impedance control command example 1")

    # Define transformation matrices
    angle = np.pi / 6
    T_torso = make_transform(rot_y(angle), [0.1, 0, 1.2])

    angle = -np.pi / 4
    T_right = make_transform(rot_y(angle), [0.35, -0.4, -0.2])
    T_left = make_transform(rot_y(angle), [0.35, 0.4, -0.2])

    target_link = "link_torso_5"

    # Build commands
    torso_command = (
        rby.ImpedanceControlCommandBuilder()
        .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name("base")
        .set_link_name(target_link)
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([100, 100, 100])
        .set_transformation(T_torso)
    )

    right_arm_command = (
        rby.ImpedanceControlCommandBuilder()
        .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name(target_link)
        .set_link_name("ee_right")
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([50, 50, 50])
        .set_damping_ratio(0.85)
        .set_transformation(T_right)
    )

    left_arm_command = (
        rby.ImpedanceControlCommandBuilder()
        .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name(target_link)
        .set_link_name("ee_left")
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([50, 50, 50])
        .set_damping_ratio(0.85)
        .set_transformation(T_left)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_torso_command(torso_command)
            .set_right_arm_command(right_arm_command)
            .set_left_arm_command(left_arm_command)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_relative_command_1(robot):
    print("Relative command example 1")


    # Define transformation matrices
    angle = -np.pi / 4
    T_right = make_transform(rot_y(angle), [0.5, -0.4, 0.9])

    # Build Cartesian command
    right_arm_command = (
        rby.CartesianCommandBuilder()
        .set_minimum_time(MINIMUM_TIME)
        .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
        .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
        .add_target(
            "base",
            "ee_right",
            T_right,
            LINEAR_VELOCITY_LIMIT,
            ANGULAR_VELOCITY_LIMIT,
            ACCELERATION_LIMIT,
        )
        # .add_joint_position_target("right_arm_1", -np.pi/3)
    )

    # Define transformation difference
    T_diff = make_transform(np.eye(3), [0, 0.8, 0])

    # Build Impedance control command
    left_arm_command = (
        rby.ImpedanceControlCommandBuilder()
        .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name("ee_right")
        .set_link_name("ee_left")
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([50, 50, 50])
        .set_damping_ratio(0.85)
        .set_transformation(T_diff)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_right_arm_command(right_arm_command)
            .set_left_arm_command(left_arm_command)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0




def example_optimal_control_1(robot):
    print("Optimal control example 1")

    # Define transformation matrices
    T_torso = make_transform(np.eye(3), [0, 0, 1.0])

    angle = -np.pi / 2
    T_right = make_transform(rot_y(angle), [0.5, -0.2, 1.0])
    T_left = make_transform(rot_y(angle), [0.5, 0.2, 1.0])

    target_link = "link_torso_5"

    # Build optimal control command
    optimal_control_command = (
        rby.OptimalControlCommandBuilder()
        .add_cartesian_target("base", target_link, T_torso, WEIGHT, WEIGHT)
        .add_cartesian_target("base", "ee_right", T_right, WEIGHT, WEIGHT)
        .add_cartesian_target("base", "ee_left", T_left, WEIGHT, WEIGHT)
        .add_joint_position_target("right_arm_2", np.pi / 2, WEIGHT / 5)
        .add_joint_position_target("left_arm_2", -np.pi / 2, WEIGHT / 5)
        .set_velocity_limit_scaling(0.5)
        .set_error_scaling(1.5)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(PATIENCE)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(optimal_control_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_optimal_control_2(robot):
    print("Optimal control example 2")

    # Define transformation matrices
    T_torso = make_transform(np.eye(3), [0, 0, 1.0])

    angle = -np.pi / 2
    T_right = make_transform(rot_y(angle), [0.4, -0.2, 1.0])
    T_left = make_transform(rot_y(angle), [0.4, 0.2, 1.0])

    target_link = "link_torso_5"

    # Build optimal control command
    optimal_control_command = (
        rby.OptimalControlCommandBuilder()
        .add_cartesian_target("base", target_link, T_torso, WEIGHT, WEIGHT)
        .add_cartesian_target("base", "ee_right", T_right, WEIGHT, WEIGHT)
        .add_cartesian_target("base", "ee_left", T_left, WEIGHT, WEIGHT)
        .add_joint_position_target("right_arm_2", 0.05, WEIGHT / 2)
        .add_joint_position_target("left_arm_2", -0.05, WEIGHT / 2)
        .set_velocity_limit_scaling(1)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(PATIENCE)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(optimal_control_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_optimal_control_3(robot):
    print("Optimal control example 3")

    # Define transformation matrices
    T_torso = make_transform(np.eye(3), [0, 0, 0])

    angle = -np.pi / 2
    T_right = make_transform(rot_y(angle), [0.5, -0.3, 1.2])
    T_left = make_transform(rot_y(angle), [0.5, 0.3, 1.2])

    COM = np.array([-0.0, 0.0, 0.47])

    target_link = "link_torso_5"

    # Build optimal control command
    optimal_control_command = (
        rby.OptimalControlCommandBuilder()
        .set_center_of_mass_target("base", COM, WEIGHT * 5)
        .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
        .add_cartesian_target("base", "ee_left", T_left, WEIGHT, WEIGHT)
        .add_cartesian_target("base", "ee_right", T_right, WEIGHT, WEIGHT)
        # .add_joint_position_target("torso_2", -np.pi / 2, WEIGHT / 4)
        # .add_joint_position_target("torso_1", np.pi/4, WEIGHT)
        # .add_joint_position_target("torso_5", 0, WEIGHT)
        .add_joint_position_target("right_arm_2", np.pi / 4, WEIGHT / 20)
        .add_joint_position_target("left_arm_2", -np.pi / 4, WEIGHT / 20)
        .set_velocity_limit_scaling(0.5)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(PATIENCE)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(optimal_control_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_mixed_command_1(robot):
    print("Mixed command example 1")

    # Define transformation matrices
    T_torso = make_transform(np.eye(3), [0, 0, 1])

    target_link = "link_torso_5"
    target_joint = "torso_2"
    torso_command = (
        rby.OptimalControlCommandBuilder()
        .set_center_of_mass_target("base", np.array([0, 0, 0.4]), WEIGHT * 1e-1)
        .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
        .add_joint_position_target(target_joint, -np.pi / 2, WEIGHT)
        .add_joint_position_target("torso_0", 0, WEIGHT)
        .set_stop_cost(STOP_COST * 1e1)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(PATIENCE)
    )


    right_arm_command = (
        rby.JointPositionCommandBuilder()
        .set_position(np.array([0, -np.pi / 4, 0, -np.pi / 2, 0, 0, 0]))
        .set_velocity_limit(np.array([np.pi] * 7))
        .set_acceleration_limit(np.array([1.0] * 7))
        .set_minimum_time(MINIMUM_TIME)
    )

    # Send command
    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_torso_command(torso_command)
                .set_right_arm_command(right_arm_command)
            )
        ),
        10,
    ).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_mixed_command_2(robot):
    print("Mixed command example 2")

    # Define transformation matrices
    angle = np.pi / 6
    T_torso = make_transform(rot_z(angle), [0, 0, 1])

    target_link = "link_torso_5"
    target_joint = "torso_2"
    torso_command = (
        rby.OptimalControlCommandBuilder()
        .set_center_of_mass_target("base", np.array([0, 0, 0.4]), WEIGHT * 1e-1)
        .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
        .add_joint_position_target(target_joint, -np.pi / 2, WEIGHT)
        .add_joint_position_target("torso_0", 0, WEIGHT)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST / 10)
        .set_patience(PATIENCE * 10)
    )

    right_arm_command = (
        rby.JointPositionCommandBuilder()
        .set_position(np.array([0, -np.pi / 4, 0, -np.pi / 2, 0, 0, 0]))
        .set_velocity_limit(np.array([np.pi] * 7))
        .set_acceleration_limit(np.array([1.0] * 7))
        .set_minimum_time(MINIMUM_TIME)
    )

    left_arm_command = (
        rby.GravityCompensationCommandBuilder()
        .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_on(True)
    )

    # Send command
    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_torso_command(torso_command)
                .set_right_arm_command(right_arm_command)
                .set_left_arm_command(left_arm_command)
            )
        ),
        10,
    ).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def go_to_home_pose_1(robot):
    print("Go to home pose 1")

    q_joint_torso = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    q_joint_right_arm[1] = -135 * D2R
    q_joint_left_arm[1] = 135 * D2R

    # Send command to go to ready position
    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_torso_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(MINIMUM_TIME * 2)
                    .set_position(q_joint_torso)
                )
                .set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(MINIMUM_TIME * 2)
                    .set_position(q_joint_right_arm)
                )
                .set_left_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(MINIMUM_TIME * 2)
                    .set_position(q_joint_left_arm)
                )
            )
        ),
        10,
    ).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def go_to_home_pose_2(robot):
    print("Go to home pose 2")

    target_joint = np.zeros(20)

    # Send command to go to home pose
    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.JointPositionCommandBuilder()
                .set_position(target_joint)
                .set_minimum_time(MINIMUM_TIME)
            )
        ),
        10,
    ).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def main(address, model_name, power, servo):
    robot = initialize_robot(address, model_name, power, servo)

    # robot.factory_reset_all_parameters()
    robot.set_parameter("default.acceleration_limit_scaling", "1.0")
    robot.set_parameter("joint_position_command.cutoff_frequency", "5")
    robot.set_parameter("cartesian_command.cutoff_frequency", "5")
    robot.set_parameter("default.linear_acceleration_limit", "20")
    robot.set_parameter("default.angular_acceleration_limit", "10")
    robot.set_parameter("manipulability_threshold", "1e4")
    # robot.set_time_scale(1.0)

    print("parameters setting is done")

    move_to_pre_control_pose(robot)

    if not example_joint_position_command_1(robot):
        print("finish motion")
    if not example_joint_position_command_2(robot):
        print("finish motion")
    if not example_cartesian_command_1(robot):
        print("finish motion")
    if not example_cartesian_command_2(robot):
        print("finish motion")
    if not example_cartesian_command_3(robot):
        print("finish motion")
    if not example_impedance_control_command_1(robot):
        print("finish motion")
    if not example_relative_command_1(robot):
        print("finish motion")
    if not example_joint_position_command_3(robot):
        print("finish motion")
    if not example_optimal_control_1(robot):
        print("finish motion")
    if not example_optimal_control_2(robot):
        print("finish motion")
    if not example_optimal_control_3(robot):
        print("finish motion")
    if not example_mixed_command_1(robot):
        print("finish motion")
    if not example_mixed_command_2(robot):
        print("finish motion")
    # if not go_to_home_pose_1(robot):
    #     print("finish motion")
    if not go_to_home_pose_2(robot):
        print("finish motion")

    print("end of demo")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="24_demo_motion")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument("--model", type=str, default='a', help="Robot Model Name (default: 'a')")
    parser.add_argument(
        "--power",
        type=str,
        default=".*",
        help="Power device name regex pattern (default: '.*')",
    )
    parser.add_argument(
        "--servo",
        type=str,
        default=".*",
        help="Servo name regex pattern (default: '.*')",
    )
    args = parser.parse_args()

    main(address=args.address, model_name = args.model, power=args.power, servo=args.servo)

*/