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
                
                torso_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_topic_name + "/torso", 10);
                right_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_topic_name + "/right_arm", 10);
                left_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_topic_name + "/left_arm", 10);
                head_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_topic_name + "/head", 10);
                position_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_topic_name + "/position_command", 10, std::bind(&RBY1_ROS2_DRIVER<ModelType>::position_command_callback, this, std::placeholders::_1));
                // Timer for 100Hz publishing (10ms)
                joint_state_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&RBY1_ROS2_DRIVER<ModelType>::read_joint_state, this));
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "에러 발생: %s", e.what());
            }

            //test code
            power_on(robot_parameter_.power_on_list);   
            servo_on(robot_parameter_.servo_on_list);
            check_controll_manager();

            
            //power_off();
    }

    template <typename ModelType>
    RBY1_ROS2_DRIVER<ModelType>::~RBY1_ROS2_DRIVER(){
        power_off();
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::init_parameter(){
        RCLCPP_INFO(this->get_logger(), "Declaring parameters...");
        this->declare_parameter<std::string>("robot_ip", "127.0.0.1:50051");
        this->declare_parameter<std::string>("model", "a");
        this->declare_parameter<std::string>("joint_topic_name", "joint_states");
        this->declare_parameter<std::vector<std::string>>("power_on", {"all"});
        this->declare_parameter<std::vector<std::string>>("servo_on", {"all"});

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
        this->get_parameter("joint_topic_name", joint_topic_name);
        this->get_parameter("power_on", robot_parameter_.power_on_list);
        this->get_parameter("servo_on", robot_parameter_.servo_on_list);
        this->get_parameter("minimum_time", robot_parameter_.minimum_time);
        this->get_parameter("angular_velocity_limit", robot_parameter_.angular_velocity_limit);
        this->get_parameter("linear_velocity_limit", robot_parameter_.linear_velocity_limit);
        this->get_parameter("acceleration_limit", robot_parameter_.acceleration_limit);
        this->get_parameter("stop_orientation_tracking_error", robot_parameter_.stop_orientation_tracking_error);
        this->get_parameter("stop_position_tracking_error", robot_parameter_.stop_position_tracking_error);
        this->get_parameter("fault_reset_trigger", fault_reset_trigger);
        this->get_parameter("node_power_off_trigger", node_power_off_trigger);

        if (address == "" || model == ""){
            RCLCPP_ERROR(this->get_logger(), "address or model isn't declared");
            rclcpp::shutdown();
        }
    }

    template <typename ModelType>
    bool RBY1_ROS2_DRIVER<ModelType>::power_on(std::vector<std::string> power_list){
        power_list_str = "";
        if (address == "127.0.0.1:50051"){
            power_list_str = ".*";
        }else{
            for (size_t i = 0; i < power_list.size(); i++) {
                if (power_list[i] == "all" || power_list[i] == ".*") {
                    power_list_str = ".*";
                    break;
                }
                power_list_str += power_list[i];
                if (power_list[i].find('v') == std::string::npos && power_list[i] != ".*") {
                    power_list_str += "v";
                }
                if (i != power_list.size() - 1){
                    power_list_str += "|";
                }
            }
        }
        RCLCPP_INFO(this->get_logger(),"power on [%s]", power_list_str.c_str());
        if (!robot_->IsPowerOn(power_list_str) && !robot_->PowerOn(power_list_str)) return false;
        return true;
    }

    template <typename ModelType>
    bool RBY1_ROS2_DRIVER<ModelType>::power_off(std::vector<std::string> power_list){
        power_list_str = "";
        if (address == "127.0.0.1:50051"){
            power_list_str = ".*";
        }else{
            for (size_t i = 0; i < power_list.size(); i++) {
                if (power_list[i] == "all" || power_list[i] == ".*") {
                    power_list_str = ".*";
                    break;
                }
                power_list_str += power_list[i];
                if (power_list[i].find('v') == std::string::npos && power_list[i] != ".*") {
                    power_list_str += "v";
                }
                if (i != power_list.size() - 1){
                    power_list_str += "|";
                }
            }
        }
        RCLCPP_INFO(this->get_logger(),"power off [%s]", power_list_str.c_str());
        if (!robot_->PowerOff(power_list_str)) return false;
        return true;
    }

    template <typename ModelType>
    bool RBY1_ROS2_DRIVER<ModelType>::servo_on(std::vector<std::string> servo_list){
        for (std::string name : servo_list) {
            if(name == "right"){
                servo_list_str += "^right_arm_.*";
            }else if(name == "left"){
                servo_list_str += "^left_arm_.*";
            }else if(name == "head"){
                servo_list_str += "^head_.*";
            }else if(name == "torso"){
                servo_list_str += "^torso_.*";
            }else if(name == "all" || name == ".*"){
                servo_list_str = ".*";
                break;
            }else{
                servo_list_str += name;
            }
            if (name != servo_list.back()){
                servo_list_str += "|";
            }
        }
        RCLCPP_INFO(this->get_logger(),"servo on [%s]", servo_list_str.c_str());
        if (!robot_->IsServoOn(servo_list_str) && !robot_->ServoOn(servo_list_str)) return false;
        
        {
            std::lock_guard<std::mutex> lock(mutex_);
            info_ = robot_->GetRobotInfo();
            //categorize_joints(); // 조인트 분류 수행
            resize_joint_states();
        }

        auto state = robot_->GetState();
        std::vector<std::string> servo_on_joints;
        // T::kRobotDOF는 모델의 총 관절 개수를 의미합니다 (보통 info.joint_infos.size()와 동일)
        for (size_t i = 0; i < info_.joint_infos.size(); ++i) {
            if (state.is_ready[i]) { // 해당 인덱스의 관절이 서보온 상태(ready) 라면
                servo_on_joints.push_back(info_.joint_infos[i].name);
            }
        }
        // 출력해보기
        std::cout << "--- Servo On Joints ---" << std::endl;
        for (const auto& name : servo_on_joints) {
            std::cout << name << std::endl;
        }
        return true;
    }

    template <typename ModelType>
    bool RBY1_ROS2_DRIVER<ModelType>::servo_on(std::string servo_name){
        if(servo_name == "all" || servo_name == ".*"){
            servo_list_str = ".*";
        }else if(servo_name == "right"){
            servo_list_str = "^right_arm_.*";
        }else if(servo_name == "left"){
            servo_list_str = "^left_arm_.*";
        }else if(servo_name == "head"){
            servo_list_str = "^head_.*";
        }else if(servo_name == "torso"){
            servo_list_str = "^torso_.*";
        }else{
            servo_list_str = servo_name;
        }
        if (!robot_->IsServoOn(servo_list_str) && !robot_->ServoOn(servo_list_str)) return false;
        
        {
            std::lock_guard<std::mutex> lock(mutex_);
            info_ = robot_->GetRobotInfo();
            //categorize_joints();
            resize_joint_states();
        }

        auto state = robot_->GetState();
        std::vector<std::string> servo_on_joints;
        // T::kRobotDOF는 모델의 총 관절 개수를 의미합니다 (보통 info.joint_infos.size()와 동일)
        for (size_t i = 0; i < info_.joint_infos.size(); ++i) {
            if (state.is_ready[i]) { // 해당 인덱스의 관절이 서보온 상태(ready) 라면
                servo_on_joints.push_back(info_.joint_infos[i].name);
            }
        }
        // 출력해보기
        std::cout << "--- Servo On Joints ---" << std::endl;
        for (const auto& name : servo_on_joints) {
            std::cout << name << std::endl;
        }
        return true;
    }

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
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto now = this->now();
            
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

            fill(robot_joint_.joint_torso,     info_.torso_joint_idx);
            fill(robot_joint_.joint_right_arm,  info_.right_arm_joint_idx);
            fill(robot_joint_.joint_left_arm,   info_.left_arm_joint_idx);
            fill(robot_joint_.joint_head,       info_.head_joint_idx);
            //fill(robot_joint_.joint_wheel,      info_.mobility_joint_idx);

            torso_pub_->publish(robot_joint_.joint_torso);
            right_arm_pub_->publish(robot_joint_.joint_right_arm);
            left_arm_pub_->publish(robot_joint_.joint_left_arm);
            head_pub_->publish(robot_joint_.joint_head);
           // wheel_pub_->publish(robot_joint_.joint_wheel);
        }
    }


    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::position_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
        // name 에서 , 가 있는지 확인하고 있으면 분리해서 어떤 파트들이 들어왔는지 확인
        // 각 파트에 맞는 순서로 position을 잘라내서 저장한 후 커멘드에 입력
        Eigen::Vector<double, 6> q_joint_torso;
        Eigen::Vector<double, 7> q_joint_right_arm;
        Eigen::Vector<double, 7> q_joint_left_arm;
        Eigen::Vector<double, 2> q_joint_head;
        
        bool use_torso = false, use_right_arm = false, use_left_arm = false, use_head = false;
        size_t current_idx = 0;

        for (const auto& raw_name : msg->name) {
            std::stringstream ss(raw_name);
            std::string part;
            while(std::getline(ss, part, ',')) {
                if (part == "torso") {
                    if (current_idx + info_.torso_joint_idx.size() <= msg->position.size()) {
                        for (size_t i = 0; i < info_.torso_joint_idx.size(); ++i) {
                            q_joint_torso[i] = msg->position[current_idx + i];
                        }
                        current_idx += info_.torso_joint_idx.size();
                        use_torso = true;
                        RCLCPP_INFO(this->get_logger(), "Torso command received, value: %f,%f,%f,%f,%f,%f", q_joint_torso[0], q_joint_torso[1], q_joint_torso[2], q_joint_torso[3], q_joint_torso[4], q_joint_torso[5]);
                    }
                } else if (part == "right_arm") {
                    if (current_idx + info_.right_arm_joint_idx.size() <= msg->position.size()) {
                        for (size_t i = 0; i < info_.right_arm_joint_idx.size(); ++i) {
                            q_joint_right_arm[i] = msg->position[current_idx + i];
                        }
                        current_idx += info_.right_arm_joint_idx.size();
                        use_right_arm = true;
                        RCLCPP_INFO(this->get_logger(), "Right arm command received, value: %f,%f,%f,%f,%f,%f,%f", q_joint_right_arm[0], q_joint_right_arm[1], q_joint_right_arm[2], q_joint_right_arm[3], q_joint_right_arm[4], q_joint_right_arm[5], q_joint_right_arm[6]);
                    }
                } else if (part == "left_arm") {
                    if (current_idx + info_.left_arm_joint_idx.size() <= msg->position.size()) {
                        for (size_t i = 0; i < info_.left_arm_joint_idx.size(); ++i) {
                            q_joint_left_arm[i] = msg->position[current_idx + i];
                        }
                        current_idx += info_.left_arm_joint_idx.size();
                        use_left_arm = true;
                        RCLCPP_INFO(this->get_logger(), "Left arm command received, value: %f,%f,%f,%f,%f,%f,%f", q_joint_left_arm[0], q_joint_left_arm[1], q_joint_left_arm[2], q_joint_left_arm[3], q_joint_left_arm[4], q_joint_left_arm[5], q_joint_left_arm[6]);
                    }
                } else if (part == "head") {
                    if (current_idx + info_.head_joint_idx.size() <= msg->position.size()) {
                        for (size_t i = 0; i < info_.head_joint_idx.size(); ++i) {
                            q_joint_head[i] = msg->position[current_idx + i];
                        }
                        current_idx += info_.head_joint_idx.size();
                        use_head = true;
                        RCLCPP_INFO(this->get_logger(), "Head command received, value: %f,%f", q_joint_head[0], q_joint_head[1]);
                    }
                }
            }
        }

        if (!use_torso && !use_right_arm && !use_left_arm && !use_head) {
            RCLCPP_WARN(this->get_logger(), "No valid component names found in position_command message.");
            return;
        }

        // Handle Head (2 joints) separately
        if (use_head) {
            rb::ComponentBasedCommandBuilder head_command;
            head_command.SetHeadCommand(rb::HeadCommandBuilder(
                rb::JointPositionCommandBuilder()
                    .SetMinimumTime(robot_parameter_.minimum_time)
                    .SetPosition(q_joint_head)));
            
            // Note: Currently whole command context is for Body. 
            // If you want to send Head at the same time, we should use a single ComponentBasedCommandBuilder.
        }

        auto status = robot_->GetControlManagerState();
        RCLCPP_INFO(this->get_logger(), "Sending command... Control Manager state: %d, Control state: %d", (int)status.state, (int)status.control_state);

        // Ensure time sync if necessary
        if (!robot_->HasEstablishedTimeSync()) {
            robot_->SyncTime();
        }

        rb::ComponentBasedCommandBuilder component_cmd_builder;
        if (use_torso || use_right_arm || use_left_arm) {
             rb::BodyComponentBasedCommandBuilder body_comp;
             if (use_torso) body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(rb::JointPositionCommandBuilder().SetMinimumTime(robot_parameter_.minimum_time).SetPosition(q_joint_torso)));
             if (use_right_arm) body_comp.SetRightArmCommand(rb::ArmCommandBuilder(rb::JointPositionCommandBuilder().SetMinimumTime(robot_parameter_.minimum_time).SetPosition(q_joint_right_arm)));
             if (use_left_arm) body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(rb::JointPositionCommandBuilder().SetMinimumTime(robot_parameter_.minimum_time).SetPosition(q_joint_left_arm)));
             component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));
        }
        if (use_head) {
            component_cmd_builder.SetHeadCommand(rb::HeadCommandBuilder(rb::JointPositionCommandBuilder().SetMinimumTime(robot_parameter_.minimum_time).SetPosition(q_joint_head)));
        }

        auto rv = robot_->SendCommand(rb::RobotCommandBuilder().SetCommand(component_cmd_builder))->Get();

        if (rv.finish_code() != rb::RobotCommandFeedback::FinishCode::kOk) {
            RCLCPP_ERROR(this->get_logger(), "Error: Failed to conduct position command. FinishCode: %s", this->finish_code_to_string(rv.finish_code()).c_str());
            RCLCPP_ERROR(this->get_logger(), "Feedback status: %d (0:Idle, 1:Init, 2:Running, 3:Finished)", (int)rv.status());
        } else {
            RCLCPP_INFO(this->get_logger(), "Command executed successfully.");
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

    // =========================================================================
    // [TEST ONLY - DRAFT] Flexible Command Builder Factory Proposal
    // These structures demonstrate how to handle various command types efficiently.
    // To use these, you should move the struct to hpp and add a member function declaration.
    // =========================================================================

    /**
     * @brief A structure representing a 'Recipe' for a robot command.
     */
    struct CommandRecipe {
        std::string command_type; // e.g., "joint_position", "cartesian", "velocity", "jog"
        double minimum_time = 2.0;
        
        // Data maps for different command flavors
        std::map<std::string, Eigen::VectorXd> joint_positions;     // part_name -> position_vector
        std::map<std::string, Eigen::VectorXd> joint_velocities;    // part_name -> velocity_vector
        
        // Add more parameters (e.g., CartesianTarget) here for extension.
    };

    /**
     * @brief Draft of a factory function that constructs a RobotCommandBuilder.
     */
    template <typename ModelType>
    rb::RobotCommandBuilder RBY1_ROS2_DRIVER<ModelType>::create_robot_command(const CommandRecipe& recipe) {
        rb::ComponentBasedCommandBuilder component_builder;
        rb::BodyComponentBasedCommandBuilder body_builder;

        if (recipe.command_type == "joint_position") {
            for (auto const& [part, pos] : recipe.joint_positions) {
                rb::JointPositionCommandBuilder jpc;
                jpc.SetMinimumTime(recipe.minimum_time).SetPosition(pos);
                
                if (part == "torso") body_builder.SetTorsoCommand(rb::TorsoCommandBuilder(jpc));
                else if (part == "right_arm") body_builder.SetRightArmCommand(rb::ArmCommandBuilder(jpc));
                else if (part == "left_arm") body_builder.SetLeftArmCommand(rb::ArmCommandBuilder(jpc));
                else if (part == "head") component_builder.SetHeadCommand(rb::HeadCommandBuilder(jpc));
            }
            component_builder.SetBodyCommand(rb::BodyCommandBuilder(body_builder));
        }
        else if (recipe.command_type == "joint_velocity") {
            // Placeholder: for (auto const& [part, vel] : recipe.joint_velocities) { ... }
        }

        return rb::RobotCommandBuilder().SetCommand(component_builder);
    }

    // Explicit template instantiations
    template class RBY1_ROS2_DRIVER<rb::y1_model::A>;
    template class RBY1_ROS2_DRIVER<rb::y1_model::M>;
}