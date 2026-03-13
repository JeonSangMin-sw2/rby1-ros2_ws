#include "rby1_ros2_driver.hpp"
namespace rby1_ros2{
    //using namespace rb;


    template <typename ModelType>
    RBY1_ROS2_DRIVER<ModelType>::RBY1_ROS2_DRIVER()
        : Node("rby1_ros2_driver"){
            
            //declare parameter from yaml
            init_parameter();
            if (address == "" || model == ""){
                RCLCPP_ERROR(this->get_logger(), "address or model isn't declared");
            }
            try{
                robot_ = rb::Robot<ModelType>::Create(address);
                robot_->Connect();
                
                // Fetch robot info once and cache it
                info_ = robot_->GetRobotInfo();
                categorize_joints();
                
                // Initialize joint state containers and publishers
                resize_joint_states();
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
            power_on(power_on_list_);   
            servo_on(servo_on_list_);
            check_controll_manager();

            robot_->SetParameter("default.acceleration_limit_scaling", std::to_string(acceleration_limit));
            robot_->SetParameter("joint_position_command.cutoff_frequency", std::to_string(angular_velocity_limit));
            robot_->SetParameter("cartesian_command.cutoff_frequency", std::to_string(linear_velocity_limit));
            robot_->SetParameter("default.linear_acceleration_limit", std::to_string(acceleration_limit));
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
        this->declare_parameter<std::vector<int64_t>>("power_on", {5,12,24,48});
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
        this->get_parameter("power_on", power_on_list_);
        this->get_parameter("servo_on", servo_on_list_);
        this->get_parameter("minimum_time", minimum_time);
        this->get_parameter("angular_velocity_limit", angular_velocity_limit);
        this->get_parameter("linear_velocity_limit", linear_velocity_limit);
        this->get_parameter("acceleration_limit", acceleration_limit);
        this->get_parameter("stop_orientation_tracking_error", stop_orientation_tracking_error);
        this->get_parameter("stop_position_tracking_error", stop_position_tracking_error);
        this->get_parameter("fault_reset_trigger", fault_reset_trigger);
        this->get_parameter("node_power_off_trigger", node_power_off_trigger);
    }

    template <typename ModelType>
    bool RBY1_ROS2_DRIVER<ModelType>::power_on(std::vector<int64_t> power_list){
        // Check for invalid values in power_list
        const std::vector<int64_t> allowed_values = {5, 12, 24, 48};
        if (address == "127.0.0.1:50051"){
            power_list_str = ".*";
        }else{
            for (int64_t p : power_list) {
                if (std::find(allowed_values.begin(), allowed_values.end(), p) == allowed_values.end()) {
                    RCLCPP_ERROR(this->get_logger(), "[power on]Invalid power value: %ld. Allowed values are 5, 12, 24, 48.", p);
                    return false;
                }
            }
            if((int)power_list.size() != 0){
                for (size_t i = 0; i < power_list.size(); i++) {
                    power_list_str += std::to_string(power_list[i]);
                    if (i != power_list.size() - 1){
                        power_list_str += "v|";
                    }else{
                        power_list_str += "v";
                    }
                }
            }
        }
        RCLCPP_INFO(this->get_logger(),"power on [%s]", power_list_str.c_str());
        if (!robot_->IsPowerOn(power_list_str) && !robot_->PowerOn(power_list_str)) return false;
        return true;
    }

    template <typename ModelType>
    bool RBY1_ROS2_DRIVER<ModelType>::power_off(std::vector<int64_t> power_list){
        const std::vector<int64_t> allowed_values = {5, 12, 24, 48};
        if (address == "127.0.0.1:50051"){
            power_list_str = ".*";
        }else{
            for (int64_t p : power_list) {
                if (std::find(allowed_values.begin(), allowed_values.end(), p) == allowed_values.end()) {
                    RCLCPP_ERROR(this->get_logger(), "[power off]Invalid power value: %ld. Allowed values are 5, 12, 24, 48.", p);
                    return false;
                }
            }
            for (size_t i = 0; i < power_list.size(); i++) {
                power_list_str += std::to_string(power_list[i]);
                if (i != power_list.size() - 1){
                    power_list_str += "v|";
                }else{
                    power_list_str += "v";
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
            categorize_joints(); // 조인트 분류 수행
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
            categorize_joints();
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
        RCLCPP_INFO(this->get_logger(), "Control Manager enabled successfully.");
        return true;
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::read_joint_state(){
        if (info_.joint_infos.empty()) return; // info가 아직 오지 않았으면 리턴
        
        auto state = robot_->GetState();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto now = this->now();
            
            // Fill torso joint state
            robot_state_.joint_torso.header.stamp = now;
            for (size_t i = 0; i < info_.torso_joint_idx.size(); ++i) {
                int idx = info_.torso_joint_idx[i];
                robot_state_.joint_torso.name[i] = info_.joint_infos[idx].name;
                robot_state_.joint_torso.position[i] = state.position[idx];
                robot_state_.joint_torso.velocity[i] = state.velocity[idx];
                robot_state_.joint_torso.effort[i] = state.torque[idx];
            }
            
            // Fill right arm joint state
            robot_state_.joint_right_arm.header.stamp = now;
            for (size_t i = 0; i < info_.right_arm_joint_idx.size(); ++i) {
                int idx = info_.right_arm_joint_idx[i];
                robot_state_.joint_right_arm.name[i] = info_.joint_infos[idx].name;
                robot_state_.joint_right_arm.position[i] = state.position[idx];
                robot_state_.joint_right_arm.velocity[i] = state.velocity[idx];
                robot_state_.joint_right_arm.effort[i] = state.torque[idx];
            }
            
            // Fill left arm joint state
            robot_state_.joint_left_arm.header.stamp = now;
            for (size_t i = 0; i < info_.left_arm_joint_idx.size(); ++i) {
                int idx = info_.left_arm_joint_idx[i];
                robot_state_.joint_left_arm.name[i] = info_.joint_infos[idx].name;
                robot_state_.joint_left_arm.position[i] = state.position[idx];
                robot_state_.joint_left_arm.velocity[i] = state.velocity[idx];
                robot_state_.joint_left_arm.effort[i] = state.torque[idx];
            }
            
            // Fill head joint state
            robot_state_.joint_head.header.stamp = now;
            for (size_t i = 0; i < info_.head_joint_idx.size(); ++i) {
                int idx = info_.head_joint_idx[i];
                robot_state_.joint_head.name[i] = info_.joint_infos[idx].name;
                robot_state_.joint_head.position[i] = state.position[idx];
                robot_state_.joint_head.velocity[i] = state.velocity[idx];
                robot_state_.joint_head.effort[i] = state.torque[idx];
            }
            
            // Mutex 범위 내에서 publish (가장 안전한 방법)
            torso_pub_->publish(robot_state_.joint_torso);
            right_arm_pub_->publish(robot_state_.joint_right_arm);
            left_arm_pub_->publish(robot_state_.joint_left_arm);
            head_pub_->publish(robot_state_.joint_head);
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::position_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
        // name 에서 , 가 있는지 확인하고 있으면 분리해서 어떤 파트들이 들어왔는지 확인
        // 각 파트에 맞는 순서로 position을 잘라내서 저장한 후 커멘드에 입력
        std::vector<double> q_joint_torso;
        std::vector<double> q_joint_right_arm;
        std::vector<double> q_joint_left_arm;
        std::vector<double> q_joint_head;
        
        bool use_torso = false, use_right_arm = false, use_left_arm = false, use_head = false;
        size_t current_idx = 0;

        for (const auto& raw_name : msg->name) {
            std::stringstream ss(raw_name);
            std::string part;
            while(std::getline(ss, part, ',')) {
                if (part == "torso") {
                    if (current_idx + info_.torso_joint_idx.size() <= msg->position.size()) {
                        q_joint_torso.assign(msg->position.begin() + current_idx, msg->position.begin() + current_idx + info_.torso_joint_idx.size());
                        current_idx += info_.torso_joint_idx.size();
                        use_torso = true;
                    }
                } else if (part == "right_arm") {
                    if (current_idx + info_.right_arm_joint_idx.size() <= msg->position.size()) {
                        q_joint_right_arm.assign(msg->position.begin() + current_idx, msg->position.begin() + current_idx + info_.right_arm_joint_idx.size());
                        current_idx += info_.right_arm_joint_idx.size();
                        use_right_arm = true;
                    }
                } else if (part == "left_arm") {
                    if (current_idx + info_.left_arm_joint_idx.size() <= msg->position.size()) {
                        q_joint_left_arm.assign(msg->position.begin() + current_idx, msg->position.begin() + current_idx + info_.left_arm_joint_idx.size());
                        current_idx += info_.left_arm_joint_idx.size();
                        use_left_arm = true;
                    }
                } else if (part == "head") {
                    if (current_idx + info_.head_joint_idx.size() <= msg->position.size()) {
                        q_joint_head.assign(msg->position.begin() + current_idx, msg->position.begin() + current_idx + info_.head_joint_idx.size());
                        current_idx += info_.head_joint_idx.size();
                        use_head = true;
                    }
                }
            }
        }

        if (!use_torso && !use_right_arm && !use_left_arm && !use_head) {
            RCLCPP_WARN(this->get_logger(), "No valid component names found in position_command message.");
            return;
        }

        auto body_cmd_builder = rb::BodyComponentBasedCommandBuilder();
        if (use_torso) body_cmd_builder.SetTorsoCommand(rb::JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(Eigen::Map<const Eigen::VectorXd>(q_joint_torso.data(), q_joint_torso.size())));
        if (use_right_arm) body_cmd_builder.SetRightArmCommand(rb::JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(Eigen::Map<const Eigen::VectorXd>(q_joint_right_arm.data(), q_joint_right_arm.size())));
        if (use_left_arm) body_cmd_builder.SetLeftArmCommand(rb::JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(Eigen::Map<const Eigen::VectorXd>(q_joint_left_arm.data(), q_joint_left_arm.size())));

        auto component_cmd_builder = rb::ComponentBasedCommandBuilder();
        component_cmd_builder.SetBodyCommand(body_cmd_builder);
        if (use_head) {
            component_cmd_builder.SetHeadCommand(rb::HeadCommandBuilder().SetCommand(rb::JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(Eigen::Map<const Eigen::VectorXd>(q_joint_head.data(), q_joint_head.size()))));
        }

        auto rv =
        robot_
            ->SendCommand(rb::RobotCommandBuilder().SetCommand(component_cmd_builder))
            ->Get();

        if (rv.finish_code() != rb::RobotCommandFeedback::FinishCode::kOk) {
            RCLCPP_ERROR(this->get_logger(), "Error: Failed to conduct position command. FinishCode: %d", (int)rv.finish_code());
        }
    }


    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::categorize_joints(){
        // SDK에서 자동으로 채워주지 않을 경우를 대비해 수동으로 분류
        info_.torso_joint_idx.clear();
        info_.right_arm_joint_idx.clear();
        info_.left_arm_joint_idx.clear();
        info_.head_joint_idx.clear();

        for (size_t i = 0; i < info_.joint_infos.size(); ++i) {
            const std::string& name = info_.joint_infos[i].name;
            if (name.find("torso_") != std::string::npos) {
                info_.torso_joint_idx.push_back(i);
            } else if (name.find("right_arm_") != std::string::npos) {
                info_.right_arm_joint_idx.push_back(i);
            } else if (name.find("left_arm_") != std::string::npos) {
                info_.left_arm_joint_idx.push_back(i);
            } else if (name.find("head_") != std::string::npos) {
                info_.head_joint_idx.push_back(i);
            }
        }
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::resize_joint_states(){
        // info의 실제 인덱스 벡터 크기에 맞춰 resize 수행 (하드코딩 제거)
        robot_state_.joint_torso.name.resize(info_.torso_joint_idx.size());
        robot_state_.joint_torso.position.resize(info_.torso_joint_idx.size());
        robot_state_.joint_torso.velocity.resize(info_.torso_joint_idx.size());
        robot_state_.joint_torso.effort.resize(info_.torso_joint_idx.size());

        robot_state_.joint_right_arm.name.resize(info_.right_arm_joint_idx.size());
        robot_state_.joint_right_arm.position.resize(info_.right_arm_joint_idx.size());
        robot_state_.joint_right_arm.velocity.resize(info_.right_arm_joint_idx.size());
        robot_state_.joint_right_arm.effort.resize(info_.right_arm_joint_idx.size());

        robot_state_.joint_left_arm.name.resize(info_.left_arm_joint_idx.size());
        robot_state_.joint_left_arm.position.resize(info_.left_arm_joint_idx.size());
        robot_state_.joint_left_arm.velocity.resize(info_.left_arm_joint_idx.size());
        robot_state_.joint_left_arm.effort.resize(info_.left_arm_joint_idx.size());

        robot_state_.joint_head.name.resize(info_.head_joint_idx.size());
        robot_state_.joint_head.position.resize(info_.head_joint_idx.size());
        robot_state_.joint_head.velocity.resize(info_.head_joint_idx.size());
        robot_state_.joint_head.effort.resize(info_.head_joint_idx.size());
    }

    // Explicit template instantiations
    template class RBY1_ROS2_DRIVER<rb::y1_model::A>;
    template class RBY1_ROS2_DRIVER<rb::y1_model::M>;
}