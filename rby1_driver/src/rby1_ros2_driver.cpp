#include "rby1_ros2_driver.hpp"

namespace rby1_ros2{

    template <typename ModelType>
    RBY1_ROS2_DRIVER<ModelType>::RBY1_ROS2_DRIVER()
        : Node("rby1_ros2_driver"){
            
            //declare parameter from yaml
            get_parameters();
            if (address == "" || model == ""){
                RCLCPP_ERROR(this->get_logger(), "address or model isn't declared");
            }
            try{
                robot_ = rb::Robot<ModelType>::Create(address);
                robot_->Connect();
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


            // read_joint_state();
            // for (int i = 0; i < robot_state_.joint_torso.position.size(); i++) {
            //     RCLCPP_INFO(this->get_logger(), "joint_torso.position[%d]: %f", i, robot_state_.joint_torso.position[i]);
            // }
            // for (int i = 0; i < robot_state_.joint_right_arm.position.size(); i++) {
            //     RCLCPP_INFO(this->get_logger(), "joint_right_arm.position[%d]: %f", i, robot_state_.joint_right_arm.position[i]);
            // }
            // for (int i = 0; i < robot_state_.joint_left_arm.position.size(); i++) {
            //     RCLCPP_INFO(this->get_logger(), "joint_left_arm.position[%d]: %f", i, robot_state_.joint_left_arm.position[i]);
            // }
            // for (int i = 0; i < robot_state_.joint_head.position.size(); i++) {
            //     RCLCPP_INFO(this->get_logger(), "joint_head.position[%d]: %f", i, robot_state_.joint_head.position[i]);
            // }
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            //std::cout << "off" << std::endl;
            power_off();
    }

    template <typename ModelType>
    RBY1_ROS2_DRIVER<ModelType>::~RBY1_ROS2_DRIVER(){
        power_off();
    }

    template <typename ModelType>
    void RBY1_ROS2_DRIVER<ModelType>::get_parameters(){
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
            }else{
                servo_list_str += name;
            }
            if (name != servo_list.back()){
                servo_list_str += "|";
            }
        }
        RCLCPP_INFO(this->get_logger(),"servo on [%s]", servo_list_str.c_str());
        if (!robot_->IsServoOn(servo_list_str) && !robot_->ServoOn(servo_list_str)) return false;
        auto info = robot_->GetRobotInfo();
        auto state = robot_->GetState();
        std::vector<std::string> servo_on_joints;
        // T::kRobotDOF는 모델의 총 관절 개수를 의미합니다 (보통 info.joint_infos.size()와 동일)
        for (size_t i = 0; i < info.joint_infos.size(); ++i) {
            if (state.is_ready[i]) { // 해당 인덱스의 관절이 서보온 상태(ready) 라면
                servo_on_joints.push_back(info.joint_infos[i].name);
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
        auto info = robot_->GetRobotInfo();
        auto state = robot_->GetState();
        std::vector<std::string> servo_on_joints;
        // T::kRobotDOF는 모델의 총 관절 개수를 의미합니다 (보통 info.joint_infos.size()와 동일)
        for (size_t i = 0; i < info.joint_infos.size(); ++i) {
            if (state.is_ready[i]) { // 해당 인덱스의 관절이 서보온 상태(ready) 라면
                servo_on_joints.push_back(info.joint_infos[i].name);
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
        auto state = robot_->GetState();
        auto info = robot_->GetRobotInfo();
        JointState torso_joint_state;
        JointState right_arm_joint_state;
        JointState left_arm_joint_state;
        JointState head_joint_state;
        for (int idx : info.torso_joint_idx) {
            torso_joint_state.position.push_back(state.position[idx]);
            torso_joint_state.velocity.push_back(state.velocity[idx]);
            torso_joint_state.effort.push_back(state.torque[idx]);
        }
        for (int idx : info.right_arm_joint_idx) {
            right_arm_joint_state.position.push_back(state.position[idx]);
            right_arm_joint_state.velocity.push_back(state.velocity[idx]);
            right_arm_joint_state.effort.push_back(state.torque[idx]);
        }
        for (int idx : info.left_arm_joint_idx) {
            left_arm_joint_state.position.push_back(state.position[idx]);
            left_arm_joint_state.velocity.push_back(state.velocity[idx]);
            left_arm_joint_state.effort.push_back(state.torque[idx]);
        }
        for (int idx : info.head_joint_idx) {
            head_joint_state.position.push_back(state.position[idx]);
            head_joint_state.velocity.push_back(state.velocity[idx]);
            head_joint_state.effort.push_back(state.torque[idx]);
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            robot_state_.joint_torso = torso_joint_state;
            robot_state_.joint_right_arm = right_arm_joint_state;
            robot_state_.joint_left_arm = left_arm_joint_state;
            robot_state_.joint_head = head_joint_state;
        }
    }

    // template <typename ModelType>
    // void RBY1_ROS2_DRIVER<ModelType>::read_joint_state(){}

    

    // Explicit template instantiations
    template class RBY1_ROS2_DRIVER<rb::y1_model::A>;
    template class RBY1_ROS2_DRIVER<rb::y1_model::M>;
}