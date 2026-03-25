# rby1_driver

## 개요
- rby1 로봇을 ros2에서 제어하기 위한 드라이버 패키지 구현
- 초기 세팅(모델, 속도,가속도제한, ip 등)을 한번에 관리할 수 있도록 yaml파일에 파라미터 세팅
- 상세 옵션 트리거(major,minor fault 뜨면 자동으로 리셋한번 해주는 기능 등) 구현
- 각 컴포넌트(모바일, 팔, torso, 머리)의 state(position,torque,velocity)를 100hz로 publish
- 대부분의 기능들을 topic 통신을 통해 제어할 수있도록 구현

|parameter|default value|description|
|---|---|---|
|robot_ip|127.0.0.1:50051|robot ip address|
|model|a|robot model|
|joint_topic_name|joint_states|joint state topic name|
|power_on|{5,12,24,48}|power on list|
|servo_on|{"all"}|servo on list|
|minimum_time|2.0|minimum time|
|angular_velocity_limit|4.712388|angular velocity limit|
|linear_velocity_limit|1.5|linear velocity limit|
|acceleration_limit|1.0|acceleration limit|
|stop_orientation_tracking_error|1e-5|stop orientation tracking error|
|stop_position_tracking_error|1e-5|stop position tracking error|
|fault_reset_trigger|false|fault reset trigger|
|node_power_off_trigger|false|node power off trigger|