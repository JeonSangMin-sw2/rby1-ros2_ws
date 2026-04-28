# rby1_driver

## 개요
- RBY1 로봇을 ROS 2에서 제어하기 위한 드라이버 패키지.
- 초기 세팅(모델, IP, 속도/가속도 제한, 제어 방식 등)을 `yaml` 파라미터 파일에서 한 번에 관리.
- 상세 옵션 트리거(초기화 단계에서 Major/Minor fault 발생 시 자동 리셋 등)를 지원.
- 지정된 주기(`get_state_period`, 기본 100Hz)로 각 컴포넌트(모바일, 팔, Torso, 머리)의 상태(`JointState`)를 Publish.
- Service 및 Action 통신을 통해 로봇 제어(전원, 서보, 개별/다중 관절 구동)를 완벽히 추상화하여 제공.

## Parameters (`config/driver_parameters.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `robot_ip` | `"127.0.0.1:50051"` | 로봇 통신 IP 주소 및 포트 |
| `model` | `"a"` | 로봇 모델 (`"a"` 또는 `"m"`) |
| `power_on` | `["all"]` | 구동 시 전원을 켤 부위 목록 (예: `["all"]`, `["5"]`) |
| `servo_on` | `["all"]` | 구동 시 서보를 켤 부위 목록 (예: `["all"]`, `["torso"]`) |
| `get_state_period` | `0.01` | 로봇 상태(`JointState`) Publish 주기 (초) |
| `minimum_time` | `2.0` | 명령 수행 시 기본 최소 실행 시간 (초) |
| `angular_velocity_limit` | `4.712388` | 관절 위치 제어 시 Cutoff frequency 한계값 |
| `linear_velocity_limit` | `1.5` | 카테시안 제어 시 Cutoff frequency 한계값 |
| `acceleration_limit` | `1.0` | 가속도 한계 스케일링 |
| `stop_orientation_tracking_error` | `1e-5` | 방향 트래킹 에러 제한값 |
| `stop_position_tracking_error` | `1e-5` | 위치 트래킹 에러 제한값 |
| `state_topic_name` | `"joint_states"` | 상태 Publisher 및 Action Server의 기본 토픽 이름 |
| `control_mode.torso` | `"joint"` | Torso 제어기 빌더 모드 (`"joint"` 또는 `"impedance"`) |
| `control_mode.right_arm` | `"joint"` | 우측 팔 제어기 빌더 모드 (`"joint"` 또는 `"impedance"`) |
| `control_mode.left_arm` | `"joint"` | 좌측 팔 제어기 빌더 모드 (`"joint"` 또는 `"impedance"`) |
| `control_mode.head` | `"joint"` | 머리 제어기 빌더 모드 (`"joint"`) |
| `fault_reset_trigger` | `false` | 구동 초기 폴트 발생 시 자동 리셋 여부 |
| `node_power_off_trigger` | `false` | 노드 종료 시 전원 자동 Off 여부 |

---

## Communication Interfaces

### Publishers
지정된 `get_state_period` (기본 100Hz) 주기로 각 부위의 Joint State를 발행합니다.
- `/<state_topic_name>/torso` (`sensor_msgs/msg/JointState`)
- `/<state_topic_name>/right_arm` (`sensor_msgs/msg/JointState`)
- `/<state_topic_name>/left_arm` (`sensor_msgs/msg/JointState`)
- `/<state_topic_name>/head` (`sensor_msgs/msg/JointState`)

### Services
로봇의 전원 및 서보 상태를 통합 제어하는 서비스를 제공.
- `/robot_power` (`rby1_msgs/srv/StateOnOff`)
  - **Request**: `bool state` (true=On, false=Off), `string parameters` (적용할 부위, 예: `"5,12,24,48"`, `"all"`)
- `/robot_servo` (`rby1_msgs/srv/StateOnOff`)
  - **Request**: `bool state` (true=On, false=Off), `string parameters` (적용할 부위, 예: `"right_arm, head"`, `"all"`)

### Action Servers
YAML 설정된 `control_mode` (Joint/Impedance)에 맞추어 동적으로 명령을 빌드하고 주입하는 액션 서버.
- `/<state_topic_name>/single_position_command` (`rby1_msgs/action/SingleJointCommand`)
  - **설명**: 단일 특정 부위에 제어 명령(Goal)을 전달.
  - **Goal**:
    - `string target_name`: 대상 부위 (`"torso"`, `"right_arm"`, `"left_arm"`, `"head"`)
    - `float64[] position`
    - `float64 minimum_time`(0.01보다 크면 기본 파라미터 사용)

- `/<state_topic_name>/multi_position_command` (`rby1_msgs/action/MultiJointCommand`)
  - **설명**: 여러 부위에 동시에 동기화된 제어 명령(Goal)을 전달.
  - **Goal**:
    - `float64[] torso`
    - `float64[] right_arm`
    - `float64[] left_arm`
    - `float64[] head`
    - `float64 minimum_time`(0.01보다 크면 기본 파라미터 사용)
