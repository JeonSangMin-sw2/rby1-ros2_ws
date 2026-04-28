#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import SingleJointCommand
from rby1_msgs.srv import StateOnOff
from std_msgs.msg import Int32

class SingleJointExample(Node):
    def __init__(self):
        super().__init__('single_joint_example')
        self._action_client = ActionClient(self, SingleJointCommand, 'joint_states/single_position_command')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.state_sub = self.create_subscription(Int32, 'joint_states/control_state', self.state_callback, 10)
        self.control_state = None

    def state_callback(self, msg):
        self.control_state = msg.data

    def wait_for_state(self):
        while self.control_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.control_state

    def send_power_request(self, state, parameters):
        req = StateOnOff.Request()
        req.state = state
        req.parameters = parameters
        future = self.power_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_servo_request(self, state, parameters):
        req = StateOnOff.Request()
        req.state = state
        req.parameters = parameters
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_goal(self, target_name, position, minimum_time):
        goal_msg = SingleJointCommand.Goal()
        goal_msg.target_name = target_name
        goal_msg.position = position
        goal_msg.minimum_time = minimum_time

        self._action_client.wait_for_server()
        self.get_logger().info('Sending Single Joint Command Goal...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = SingleJointExample()

    state = action_client.wait_for_state()
    if state not in [2, 3]:
        action_client.get_logger().info('Robot is not enabled (or in fault). Sending Power and Servo ON (all)...')
        action_client.send_power_request(True, 'all')
        action_client.send_servo_request(True, 'all')
        while action_client.wait_for_state() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(action_client, timeout_sec=0.1)
        action_client.get_logger().info('Robot enabled.')

    # Move right arm's 7 joints
    target = "right_arm"
    position = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]
    min_time = 3.0
    
    future = action_client.send_goal(target, position, min_time)
    rclpy.spin_until_future_complete(action_client, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        action_client.get_logger().info('Goal rejected :(')
        return

    action_client.get_logger().info('Goal accepted :)')
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(action_client, get_result_future)

    result = get_result_future.result().result
    action_client.get_logger().info(f'Result: {result.success}, Code: {result.finish_code}')

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
