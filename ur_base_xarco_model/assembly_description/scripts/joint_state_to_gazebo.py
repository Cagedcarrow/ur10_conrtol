#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelConfiguration


class JointStateToGazebo(Node):
    def __init__(self):
        super().__init__('joint_state_to_gazebo')
        self.declare_parameter('model_name', 'assembly_robot_visual')
        self.declare_parameter('update_hz', 20.0)

        self.model_name = str(self.get_parameter('model_name').value)
        self.update_dt = 1.0 / max(float(self.get_parameter('update_hz').value), 1.0)

        self._joint_map = {}
        self._last_sent = self.get_clock().now()

        self._target_joints = [
            'ur10_shoulder_pan',
            'ur10_shoulder_lift',
            'ur10_elbow',
            'ur10_wrist_1',
            'ur10_wrist_2',
            'ur10_wrist_3',
        ]

        self._cli = self.create_client(SetModelConfiguration, '/gazebo/set_model_configuration')
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.create_timer(self.update_dt, self._tick)

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_map[name] = float(pos)

    def _tick(self):
        if not self._cli.service_is_ready():
            return

        if not all(name in self._joint_map for name in self._target_joints):
            return

        req = SetModelConfiguration.Request()
        req.model_name = self.model_name
        req.urdf_param_name = 'robot_description'
        req.joint_names = list(self._target_joints)
        req.joint_positions = [self._joint_map[n] for n in self._target_joints]

        self._cli.call_async(req)


def main():
    rclpy.init()
    node = JointStateToGazebo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
