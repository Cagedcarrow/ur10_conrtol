#!/usr/bin/env python3
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node


class WaitControllerManager(Node):
    def __init__(self):
        super().__init__('wait_controller_manager')
        self.declare_parameter('service_name', '/controller_manager/list_controllers')
        self.declare_parameter('timeout_sec', 30.0)
        self.declare_parameter('poll_interval_sec', 0.5)


def main():
    rclpy.init()
    node = WaitControllerManager()
    service_name = str(node.get_parameter('service_name').value)
    timeout_sec = float(node.get_parameter('timeout_sec').value)
    poll_interval = float(node.get_parameter('poll_interval_sec').value)

    client = node.create_client(ListControllers, service_name)
    deadline = time.time() + timeout_sec

    while rclpy.ok() and time.time() < deadline:
        if client.wait_for_service(timeout_sec=poll_interval):
            req = ListControllers.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=poll_interval)
            if future.done() and future.exception() is None:
                node.get_logger().info(f'Controller manager service callable: {service_name}')
                node.destroy_node()
                rclpy.shutdown()
                return 0
        time.sleep(poll_interval)

    node.get_logger().error(f'Timeout waiting for callable service: {service_name}')
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
