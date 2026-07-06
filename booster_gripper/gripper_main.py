import sys

import rclpy
from rclpy.node import Node

from booster_gripper.node.gripper_node import GripperNode

MOTOR_IDS = [22, 23]
RATE_SECONDS = 0.08   

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_port>")
        print(f"Example: {sys.argv[0]} /dev/ttyUSB0")
        return 1

    port = sys.argv[1]

    rclpy.init()

    node = Node("gripper")
    gripper_node = GripperNode(node, port, RATE_SECONDS, MOTOR_IDS)

    rclpy.spin(gripper_node.node)

    gripper_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()