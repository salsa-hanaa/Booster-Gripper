from rclpy.node import Node
from booster_joint_interface.msg import SetJoints, SetTorques, Joint
from booster_gripper.process.gripper_process import GripperProcess

class GripperNode:
    def __init__(self, node: Node, port: str, rate_seconds: float, motor_ids: list[int]):
        self.node = node
        self.gripper_state_publisher = self.node.create_publisher(SetJoints, '/joint/gripper_state', 10)
        self.timer = self.node.create_timer(rate_seconds, self.publish_gripper_state)
        self.gripper_process = GripperProcess(port, rate_seconds, motor_ids)
        self.set_grippers_subscription = self.node.create_subscription(
            SetJoints,
            '/joint/set_grippers',
            self.set_grippers_callback,
            10
        )

    def set_grippers_callback(self, msg: SetJoints):
        self.gripper_process.move_to_position(msg)

    def set_torque_callback(self, msg: SetTorques):
        msg = SetTorques()
        for id in msg.torques:
            self.gripper_process.set_torque(id, msg.torques)

    def publish_gripper_state(self):
        motor_state = self.gripper_process.get_motor_state()
        msg = SetJoints()
        for motor_id, angle in motor_state.items():
            joint = Joint()
            joint.id = motor_id
            joint.position = motor_state[motor_id]
            msg.joints.append(joint)
            
        self.gripper_state_publisher.publish(msg)