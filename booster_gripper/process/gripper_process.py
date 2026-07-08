import time

from rs05_motor import RS05Motor
from booster_joint_interface.msg import SetJoints

MAX_LIMIT_RIGHT = 0
MIN_LIMIT_RIGHT = -1.5
MAX_LIMIT_LEFT = 1.5
MIN_LIMIT_LEFT = 0

LEFT = 22
RIGHT = 23

class GripperProcess:
    def __init__(self, port: str, rate_seconds: float, motor_ids: list[int]):
        self.port = port
        self.rate_seconds = rate_seconds
        self.motor_ids = motor_ids
        self.motor_state: dict[int, float] = {id: 0.0 for id in motor_ids}
        self.motors = dict[int, RS05Motor] ({mid: RS05Motor(port) for mid in motor_ids})
        self.initialize_motors()

    def initialize_motors(self):
        for id, motor in self.motors.items():
            motor.enable(id)

    def move_to_position(self, set_joints_msg: SetJoints, kp: float = 50.0, kd: float = 1.0):
        for joint in set_joints_msg.joints:
            motor = self.motors.get(joint.id)
            print(f"Moving Motor ID: {joint.id} to Position: {joint.position} degrees")
            if motor is not None:
                target_pos = joint.position
                if (joint.id == LEFT):
                    target_pos = max(MIN_LIMIT_LEFT, min(MAX_LIMIT_LEFT, joint.position))
                elif (joint.id == RIGHT):
                    target_pos = max(MIN_LIMIT_RIGHT, min(MAX_LIMIT_RIGHT, joint.position))
                    print("target: " + str(target_pos))
                motor.move_gripper(joint.id, angle_rad=target_pos, kp=kp, kd=kd)

    def set_torque(self, id, torque_enable: bool):
        motor = self.motors.get(id)
        if motor is not None:
            if torque_enable:
                motor.move_gripper(id, angle_rad=self.motor_state[id], kp=50.0, kd=1.0)
            else:
                motor.move_gripper(id, angle_rad=self.motor_state[id], kp=0.0, kd=0.0)
                print(f"Disabling torque for Motor ID: {id}")

    def get_motor_state(self):
        for id in self.motor_ids:
            self.motor_state[id] = self.motors[id].check_joints(id)
        return self.motor_state
    
    def deg_to_rad(self, degrees: float) -> float:
        return degrees * (3.141592653589793 / 180.0)
    
    def rad_to_deg(self, radians: float) -> float:
        return radians * (180.0 / 3.141592653589793)
    
    def stop_motor(self, motor_id: int):
        self.motors[motor_id].stop(motor_id)

    def close(self):
        for motor in self.motors.values():
            motor.close()

