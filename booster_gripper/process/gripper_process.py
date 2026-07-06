import time

from rs05_motor import RS05Motor
from booster_joint_interface.msg import SetJoints

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
            if motor is not None:
                motor.move_gripper(joint.id, angle_rad=self.deg_to_rad(joint.position), kp=kp, kd=kd)
        time.sleep(self.rate_seconds)

    def set_torque(self, ids: list[int], torque_enable: bool):
        for id in ids:
            motor = self.motors.get(id)
            if motor is not None:
                if torque_enable:
                    motor.enable(id)
                else:
                    motor.stop(id)

    def get_motor_state(self):
        for id in self.motor_ids:
            self.motor_state[id] = self.motors[id].check_position(id)
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

