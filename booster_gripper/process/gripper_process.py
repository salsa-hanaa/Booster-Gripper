# gripper_process.py
import math
from rs05_motor import RS05Motor
from booster_joint_interface.msg import SetJoints

MAX_LIMIT_RIGHT = 0
MIN_LIMIT_RIGHT = -1.5
MAX_LIMIT_LEFT = 1.5
MIN_LIMIT_LEFT = 0
LEFT = 22
RIGHT = 23

MAX_STEP_RAD = 0.05  # max change in commanded position (rad) per tick


class GripperProcess:
    def __init__(self, port: str, rate_seconds: float, motor_ids: list[int]):
        self.port = port
        self.rate_seconds = rate_seconds
        self.motor_ids = motor_ids
        self.motor_state: dict[int, float] = {id: 0.0 for id in motor_ids}
        self.motors = dict[int, RS05Motor]({mid: RS05Motor(port) for mid in motor_ids})

        self._pending_targets: dict[int, tuple[float, float, float]] = {}
        self._motion = None          # active motion dict, or None if idle
        self._interpolating = False  # the one flag everything checks

        self.initialize_motors()

    def initialize_motors(self):
        for id, motor in self.motors.items():
            motor.enable(id)
            pos = motor.read_joints(id)
            self.motor_state[id] = pos

    def is_interpolating(self) -> bool:
        return self._interpolating

    # -- commands -----------------------------------------------------

    def move_to_position(self, set_joints_msg: SetJoints, kp: float = 50.0, kd: float = 1.0):
        if self._interpolating:
            print("Interpolation in progress, ignoring move command")
            return

        for joint in set_joints_msg.joints:
            motor = self.motors.get(joint.id)
            if motor is None:
                continue

            target_pos = joint.position
            if joint.id == LEFT:
                target_pos = max(MIN_LIMIT_LEFT, min(MAX_LIMIT_LEFT, joint.position))
            elif joint.id == RIGHT:
                target_pos = max(MIN_LIMIT_RIGHT, min(MAX_LIMIT_RIGHT, joint.position))

            print(f"Queueing Motor ID: {joint.id} -> target: {target_pos}")
            self._pending_targets[joint.id] = (target_pos, kp, kd)

        if self._pending_targets:
            self._interpolating = True
            self._start_next_motion()

    def set_torque(self, id, torque_enable: bool):
        if self._interpolating:
            print("Interpolation in progress, ignoring torque command")
            return

        motor = self.motors.get(id)
        if motor is None:
            return

        if torque_enable:
            motor.move_gripper(id, angle_rad=self.motor_state[id], kp=50.0, kd=1.0)
        else:
            motor.move_gripper(id, angle_rad=self.motor_state[id], kp=0.0, kd=0.0)
        print(f"{'Enabling' if torque_enable else 'Disabling'} torque for Motor ID: {id}")

    def get_motor_state(self):
        if self._interpolating:
            return self.motor_state
        for id in self.motor_ids:
            self.motor_state[id] = self.motors[id].check_joints(id)
        return self.motor_state

    def deg_to_rad(self, degrees: float) -> float:
        return degrees * (3.141592653589793 / 180.0)

    def rad_to_deg(self, radians: float) -> float:
        return radians * (180.0 / 3.141592653589793)

    def stop_motor(self, motor_id: int):
        if self._interpolating:
            print("Interpolation in progress, ignoring stop command")
            return
        self.motors[motor_id].stop(motor_id)

    def close(self):
        for motor in self.motors.values():
            motor.close()

    # -- interpolation --------------------------------------------------

    def _priority_order(self):
        order = [mid for mid in (LEFT, RIGHT) if mid in self.motor_ids]
        order += [mid for mid in self.motor_ids if mid not in order]
        return order

    def _start_next_motion(self):
        next_id = next((mid for mid in self._priority_order() if mid in self._pending_targets), None)

        if next_id is None:
            self._motion = None
            self._interpolating = False
            return

        target_pos, kp, kd = self._pending_targets.pop(next_id)

        # One real read to establish the start point. After this, the
        # motion is tracked purely in software -- no more reads.
        start_pos = self.motors[next_id].read_joints(next_id)
        self.motor_state[next_id] = start_pos

        distance = target_pos - start_pos
        ticks = max(1, math.ceil(abs(distance) / MAX_STEP_RAD))

        self._motion = {
            "motor_id": next_id,
            "current_pos": start_pos,
            "delta": distance / ticks,
            "ticks_remaining": ticks,
            "target_pos": target_pos,
            "kp": kp,
            "kd": kd,
        }

    def step_interpolation(self):
        """Advance the active motion by one tick."""
        if self._motion is None:
            self._interpolating = False
            return

        m = self._motion
        m["ticks_remaining"] -= 1
        next_pos = m["target_pos"] if m["ticks_remaining"] <= 0 else m["current_pos"] + m["delta"]

        self.motors[m["motor_id"]].move_gripper(m["motor_id"], angle_rad=next_pos, kp=m["kp"], kd=m["kd"])
        m["current_pos"] = next_pos
        self.motor_state[m["motor_id"]] = next_pos

        if m["ticks_remaining"] <= 0:
            self._start_next_motion()  # LEFT done -> RIGHT starts, or fully done