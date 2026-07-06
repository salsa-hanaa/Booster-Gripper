import time

from rs05_motor import RS05Motor

class GripperProcess:
    def __init__(self, port: str, motor_id: int):
        self.motor = RS05Motor(port)
        self.motor_id = motor_id
        self.motor.enable(self.motor_id)
        time.sleep(0.2)

    def move_to_position(self, angle_rad: float, kp: float = 50.0, kd: float = 1.0):
        """Gerakkan gripper ke posisi tertentu dalam radian."""
        self.motor.move_gripper(self.motor_id, angle_rad=angle_rad, kp=kp, kd=kd)
        time.sleep(1.5)  # Tunggu sampai gerakan selesai

    def check_position(self):
        """Baca posisi aktual gripper."""
        return self.motor.check_joints(self.motor_id)

    def stop_motor(self):
        """Hentikan motor gripper."""
        self.motor.stop(self.motor_id)

    def close(self):
        """Tutup koneksi dengan motor."""
        self.motor.close()