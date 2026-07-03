import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from rs05_motor import RS05Motor

if __name__ == "__main__":
    PORT = "/dev/ttyUSB0"
    MOTOR_ID = 23

    print("Menginisialisasi Motor...")
    motor = RS05Motor(PORT)
    
    print("Mengaktifkan Motor (Enable)...")
    motor.enable(MOTOR_ID)
    time.sleep(0.2)

    print("Menggerakkan Gripper ke Posisi 10.0 Radian...")
    motor.move_gripper(MOTOR_ID, angle_rad=10.0, kp=50.0, kd=1.0)
    time.sleep(1.5)

    # print("Mematikan Motor (Stop)...")
    # motor.stop(MOTOR_ID)
    
    motor.close()
    print("Demo Selesai.")