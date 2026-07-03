import logging
from .communicate import SerialTransport
from .conversion import float_to_uint16, POS_RANGE, VEL_RANGE, KP_RANGE, KD_RANGE, TORQUE_RANGE
from .bit_convert import build_at_frame, COMM_TYPE_ENABLE, COMM_TYPE_STOP, COMM_TYPE_MOTION, DEFAULT_HOST_ID

# Set up logging agar formatnya rapi dan gampang dibaca
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class RS05Motor:
    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 0.2):
        logging.info(f"Menghubungkan ke port {port} dengan baudrate {baudrate}...")
        self.transport = SerialTransport(port, baudrate, timeout)
        logging.info("Inisialisasi handshake port selesai.")

    def _execute_and_log(self, action_name: str, frame: bytes) -> bytes:
        """Helper untuk mengirim frame, mencetak TxHex, dan menangkap RxHex dari motor."""
        # Cetak log TxHex dalam bentuk string lowercase hex (mirip hasil logmu)
        tx_hex = frame.hex()
        print(f"\n[{action_name}]")
        print(f"  TxHex: \"{tx_hex}\"")
        
        # Kirim data ke hardware
        rx_bytes = self.transport.send_frame(frame, read_reply=True)
        
        # Cetak log RxHex jika motor memberikan balasan
        if rx_bytes:
            # Mengambil data hex bersih dari respons (menghilangkan noise jika ada)
            rx_hex = rx_bytes.hex()
            print(f"  RxHex: \"{rx_hex}\"")
        else:
            print("  RxHex: \"No Response/Timeout\"")
            
        return rx_bytes

    def enable(self, motor_id: int):
        """Nyalakan/aktifkan motor."""
        payload = bytes(8)
        frame = build_at_frame(COMM_TYPE_ENABLE, DEFAULT_HOST_ID, motor_id, payload)
        logging.info(f"Mengirim perintah ENABLE ke Motor ID: {motor_id}")
        return self._execute_and_log("ENABLE_MOTOR", frame)

    def stop(self, motor_id: int):
        """Matikan motor."""
        payload = bytes(8)
        frame = build_at_frame(COMM_TYPE_STOP, DEFAULT_HOST_ID, motor_id, payload)
        logging.info(f"Mengirim perintah STOP ke Motor ID: {motor_id}")
        return self._execute_and_log("STOP_MOTOR", frame)

    def move_gripper(self, motor_id: int, angle_rad: float, kp: float, kd: float,
                     velocity: float = 0.0, torque: float = 0.0):
        """
        Gerakkan motor ke posisi target pakai mode MIT (Motion Control).
        Menampilkan log parameter input dunia nyata agar mudah dicocokkan dengan UI.
        """
        logging.info(
            f"Kirim Perintah Gerak -> ID: {motor_id} | Target Angle: {angle_rad:.2f} rad "
            f"| Kp: {kp:.1f} | Kd: {kd:.1f} | Target Vel: {velocity:.1f} | Torque FF: {torque:.1f}"
        )
        
        pos_raw = float_to_uint16(angle_rad, *POS_RANGE)
        vel_raw = float_to_uint16(velocity, *VEL_RANGE)
        kp_raw  = float_to_uint16(kp, *KP_RANGE)
        kd_raw  = float_to_uint16(kd, *KD_RANGE)
        torque_raw = float_to_uint16(torque, *TORQUE_RANGE)

        payload = (
            pos_raw.to_bytes(2, "big") +
            vel_raw.to_bytes(2, "big") +
            kp_raw.to_bytes(2, "big") +
            kd_raw.to_bytes(2, "big")
        )

        frame = build_at_frame(COMM_TYPE_MOTION, torque_raw, motor_id, payload)
        return self._execute_and_log("MOTION_CONTROL", frame)

    def close(self):
        """Tutup koneksi serial."""
        logging.info("Menutup koneksi serial.")
        self.transport.close()