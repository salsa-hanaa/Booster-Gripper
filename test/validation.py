import sys
import os
import pytest

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from rs05_motor.bit_convert import build_at_frame, COMM_TYPE_ENABLE, COMM_TYPE_MOTION, DEFAULT_HOST_ID
from rs05_motor.conversion import float_to_uint16, TORQUE_RANGE

def test_enable_frame_match():
    """Validasi apakah fungsi menghasilkan string biner yang sama persis dengan log nyata."""
    payload = bytes(8)
    frame = build_at_frame(COMM_TYPE_ENABLE, DEFAULT_HOST_ID, motor_id=22, payload=payload)
    
    expected_hex = "41541807e8b40800000000000000000d0a"
    assert frame.hex() == expected_hex

def test_motion_frame_match():
    """Validasi frame paket gerak (Angle: 3.0, KP: 8.0, KD: 1.0, Torque: 0.0)"""
    torque_raw = float_to_uint16(0.0, *TORQUE_RANGE)
    assert torque_raw == 0x7FFF
    
    payload = bytes.fromhex("9e8b7fff04183333")
    frame = build_at_frame(COMM_TYPE_MOTION, torque_raw, motor_id=22, payload=payload)
    
    expected_hex = "41540bfff8b4089e8b7fff041833330d0a"
    assert frame.hex() == expected_hex