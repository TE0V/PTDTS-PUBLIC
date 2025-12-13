"""
Hardware abstraction layer for PTDTS
Provides interfaces and implementations for all hardware components
"""

from .interfaces import (
    EncoderInterface,
    MotorInterface,
    ServoInterface,
    CameraInterface,
    AudioInterface
)

from .encoder import LS7366REncoder, SimulatedEncoder
from .motor import DRV8874Motor, SimulatedMotor
from .servo import AxonServo, AxonServoClosedLoop, SimulatedServo
from .camera import Picamera2Camera, SimulatedCamera, DualCameraManager
from .acoustic import ODASAcousticDetector, SimulatedAcoustic
from .ads1115 import ADS1115Reader
from .factory import HardwareFactory

__all__ = [
    # Interfaces
    'EncoderInterface',
    'MotorInterface',
    'ServoInterface',
    'CameraInterface',
    'AudioInterface',
    # Encoder implementations
    'LS7366REncoder',
    'SimulatedEncoder',
    # Motor implementations
    'DRV8874Motor',
    'SimulatedMotor',
    # Servo implementations
    'AxonServo',
    'AxonServoClosedLoop',
    'SimulatedServo',
    # Camera implementations
    'Picamera2Camera',
    'SimulatedCamera',
    'DualCameraManager',
    # Acoustic implementations
    'ODASAcousticDetector',
    'SimulatedAcoustic',
    # ADC
    'ADS1115Reader',
    # Factory
    'HardwareFactory',
]
