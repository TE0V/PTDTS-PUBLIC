"""
Hardware interfaces for PTDTS
Abstract base classes defining contracts for hardware components
"""

from abc import ABC, abstractmethod
from typing import Optional, Tuple
import numpy as np


class EncoderInterface(ABC):
    """
    Abstract interface for rotary encoders
    Provides position and velocity feedback
    """

    @abstractmethod
    def get_count(self) -> int:
        """
        Get current encoder count

        Returns:
            Raw encoder count (can be negative)
        """
        pass

    @abstractmethod
    def get_angle(self) -> float:
        """
        Get current angle in degrees (0-360)

        Returns:
            Angle in degrees
        """
        pass

    @abstractmethod
    def get_velocity(self) -> float:
        """
        Get current angular velocity in degrees per second

        Returns:
            Angular velocity in deg/sec
        """
        pass

    @abstractmethod
    def reset(self):
        """Reset encoder count to zero"""
        pass

    @abstractmethod
    def close(self):
        """Clean up resources"""
        pass


class MotorInterface(ABC):
    """
    Abstract interface for DC motors
    Provides speed and direction control
    """

    @abstractmethod
    def set_pwm(self, pwm: float):
        """
        Set motor PWM duty cycle

        Args:
            pwm: PWM duty cycle (-1.0 to 1.0)
                 Positive = CW, Negative = CCW
        """
        pass

    @abstractmethod
    def stop(self):
        """Stop motor immediately"""
        pass

    @abstractmethod
    def brake(self):
        """
        Brake motor (active braking, both inputs high)
        More aggressive than stop()
        """
        pass

    @abstractmethod
    def get_current_pwm(self) -> float:
        """
        Get current PWM setting

        Returns:
            Current PWM duty cycle
        """
        pass

    @abstractmethod
    def close(self):
        """Clean up resources"""
        pass


class ServoInterface(ABC):
    """
    Abstract interface for servo motors
    Provides position control
    """

    @abstractmethod
    def set_angle(self, angle: float):
        """
        Set servo angle

        Args:
            angle: Target angle in degrees (0-180)
        """
        pass

    @abstractmethod
    def get_angle(self) -> float:
        """
        Get current servo angle

        Returns:
            Current angle in degrees
        """
        pass

    @abstractmethod
    def close(self):
        """Clean up resources"""
        pass


class CameraInterface(ABC):
    """
    Abstract interface for cameras
    Provides frame capture
    """

    @abstractmethod
    def start(self):
        """Start camera"""
        pass

    @abstractmethod
    def stop(self):
        """Stop camera"""
        pass

    @abstractmethod
    def capture_array(self) -> np.ndarray:
        """
        Capture a frame as numpy array

        Returns:
            Frame as numpy array (H, W, 3) in RGB format
        """
        pass

    @abstractmethod
    def get_resolution(self) -> Tuple[int, int]:
        """
        Get camera resolution

        Returns:
            (width, height) tuple
        """
        pass

    @abstractmethod
    def get_framerate(self) -> int:
        """
        Get camera framerate

        Returns:
            Framerate in FPS
        """
        pass

    @abstractmethod
    def close(self):
        """Clean up resources"""
        pass


class AudioInterface(ABC):
    """
    Abstract interface for acoustic detection
    Provides direction of arrival (DOA) from microphone array
    """

    @abstractmethod
    def start(self):
        """Start acoustic detection"""
        pass

    @abstractmethod
    def stop(self):
        """Stop acoustic detection"""
        pass

    @abstractmethod
    def get_detections(self) -> list:
        """
        Get recent acoustic detections

        Returns:
            List of detection dictionaries with keys:
            - azimuth: Direction in degrees (0-360)
            - elevation: Elevation in degrees (-90 to 90)
            - energy: Signal energy (0-1)
            - timestamp: Detection timestamp
        """
        pass

    @abstractmethod
    def is_running(self) -> bool:
        """
        Check if acoustic detection is running

        Returns:
            True if running
        """
        pass

    @abstractmethod
    def close(self):
        """Clean up resources"""
        pass
