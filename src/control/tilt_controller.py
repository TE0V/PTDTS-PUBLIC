"""
Tilt controller for PTDTS
Simple position control for servo
"""

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class TiltController:
    """
    Tilt axis controller
    Position-based control for servo
    """

    def __init__(
        self,
        servo,
        config
    ):
        """
        Initialize tilt controller

        Args:
            servo: Servo interface
            config: Configuration object
        """
        self.servo = servo
        self.config = config

        # Set to default position
        self.servo.set_angle(config.tilt_servo.default_angle)

        logger.info(f"Tilt controller initialized: default={config.tilt_servo.default_angle}°")

    def set_angle(self, angle: float):
        """
        Set tilt angle

        Args:
            angle: Target angle in degrees (0-180, 90=horizontal)
        """
        # Clamp to limits
        angle = max(
            self.config.tilt_servo.min_angle,
            min(self.config.tilt_servo.max_angle, angle)
        )

        self.servo.set_angle(angle)
        logger.debug(f"Tilt angle set: {angle:.1f}°")

    def set_target_angle(self, angle: float):
        """
        Set target angle (alias for set_angle for API consistency)

        Args:
            angle: Target angle in degrees (0-180, 90=horizontal)
        """
        self.set_angle(angle)

    def get_angle(self) -> float:
        """
        Get current tilt angle

        Returns:
            Current angle in degrees
        """
        return self.servo.get_angle()

    def set_horizontal(self):
        """Set to horizontal position (90°)"""
        self.set_angle(90.0)

    def update(self):
        """
        Update tilt controller
        Called periodically from main loop

        Note: Tilt servo doesn't need periodic updates like pan motor,
        but this method is provided for consistency with the main loop.
        """
        # Tilt servo control is event-driven via set_angle()
        # No periodic updates needed for simple position control
        pass

    def get_status(self) -> dict:
        """
        Get current controller status for telemetry

        Returns:
            Dictionary with current status information
        """
        current_angle = self.servo.get_angle()

        return {
            'current_angle': current_angle,
            'target_angle': current_angle,  # Servo reaches target immediately
            'pwm': getattr(self.servo, 'current_pwm_us', 1500)  # Default center value
        }

    def save_state(self, state_manager) -> None:
        """
        Save controller state to StateManager.

        Args:
            state_manager: StateManager instance to save state to
        """
        current_angle = self.servo.get_angle()

        # Get PID state if servo has closed-loop control
        pid_integral = 0.0
        pid_last_error = 0.0
        if hasattr(self.servo, 'pid'):
            pid_integral = self.servo.pid.integral
            pid_last_error = self.servo.pid.last_error

        state_manager.set_tilt_controller_state(
            current_angle=current_angle,
            target_angle=current_angle,  # Servo reaches target immediately
            pid_integral=pid_integral,
            pid_last_error=pid_last_error
        )

        logger.info(f"Tilt controller state saved: angle={current_angle:.2f}°")

    def load_state(self, state_manager) -> bool:
        """
        Load controller state from StateManager.

        Args:
            state_manager: StateManager instance to load state from

        Returns:
            True if state was loaded successfully, False otherwise
        """
        tilt_state = state_manager.get_tilt_controller_state()
        if tilt_state:
            target_angle = tilt_state.get('target_angle', self.config.tilt_servo.default_angle)

            # Restore servo position
            self.set_angle(target_angle)

            # Restore PID state if servo has closed-loop control
            if hasattr(self.servo, 'pid'):
                self.servo.pid.integral = tilt_state.get('pid_integral', 0.0)
                self.servo.pid.last_error = tilt_state.get('pid_last_error', 0.0)

            logger.info(f"Tilt controller state restored: angle={target_angle:.2f}°")
            return True
        else:
            logger.info("No saved tilt controller state found")
            return False

    def close(self):
        """Clean up resources"""
        # Return to default position
        self.servo.set_angle(self.config.tilt_servo.default_angle)
        logger.info("Tilt controller closed")
