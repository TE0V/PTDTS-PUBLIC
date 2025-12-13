"""
PID Controller implementation for PTDTS
General-purpose PID with anti-windup and derivative filtering
"""

import time
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class PIDController:
    """
    PID (Proportional-Integral-Derivative) Controller
    Implements anti-windup, derivative filtering, and output limiting
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        setpoint: float = 0.0,
        output_limits: Optional[tuple] = None,
        integral_limits: Optional[tuple] = None,
        deadzone: float = 0.0,
        derivative_filter_alpha: float = 0.5,
        error_function: Optional[callable] = None
    ):
        """
        Initialize PID controller

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            setpoint: Initial setpoint
            output_limits: (min, max) output limits
            integral_limits: (min, max) integral term limits (anti-windup)
            deadzone: Error deadzone (no output if |error| < deadzone)
            derivative_filter_alpha: Low-pass filter coefficient for derivative (0-1)
            error_function: Optional custom error calculation function(setpoint, measurement) -> error
                           If None, uses default: setpoint - measurement
                           Useful for angle wraparound: error_function=lambda s, m: wrap_angle(s - m)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.integral_limits = integral_limits
        self.deadzone = deadzone
        self.derivative_filter_alpha = derivative_filter_alpha
        self.error_function = error_function if error_function is not None else lambda s, m: s - m

        # State
        self.integral = 0.0
        self.last_error = 0.0
        self.last_derivative = 0.0
        self.last_time: Optional[float] = None
        self.last_output = 0.0

    def compute(self, measurement: float, dt: Optional[float] = None) -> float:
        """
        Compute PID output

        Args:
            measurement: Current process variable
            dt: Time step (if None, auto-calculated)

        Returns:
            Control output
        """
        # Calculate time step
        current_time = time.time()
        if dt is None:
            if self.last_time is not None:
                dt = current_time - self.last_time
            else:
                dt = 0.0
        self.last_time = current_time

        # Calculate error (using custom function if provided, e.g., for angle wraparound)
        error = self.error_function(self.setpoint, measurement)

        # Apply deadzone
        if abs(error) < self.deadzone:
            error = 0.0
            self.integral = 0.0  # Reset integral in deadzone

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        if dt > 0 and abs(error) > 0:
            # Only integrate if not saturated
            self.integral += error * dt

            # Clamp integral
            if self.integral_limits is not None:
                self.integral = max(
                    self.integral_limits[0],
                    min(self.integral_limits[1], self.integral)
                )

        i_term = self.ki * self.integral

        # Derivative term with filtering
        if dt > 0:
            # Calculate instantaneous derivative
            error_rate = (error - self.last_error) / dt

            # Low-pass filter
            filtered_derivative = (
                self.derivative_filter_alpha * self.last_derivative +
                (1 - self.derivative_filter_alpha) * error_rate
            )
            self.last_derivative = filtered_derivative

            d_term = self.kd * filtered_derivative
        else:
            d_term = 0.0

        # Total output
        output = p_term + i_term + d_term

        # Apply output limits
        if self.output_limits is not None:
            output = max(
                self.output_limits[0],
                min(self.output_limits[1], output)
            )

        # Update state
        self.last_error = error
        self.last_output = output

        return output

    def set_setpoint(self, setpoint: float):
        """
        Set new setpoint

        Args:
            setpoint: Desired setpoint
        """
        self.setpoint = setpoint

    def get_setpoint(self) -> float:
        """
        Get current setpoint

        Returns:
            Current setpoint
        """
        return self.setpoint

    def reset(self):
        """Reset PID state (integral, derivative, errors)"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_derivative = 0.0
        self.last_time = None
        self.last_output = 0.0

    def set_gains(self, kp: Optional[float] = None, ki: Optional[float] = None, kd: Optional[float] = None):
        """
        Update PID gains

        Args:
            kp: Proportional gain (None = no change)
            ki: Integral gain (None = no change)
            kd: Derivative gain (None = no change)
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

    def get_gains(self) -> tuple:
        """
        Get current PID gains

        Returns:
            (kp, ki, kd) tuple
        """
        return (self.kp, self.ki, self.kd)

    def get_components(self) -> dict:
        """
        Get PID component values (for debugging/tuning)

        Returns:
            Dictionary with p_term, i_term, d_term, output
        """
        error = self.setpoint - self.last_error  # Approximate current error
        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * self.last_derivative

        return {
            'p_term': p_term,
            'i_term': i_term,
            'd_term': d_term,
            'error': error,
            'integral': self.integral,
            'derivative': self.last_derivative,
            'output': self.last_output
        }
