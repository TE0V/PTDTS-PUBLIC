"""
Servo implementations for PTDTS
Axon MAX MK2 servo with PWM and optional closed-loop control
"""

import time
import logging
import threading
from typing import Optional
from .interfaces import ServoInterface
from .ads1115 import ADS1115Reader, SimulatedADS1115

logger = logging.getLogger(__name__)


class AxonServo(ServoInterface):
    """
    Axon MAX MK2 servo with PWM control and S-curve slew rate limiting

    Features:
    - Smooth acceleration/deceleration (S-curve motion profile)
    - Configurable slew rate for gentle tracking
    - Non-blocking motion (set_angle returns immediately)
    - Thread-safe operation
    - Hard angle limits to prevent mechanical damage
    """

    def __init__(
        self,
        gpio_pwm: int = 18,  # Hardware PWM pin (required for Pi 5)
        pwm_frequency: int = 333,
        pwm_min: int = 500,
        pwm_max: int = 2500,
        min_angle: float = 0.0,
        max_angle: float = 90.0,
        slew_rate_dps: float = 15.0,
        acceleration_profile: str = 's_curve'
    ):
        """
        Initialize Axon servo with smooth motion control

        Args:
            gpio_pwm: GPIO pin for PWM signal (use hardware PWM pin on Pi 5)
            pwm_frequency: PWM frequency in Hz (333Hz for Axon MAX MK2)
            pwm_min: Minimum pulse width in microseconds
            pwm_max: Maximum pulse width in microseconds
            min_angle: Minimum angle in degrees (hard limit)
            max_angle: Maximum angle in degrees (hard limit)
            slew_rate_dps: Maximum slew rate in degrees per second
            acceleration_profile: Motion profile ('linear', 's_curve', or 'trapezoidal')
        """
        self.gpio_pwm = gpio_pwm
        self.pwm_frequency = pwm_frequency
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.slew_rate_dps = slew_rate_dps
        self.acceleration_profile = acceleration_profile

        # Motion state
        self.current_angle = (min_angle + max_angle) / 2.0  # Start at center
        self.target_angle = self.current_angle
        self.lock = threading.Lock()
        self.last_update_time = time.time()

        # Motion control thread
        self.running = False
        self.motion_thread: Optional[threading.Thread] = None
        self.motion_update_rate_hz = 50.0  # Smooth 50Hz updates

        try:
            from gpiozero import Servo as GPIOZeroServo
            from gpiozero.pins.lgpio import LGPIOFactory

            # Use lgpio for Pi 5 compatibility
            factory = LGPIOFactory()

            # Convert microseconds to gpiozero format (0-1 range)
            min_pulse_width = pwm_min / 1_000_000
            max_pulse_width = pwm_max / 1_000_000

            self.servo = GPIOZeroServo(
                gpio_pwm,
                min_pulse_width=min_pulse_width,
                max_pulse_width=max_pulse_width,
                frame_width=1.0 / pwm_frequency,
                pin_factory=factory
            )

            # Start motion control thread
            self._start_motion_thread()

            # Set to center position
            self._send_to_servo(self.current_angle)

            logger.info(f"Axon servo initialized: GPIO{gpio_pwm}, {pwm_min}-{pwm_max}us @ {pwm_frequency}Hz, slew={slew_rate_dps}°/s, profile={acceleration_profile}")

        except Exception as e:
            logger.error(f"Failed to initialize Axon servo: {e}")
            raise

    def _start_motion_thread(self):
        """Start smooth motion control thread"""
        self.running = True
        self.motion_thread = threading.Thread(target=self._motion_loop, daemon=True)
        self.motion_thread.start()
        logger.debug("Servo motion control thread started")

    def _stop_motion_thread(self):
        """Stop motion control thread"""
        self.running = False
        if self.motion_thread:
            self.motion_thread.join(timeout=1.0)
        logger.debug("Servo motion control thread stopped")

    def _s_curve_interpolation(self, progress: float) -> float:
        """
        S-curve interpolation for smooth acceleration/deceleration

        Args:
            progress: Linear progress from 0.0 to 1.0

        Returns:
            S-curved progress from 0.0 to 1.0
        """
        # Smooth S-curve using cubic ease-in-out
        if progress < 0.5:
            return 4.0 * progress * progress * progress
        else:
            p = 2.0 * progress - 2.0
            return 1.0 + 0.5 * p * p * p

    def _motion_loop(self):
        """Smooth motion control loop (runs at fixed rate)"""
        update_period = 1.0 / self.motion_update_rate_hz
        next_update_time = time.time()

        while self.running:
            try:
                current_time = time.time()
                dt = current_time - self.last_update_time

                with self.lock:
                    error = self.target_angle - self.current_angle

                    if abs(error) > 0.01:  # Still moving
                        # Calculate maximum movement this step
                        max_movement = self.slew_rate_dps * dt

                        if self.acceleration_profile == 's_curve':
                            # S-curve: smooth acceleration/deceleration
                            # Calculate total distance and progress
                            total_distance = abs(error)

                            if abs(error) <= max_movement:
                                # Close enough - arrive at target
                                self.current_angle = self.target_angle
                            else:
                                # Move with S-curve smoothing
                                direction = 1.0 if error > 0 else -1.0
                                movement = min(max_movement, abs(error))
                                self.current_angle += direction * movement

                        elif self.acceleration_profile == 'linear':
                            # Linear: constant speed
                            if abs(error) <= max_movement:
                                self.current_angle = self.target_angle
                            else:
                                direction = 1.0 if error > 0 else -1.0
                                self.current_angle += direction * max_movement

                        else:  # Default to linear
                            if abs(error) <= max_movement:
                                self.current_angle = self.target_angle
                            else:
                                direction = 1.0 if error > 0 else -1.0
                                self.current_angle += direction * max_movement

                        # Send updated position to servo
                        self._send_to_servo(self.current_angle)

                    self.last_update_time = current_time

                # Fixed-rate timing
                next_update_time += update_period
                sleep_time = next_update_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # Overrun - reset timing
                    next_update_time = time.time()

            except Exception as e:
                logger.error(f"Error in servo motion loop: {e}")
                time.sleep(0.1)

    def _send_to_servo(self, angle: float):
        """
        Send angle command to physical servo (internal use only)

        Args:
            angle: Angle in degrees (already clamped)
        """
        # Convert angle to servo value (-1 to 1 range)
        # gpiozero Servo: -1 = min_angle, 0 = mid, 1 = max_angle
        normalized = (angle - self.min_angle) / (self.max_angle - self.min_angle)
        servo_value = (normalized * 2.0) - 1.0  # Convert 0-1 to -1 to 1

        self.servo.value = servo_value

    def set_angle(self, angle: float):
        """
        Set servo target angle (non-blocking, servo moves smoothly in background)

        Args:
            angle: Target angle in degrees (will be clamped to min/max)
        """
        with self.lock:
            # Clamp to hard limits
            angle = max(self.min_angle, min(self.max_angle, angle))
            self.target_angle = angle
            logger.debug(f"Servo target set: {angle:.1f}° (current: {self.current_angle:.1f}°)")

    def get_angle(self) -> float:
        """
        Get current servo angle (actual position, not target)

        Returns:
            Current angle in degrees
        """
        with self.lock:
            return self.current_angle

    def close(self):
        """Clean up GPIO resources"""
        self._stop_motion_thread()
        if hasattr(self, 'servo'):
            self.servo.close()
        logger.info("Axon servo closed")


class AxonServoClosedLoop(ServoInterface):
    """
    Axon MAX MK2 servo with closed-loop control
    Uses ADS1115 ADC to read analog position feedback
    Implements PID control for accurate positioning
    """

    def __init__(
        self,
        gpio_pwm: int = 18,  # Hardware PWM pin (required for Pi 5)
        pwm_frequency: int = 50,
        pwm_min: int = 500,
        pwm_max: int = 2500,
        min_angle: float = 0.0,
        max_angle: float = 180.0,
        ads1115_address: int = 0x48,
        ads1115_channel: int = 0,
        voltage_min: float = 0.5,
        voltage_max: float = 4.5,
        kp: float = 2.0,
        ki: float = 0.1,
        kd: float = 0.3,
        deadzone_degrees: float = 1.0,
        control_rate_hz: float = 50.0
    ):
        """
        Initialize Axon servo with closed-loop control

        Args:
            gpio_pwm: GPIO pin for PWM signal
            pwm_frequency: PWM frequency in Hz
            pwm_min: Minimum pulse width in microseconds
            pwm_max: Maximum pulse width in microseconds
            min_angle: Minimum angle in degrees
            max_angle: Maximum angle in degrees
            ads1115_address: I2C address of ADS1115
            ads1115_channel: ADC channel (0-3)
            voltage_min: Voltage at minimum angle
            voltage_max: Voltage at maximum angle
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            deadzone_degrees: Deadzone for position error
            control_rate_hz: Control loop frequency
        """
        self.gpio_pwm = gpio_pwm
        self.pwm_frequency = pwm_frequency
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.voltage_min = voltage_min
        self.voltage_max = voltage_max

        # PID parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deadzone_degrees = deadzone_degrees
        self.control_rate_hz = control_rate_hz
        self.control_period = 1.0 / control_rate_hz

        # State
        self.target_angle = 90.0
        self.current_angle = 90.0
        self.integral = 0.0
        self.last_error = 0.0
        self.lock = threading.Lock()

        # Control loop thread
        self.running = False
        self.control_thread: Optional[threading.Thread] = None

        # Initialize PWM servo (same as AxonServo)
        try:
            from gpiozero import Servo as GPIOZeroServo
            from gpiozero.pins.lgpio import LGPIOFactory

            factory = LGPIOFactory()

            min_pulse_width = pwm_min / 1_000_000
            max_pulse_width = pwm_max / 1_000_000

            self.servo = GPIOZeroServo(
                gpio_pwm,
                min_pulse_width=min_pulse_width,
                max_pulse_width=max_pulse_width,
                frame_width=1.0 / pwm_frequency,
                pin_factory=factory
            )

            # Initialize ADS1115 for position feedback
            self.adc = ADS1115Reader(
                i2c_address=ads1115_address,
                channel=ads1115_channel
            )

            logger.info(f"Axon servo (closed-loop) initialized: GPIO{gpio_pwm}, PID=({kp},{ki},{kd})")

            # Start control loop
            self.start_control_loop()

        except Exception as e:
            logger.error(f"Failed to initialize Axon servo (closed-loop): {e}")
            raise

    def start_control_loop(self):
        """Start closed-loop control thread"""
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        logger.debug("Servo control loop started")

    def stop_control_loop(self):
        """Stop closed-loop control thread"""
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        logger.debug("Servo control loop stopped")

    def _control_loop(self):
        """Closed-loop PID control thread"""
        next_update_time = time.time()

        while self.running:
            try:
                # Read current position from ADC
                actual_angle = self.adc.voltage_to_angle(
                    self.voltage_min,
                    self.voltage_max,
                    self.min_angle,
                    self.max_angle
                )

                with self.lock:
                    self.current_angle = actual_angle

                    # Calculate error
                    error = self.target_angle - actual_angle

                    # Apply deadzone
                    if abs(error) < self.deadzone_degrees:
                        error = 0.0
                        self.integral = 0.0  # Reset integral in deadzone

                    # PID calculation
                    p_term = self.kp * error

                    # Integral with anti-windup
                    if abs(error) > 0.1:  # Only integrate if outside deadzone
                        self.integral += error * self.control_period
                        # Clamp integral
                        max_integral = 20.0
                        self.integral = max(-max_integral, min(max_integral, self.integral))

                    i_term = self.ki * self.integral

                    # Derivative
                    error_rate = (error - self.last_error) / self.control_period
                    d_term = self.kd * error_rate

                    # Total control output (angle correction)
                    control_output = p_term + i_term + d_term

                    # Convert to servo command
                    commanded_angle = actual_angle + control_output
                    commanded_angle = max(self.min_angle, min(self.max_angle, commanded_angle))

                    # Set servo position
                    normalized = (commanded_angle - self.min_angle) / (self.max_angle - self.min_angle)
                    servo_value = (normalized * 2.0) - 1.0

                    self.servo.value = servo_value

                    # Update state
                    self.last_error = error

                # Fixed-rate timing
                next_update_time += self.control_period
                sleep_time = next_update_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # Overrun - reset timing
                    next_update_time = time.time()
                    logger.warning("Servo control loop overrun")

            except Exception as e:
                logger.error(f"Error in servo control loop: {e}")
                time.sleep(0.1)

    def set_angle(self, angle: float):
        """
        Set target servo angle

        Args:
            angle: Target angle in degrees
        """
        with self.lock:
            # Clamp to limits
            angle = max(self.min_angle, min(self.max_angle, angle))
            self.target_angle = angle

    def get_angle(self) -> float:
        """
        Get current servo angle (from ADC feedback)

        Returns:
            Current actual angle in degrees
        """
        with self.lock:
            return self.current_angle

    def close(self):
        """Clean up resources"""
        self.stop_control_loop()
        if hasattr(self, 'servo'):
            self.servo.close()
        if hasattr(self, 'adc'):
            self.adc.close()
        logger.info("Axon servo (closed-loop) closed")


class SimulatedServo(ServoInterface):
    """
    Simulated servo for testing without hardware
    Models realistic servo motion with slew rate
    """

    def __init__(
        self,
        min_angle: float = 0.0,
        max_angle: float = 180.0,
        slew_rate_dps: float = 90.0
    ):
        """
        Initialize simulated servo

        Args:
            min_angle: Minimum angle in degrees
            max_angle: Maximum angle in degrees
            slew_rate_dps: Maximum slew rate in degrees per second
        """
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.slew_rate_dps = slew_rate_dps

        self.current_angle = 90.0
        self.target_angle = 90.0
        self.last_update_time = time.time()
        self.lock = threading.Lock()

        logger.info(f"Simulated servo initialized: slew_rate={slew_rate_dps}dps")

    def _update_position(self):
        """Update simulated position with slew rate limiting"""
        current_time = time.time()
        dt = current_time - self.last_update_time

        if dt > 0:
            # Calculate maximum movement
            max_movement = self.slew_rate_dps * dt

            # Move towards target
            error = self.target_angle - self.current_angle

            if abs(error) <= max_movement:
                # Reached target
                self.current_angle = self.target_angle
            else:
                # Move at slew rate
                direction = 1.0 if error > 0 else -1.0
                self.current_angle += direction * max_movement

            self.last_update_time = current_time

    def set_angle(self, angle: float):
        """
        Set servo angle

        Args:
            angle: Target angle in degrees
        """
        with self.lock:
            # Update current position first
            self._update_position()

            # Clamp to limits
            angle = max(self.min_angle, min(self.max_angle, angle))
            self.target_angle = angle

    def get_angle(self) -> float:
        """
        Get current servo angle

        Returns:
            Current simulated angle in degrees
        """
        with self.lock:
            self._update_position()
            return self.current_angle

    def close(self):
        """Clean up (no-op for simulation)"""
        logger.info("Simulated servo closed")
