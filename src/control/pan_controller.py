"""
Pan controller for PTDTS
Manages DC motor control with both position and velocity modes
"""

import time
import logging
import threading
from enum import Enum
from typing import Optional
from .pid import PIDController
from ..utils.timing_monitor import LoopTimingMonitor

logger = logging.getLogger(__name__)


class PanMode(Enum):
    """Pan control modes"""
    IDLE = "idle"  # Motor stopped
    POSITION = "position"  # Position-based control (acoustic panning)
    VELOCITY = "velocity"  # Continuous velocity control (tracking)
    MANUAL = "manual"  # Manual velocity command


class PanController:
    """
    Pan axis controller
    Supports both position-based and continuous velocity control modes
    """

    def __init__(
        self,
        motor,
        encoder,
        config
    ):
        """
        Initialize pan controller

        Args:
            motor: Motor interface
            encoder: Encoder interface
            config: Configuration object
        """
        self.motor = motor
        self.encoder = encoder
        self.config = config

        # Control mode
        self.mode = PanMode.IDLE
        self.lock = threading.Lock()

        # Position control PID with angle wraparound handling
        self.position_pid = PIDController(
            kp=config.pan_motor.position_pid['kp'],
            ki=config.pan_motor.position_pid['ki'],
            kd=config.pan_motor.position_pid['kd'],
            deadzone=config.pan_motor.position_pid['deadzone_degrees'],
            output_limits=(-config.pan_motor.max_velocity_dps, config.pan_motor.max_velocity_dps),
            error_function=self._angle_error  # Handle 0°/360° wraparound correctly
        )

        # Velocity control PID
        self.velocity_pid = PIDController(
            kp=config.pan_motor.velocity_pid['kp'],
            ki=config.pan_motor.velocity_pid['ki'],
            kd=config.pan_motor.velocity_pid['kd'],
            output_limits=(-config.pan_motor.max_pwm, config.pan_motor.max_pwm)
        )

        # Tracking state
        self.target_position = 0.0  # For POSITION mode
        self.target_velocity = 0.0  # For VELOCITY mode
        self.manual_pwm = 0.0  # For MANUAL mode (direct PWM control)

        # Panning state
        self.is_panning = False
        self.panning_complete = False

        # Motor stuck detection for startup boost/min_pwm
        self.motor_stopped_threshold_dps = 0.5  # Consider stopped if |velocity| < this
        self.motor_stuck_time_threshold = 0.2  # Apply boost/min_pwm after stopped for this long
        self.last_moving_time = time.time()  # Last time motor had significant velocity
        self.motor_is_stuck = False  # True if motor has been stopped for threshold time

        # Control loop
        self.running = False
        self.control_thread: Optional[threading.Thread] = None
        self.control_period = 1.0 / config.pan_motor.control_rate_hz

        # Timing monitoring
        self.timing_monitor = LoopTimingMonitor(
            target_rate_hz=config.pan_motor.control_rate_hz,
            window_size=100
        )

        logger.info(f"Pan controller initialized: rate={config.pan_motor.control_rate_hz}Hz")

    def start(self):
        """Start control loop"""
        if not self.running:
            self.running = True
            self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
            self.control_thread.start()
            logger.info("Pan control loop started")

    def stop(self):
        """Stop control loop"""
        if self.running:
            self.running = False
            if self.control_thread:
                self.control_thread.join(timeout=2.0)
            self.motor.stop()
            logger.info("Pan control loop stopped")

    def set_position_target(self, angle_degrees: float):
        """
        Set target position and switch to POSITION mode

        Args:
            angle_degrees: Target angle in degrees (0-360)
        """
        with self.lock:
            self.target_position = angle_degrees
            self.mode = PanMode.POSITION
            self.is_panning = True
            self.panning_complete = False
            self.position_pid.set_setpoint(angle_degrees)
            logger.debug(f"Pan position target set: {angle_degrees:.1f}°")

    def set_velocity_target(self, velocity_dps: float):
        """
        Set target velocity and switch to VELOCITY mode (for tracking)

        Args:
            velocity_dps: Target velocity in degrees per second
        """
        with self.lock:
            self.target_velocity = velocity_dps
            if self.mode != PanMode.MANUAL:  # Don't override manual mode
                self.mode = PanMode.VELOCITY
            logger.debug(f"Pan velocity target set: {velocity_dps:.1f}°/s")

    def set_manual_velocity(self, velocity_dps: float):
        """
        Set manual velocity and switch to MANUAL mode

        DEPRECATED: This method is kept for API compatibility but now
        converts velocity to direct PWM for smooth control.
        Use set_manual_pwm() for direct PWM control.

        Args:
            velocity_dps: Manual velocity command in degrees per second
        """
        # Convert velocity to PWM for direct control (no PID)
        # Map velocity range (-180 to +180) to PWM range (-max_pwm to +max_pwm)
        max_vel = self.config.pan_motor.max_velocity_dps
        pwm = (velocity_dps / max_vel) * self.config.pan_motor.max_pwm

        with self.lock:
            self.manual_pwm = pwm
            self.mode = PanMode.MANUAL
            logger.debug(f"Pan manual velocity set: {velocity_dps:.1f}°/s → PWM: {pwm:.3f}")

    def set_manual_pwm(self, pwm: float):
        """
        Set manual PWM and switch to MANUAL mode (direct control)

        Args:
            pwm: Direct PWM duty cycle (-1.0 to 1.0)
        """
        with self.lock:
            self.manual_pwm = pwm
            self.mode = PanMode.MANUAL
            logger.debug(f"Pan manual PWM set: {pwm:.3f}")

    def set_tracking_pwm(self, pwm: float):
        """
        Set tracking PWM and switch to VELOCITY mode (using direct PWM)

        This method is for visual tracking mode. Unlike MANUAL mode which uses
        set_manual_pwm(), this keeps the controller in VELOCITY mode for proper
        state machine integration while using direct PWM control.

        When use_direct_pwm is enabled, the target_velocity field stores the
        PWM value instead of a velocity in degrees/second. This provides the
        same smooth, predictable control as manual mode while maintaining
        compatibility with the VELOCITY state.

        Args:
            pwm: Direct PWM duty cycle (-1.0 to 1.0)
        """
        with self.lock:
            old_mode = self.mode
            self.target_velocity = pwm  # Store PWM in target_velocity for VELOCITY mode
            if self.mode != PanMode.MANUAL:  # Don't override manual mode
                self.mode = PanMode.VELOCITY
            logger.info(f"🎯 TRACKING PWM: {pwm:.3f} | Mode: {old_mode.name}→{self.mode.name} | "
                       f"use_direct_pwm={self.config.tracking.use_direct_pwm}")

    def set_idle(self):
        """Set to IDLE mode (stop motor)"""
        with self.lock:
            self.mode = PanMode.IDLE
            self.is_panning = False
            logger.debug("Pan mode set to IDLE")

    def is_position_reached(self) -> bool:
        """
        Check if position target has been reached

        Returns:
            True if in POSITION mode and target reached
        """
        with self.lock:
            return self.mode == PanMode.POSITION and self.panning_complete

    def get_current_angle(self) -> float:
        """
        Get current pan angle

        Returns:
            Current angle in degrees
        """
        return self.encoder.get_angle()

    def get_current_velocity(self) -> float:
        """
        Get current pan velocity

        Returns:
            Current velocity in degrees per second
        """
        return self.encoder.get_velocity()

    def get_mode(self) -> PanMode:
        """
        Get current control mode

        Returns:
            Current PanMode
        """
        with self.lock:
            return self.mode

    def _control_loop(self):
        """Main control loop (runs at fixed rate)"""
        next_update_time = time.time()
        last_loop_time = time.time()

        while self.running:
            try:
                # Start timing measurement
                self.timing_monitor.start_iteration()

                # Read current state
                current_angle = self.encoder.get_angle()
                current_velocity = self.encoder.get_velocity()

                with self.lock:
                    mode = self.mode

                # Compute motor command based on mode
                if mode == PanMode.IDLE:
                    # Stop motor
                    pwm_command = 0.0

                elif mode == PanMode.POSITION:
                    # Position control with trapezoidal profile
                    error = self._angle_error(self.target_position, current_angle)

                    # Check if arrived
                    if abs(error) < self.config.pan_motor.panning['arrival_threshold_degrees']:
                        with self.lock:
                            self.panning_complete = True
                        pwm_command = 0.0
                    else:
                        # Calculate target velocity with approach deceleration
                        approach_distance = self.config.pan_motor.panning['approach_distance_degrees']
                        decel_factor = self.config.pan_motor.panning['decel_factor']

                        if abs(error) < approach_distance:
                            # Approaching target - reduce speed
                            speed_multiplier = decel_factor + (1.0 - decel_factor) * (abs(error) / approach_distance)
                        else:
                            # Far from target - full speed
                            speed_multiplier = 1.0

                        # Position PID outputs target velocity
                        target_vel = self.position_pid.compute(current_angle, self.control_period)
                        target_vel *= speed_multiplier

                        # Velocity PID converts to PWM
                        self.velocity_pid.set_setpoint(target_vel)
                        pwm_command = self.velocity_pid.compute(current_velocity, self.control_period)

                elif mode == PanMode.VELOCITY:
                    # Continuous velocity control (tracking)
                    # Check if using direct PWM (like MANUAL mode) or legacy velocity PID
                    if self.config.tracking.use_direct_pwm:
                        # Direct PWM control - target_velocity stores PWM value
                        # This provides smooth, predictable control without oscillation
                        with self.lock:
                            pwm_command = self.target_velocity
                        # Log PWM application in control loop (at 50Hz)
                        if abs(pwm_command) > 0.001:  # Only log non-zero commands
                            logger.debug(f"⚙️ CONTROL LOOP: Applying PWM={pwm_command:.3f} in VELOCITY mode")
                    else:
                        # Legacy velocity PID control - target_velocity stores deg/s
                        self.velocity_pid.set_setpoint(self.target_velocity)
                        pwm_command = self.velocity_pid.compute(current_velocity, self.control_period)

                elif mode == PanMode.MANUAL:
                    # Manual DIRECT PWM control (no PID - like test_pan_motor.py)
                    # This provides smooth, predictable control without oscillation
                    with self.lock:
                        pwm_command = self.manual_pwm

                else:
                    pwm_command = 0.0

                # Update motor stuck detection state
                # Track when motor last had significant velocity to distinguish:
                # - Motor stuck (stopped for >0.2s) → needs boost/min_pwm
                # - Intentional slow motion (PID outputting gentle commands) → no boost/min_pwm
                if abs(current_velocity) > self.motor_stopped_threshold_dps:
                    # Motor is moving - update last_moving_time
                    self.last_moving_time = time.time()
                    self.motor_is_stuck = False
                else:
                    # Motor stopped - check if stuck for long enough
                    stopped_duration = time.time() - self.last_moving_time
                    self.motor_is_stuck = stopped_duration > self.motor_stuck_time_threshold

                # Apply startup boost for static friction (ONLY when motor is stuck)
                if abs(pwm_command) > 0.01 and self.motor_is_stuck:
                    # Motor has been stuck for >0.2s and commanded to move - apply boost
                    boost = self.config.pan_motor.startup_boost
                    if pwm_command > 0:
                        pwm_command = max(pwm_command, boost)
                    else:
                        pwm_command = min(pwm_command, -boost)

                # Apply minimum PWM threshold (ONLY when motor is stuck)
                # Once moving, allow PID to output gentle commands for fine control
                if self.motor_is_stuck and abs(pwm_command) < self.config.pan_motor.min_pwm and abs(pwm_command) > 0.01:
                    if pwm_command > 0:
                        pwm_command = self.config.pan_motor.min_pwm
                    else:
                        pwm_command = -self.config.pan_motor.min_pwm

                # Send command to motor
                self.motor.set_pwm(pwm_command)

                # End timing measurement
                self.timing_monitor.end_iteration()

                # Record actual loop period
                current_loop_time = time.time()
                loop_period = current_loop_time - last_loop_time
                self.timing_monitor.record_loop_period(loop_period)
                last_loop_time = current_loop_time

                # Fixed-rate timing
                next_update_time += self.control_period
                sleep_time = next_update_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # Loop overrun
                    next_update_time = time.time()
                    if self.config.performance.warn_loop_overrun:
                        logger.warning(f"Pan control loop overrun: {-sleep_time*1000:.1f}ms")

            except Exception as e:
                logger.error(f"Error in pan control loop: {e}")
                time.sleep(0.1)

    def _angle_error(self, target: float, current: float) -> float:
        """
        Calculate angle error with wrap-around handling

        Args:
            target: Target angle (0-360)
            current: Current angle (0-360)

        Returns:
            Error in degrees (shortest path)
        """
        error = target - current

        # Wrap to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        return error

    def get_status(self) -> dict:
        """
        Get current controller status for telemetry

        Returns:
            Dictionary with current status information
        """
        with self.lock:
            # Get current position and velocity from encoder
            current_angle = self.encoder.get_angle()
            current_velocity = self.encoder.get_velocity()

            status = {
                'mode': self.mode.value,
                'position_deg': current_angle,
                'velocity_dps': current_velocity,
                'target_position': self.target_position if self.mode == PanMode.POSITION else None,
                'target_velocity': self.target_velocity if self.mode == PanMode.VELOCITY else None,
                'manual_pwm': self.manual_pwm if self.mode == PanMode.MANUAL else None,
                'encoder': self.encoder.get_count(),
                'pwm': getattr(self.motor, 'current_pwm', 0.0),
                'panning_complete': self.panning_complete
            }

            # Add motor status if available
            if hasattr(self.motor, 'get_status'):
                status['motor'] = self.motor.get_status()

            # Add timing metrics
            status['timing'] = self.timing_monitor.get_metrics()

            return status

    def save_state(self, state_manager) -> None:
        """
        Save controller state to StateManager.

        Args:
            state_manager: StateManager instance to save state to
        """
        with self.lock:
            # Get PID state
            pid_integral = self.velocity_pid.integral
            pid_last_error = self.velocity_pid.last_error

            # Save pan controller state
            state_manager.set_pan_controller_state(
                mode=self.mode.value,
                target_position=self.target_position if self.mode == PanMode.POSITION else None,
                target_velocity=self.target_velocity if self.mode == PanMode.VELOCITY else (
                    self.manual_pwm if self.mode == PanMode.MANUAL else None
                ),
                pid_integral=pid_integral,
                pid_last_error=pid_last_error
            )

            # Save encoder state (absolute zero reference)
            state_manager.set_encoder_state(
                count=self.encoder.get_count(),
                angle=self.encoder.get_angle(),
                velocity=self.encoder.get_velocity()
            )

            logger.info(f"Pan controller state saved: mode={self.mode.value}, angle={self.encoder.get_angle():.2f}°")

    def load_state(self, state_manager) -> bool:
        """
        Load controller state from StateManager.

        Args:
            state_manager: StateManager instance to load state from

        Returns:
            True if state was loaded successfully, False otherwise
        """
        # Load encoder state first (absolute zero reference)
        encoder_state = state_manager.get_encoder_state()
        if encoder_state:
            # Restore encoder position
            saved_count = encoder_state.get('count', 0)
            saved_angle = encoder_state.get('angle', 0.0)

            # Set encoder to saved position
            if hasattr(self.encoder, 'set_count'):
                self.encoder.set_count(saved_count)
                logger.info(f"Restored encoder position: count={saved_count}, angle={saved_angle:.2f}°")
            else:
                logger.warning("Encoder does not support set_count() - cannot restore position")

        # Load pan controller state
        pan_state = state_manager.get_pan_controller_state()
        if pan_state:
            with self.lock:
                # Restore mode
                mode_str = pan_state.get('mode', 'idle')
                try:
                    self.mode = PanMode(mode_str)
                except ValueError:
                    self.mode = PanMode.IDLE
                    logger.warning(f"Invalid saved mode '{mode_str}', defaulting to IDLE")

                # Restore targets
                self.target_position = pan_state.get('target_position', 0.0)
                saved_velocity = pan_state.get('target_velocity', 0.0)

                # For MANUAL mode, restore as manual_pwm; for VELOCITY mode, restore as target_velocity
                if self.mode == PanMode.MANUAL:
                    self.manual_pwm = saved_velocity  # Was saved as target_velocity for compatibility
                else:
                    self.target_velocity = saved_velocity

                # Restore PID state
                self.velocity_pid.integral = pan_state.get('pid_integral', 0.0)
                self.velocity_pid.last_error = pan_state.get('pid_last_error', 0.0)

                # If we were in POSITION mode, restore the setpoint
                if self.mode == PanMode.POSITION and self.target_position is not None:
                    self.position_pid.set_setpoint(self.target_position)

                logger.info(f"Pan controller state restored: mode={self.mode.value}, " +
                           f"target_pos={self.target_position}, target_vel={self.target_velocity}")

            return True
        else:
            logger.info("No saved pan controller state found")
            return False

    def close(self):
        """Clean up resources"""
        self.stop()
        logger.info("Pan controller closed")
