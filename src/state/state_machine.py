"""
State machine for PTDTS
Manages system states and transitions
"""

import time
import logging
import threading
from enum import Enum
from typing import Optional, Dict, Any
from dataclasses import dataclass

logger = logging.getLogger(__name__)


class SystemState(Enum):
    """System states"""
    LISTENING = "listening"  # Acoustic monitoring
    PANNING = "panning"  # Slewing to acoustic bearing
    DETECTING = "detecting"  # Visual search at location
    TRACKING = "tracking"  # Active tracking
    MANUAL = "manual"  # Manual control
    ERROR = "error"  # Error state


@dataclass
class StateTransition:
    """State transition record"""
    from_state: SystemState
    to_state: SystemState
    timestamp: float
    reason: str


class StateMachine:
    """
    PTDTS state machine
    Coordinates all subsystems based on current state
    """

    def __init__(
        self,
        pan_controller,
        tilt_controller,
        camera_manager,
        visual_detector,
        acoustic_detector,
        config
    ):
        """
        Initialize state machine

        Args:
            pan_controller: Pan axis controller
            tilt_controller: Tilt axis controller
            camera_manager: Dual camera manager
            visual_detector: Visual detector (YOLO)
            acoustic_detector: Acoustic detector (ODAS)
            config: Configuration object
        """
        self.pan_controller = pan_controller
        self.tilt_controller = tilt_controller
        self.camera_manager = camera_manager
        self.visual_detector = visual_detector
        self.acoustic_detector = acoustic_detector
        self.config = config

        # Current state
        self.current_state = SystemState.LISTENING
        self.lock = threading.Lock()

        # State timers
        self.state_entry_time = time.time()
        self.last_detection_time = 0.0

        # Acoustic detection queue
        self.pending_acoustic_detection: Optional[Dict] = None

        # Visual detection state
        self.current_target: Optional[Dict] = None
        self.tracking_lost_count = 0
        self.visual_confirmation_frames = 0
        self.last_tracking_pwm = 0.0  # Last PWM command sent during tracking
        self.target_lost_start_time = None  # When target was first lost

        # Position-based tracking state (prevents oscillation)
        self.last_position_command_time = 0.0  # When we last sent a position command
        self.last_pixel_error = 0.0  # Pixel error from last command

        # State history
        self.state_history = []
        self.max_history = 100

        # Manual mode state
        self.manual_mode_requested = False

        logger.info(f"State machine initialized: initial_state={self.current_state.value}")

    def update(self):
        """
        Update state machine
        Called periodically from main loop
        """
        with self.lock:
            current_time = time.time()
            state_duration = current_time - self.state_entry_time

            # Check for manual mode request
            if self.manual_mode_requested and self.current_state != SystemState.MANUAL:
                self._transition_to(SystemState.MANUAL, "Manual mode requested")
                return

            # State-specific behavior and transitions
            if self.current_state == SystemState.LISTENING:
                self._listening_behavior()

            elif self.current_state == SystemState.PANNING:
                self._panning_behavior(state_duration)

            elif self.current_state == SystemState.DETECTING:
                self._detecting_behavior(state_duration)

            elif self.current_state == SystemState.TRACKING:
                self._tracking_behavior(state_duration)

            elif self.current_state == SystemState.MANUAL:
                self._manual_behavior()

            elif self.current_state == SystemState.ERROR:
                self._error_behavior()

    def _listening_behavior(self):
        """LISTENING state behavior"""
        # Check for visual detections first (opportunistic tracking)
        # Capture frame from current camera (should be HQ in LISTENING)
        frame, camera_name = self.camera_manager.capture_frame()

        if frame is not None and self.visual_detector is not None:
            # Run visual detection
            visual_detections = self.visual_detector.detect(frame)

            # Filter by detection confidence
            valid_visual = [
                d for d in visual_detections
                if d.confidence >= self.config.yolo.detection_confidence
            ]

            if valid_visual:
                # Visual detection found - require consecutive frames before switching cameras
                best = valid_visual[0]
                self.current_target = {
                    'center_x': best.center[0],
                    'center_y': best.center[1],
                    'confidence': best.confidence,
                    'class_name': best.class_name,
                    'timestamp': time.time()
                }

                # Increment confirmation counter
                self.visual_confirmation_frames += 1

                # Require multiple consecutive detections to prevent camera thrashing
                required_frames = self.config.state_machine.detection['visual_confirmation_frames']
                if self.visual_confirmation_frames >= required_frames:
                    logger.info(f"Visual confirmation in LISTENING mode: {best.class_name} "
                               f"(conf: {best.confidence:.2f}, {self.visual_confirmation_frames} frames) - transitioning to TRACKING")

                    # Check if drone is centered enough for GS camera switch
                    frame_width = frame.shape[1]
                    pixel_error = best.center[0] - (frame_width / 2)
                    percent_off_center = abs(pixel_error) / (frame_width / 2) * 100
                    center_threshold = self.config.cameras.switching.get('gs_center_threshold_percent', 40)

                    # Only switch to GS camera if drone is reasonably centered
                    if percent_off_center <= center_threshold:
                        self.camera_manager.switch_to_gs()
                        settling_delay = self.config.cameras.switching['settling_delay_ms'] / 1000.0
                        time.sleep(settling_delay)
                        logger.info(f"Switched to GS camera (target {percent_off_center:.1f}% off-center, threshold {center_threshold}%)")
                    else:
                        # Stay on HQ camera - wider FOV safer for off-center tracking
                        logger.info(f"Staying on HQ camera - target {percent_off_center:.1f}% off-center (threshold {center_threshold}%)")

                    # Reset tracking state for clean start
                    self.last_tracking_pwm = 0.0
                    self.visual_confirmation_frames = 0

                    # Transition directly to TRACKING (skip PANNING and DETECTING)
                    self._transition_to(
                        SystemState.TRACKING,
                        f"Visual detection: {best.class_name} (conf: {best.confidence:.2f})"
                    )
                    return
                else:
                    logger.debug(f"Visual detection in LISTENING mode: {best.class_name} "
                                f"(conf: {best.confidence:.2f}) - confirming ({self.visual_confirmation_frames}/{required_frames})")
            else:
                # No detection - reset confirmation counter
                self.visual_confirmation_frames = 0

        # Check for acoustic detections
        detections = self.acoustic_detector.get_detections()

        if detections:
            # Filter by energy threshold
            valid_detections = [
                d for d in detections
                if d['energy'] >= self.config.acoustic.energy_threshold
            ]

            if valid_detections:
                # Take most recent high-energy detection
                latest = max(valid_detections, key=lambda d: d['timestamp'])

                # Queue acoustic detection
                self.pending_acoustic_detection = latest

                # Transition to PANNING
                self._transition_to(
                    SystemState.PANNING,
                    f"Acoustic detection at {latest['azimuth']:.1f}°"
                )

    def _panning_behavior(self, state_duration: float):
        """PANNING state behavior"""
        # Check for timeout
        if state_duration > self.config.state_machine.timeouts['panning']:
            logger.warning("Panning timeout")
            self._transition_to(SystemState.LISTENING, "Panning timeout")
            return

        # Check if pan has arrived at target
        if self.pan_controller.is_position_reached():
            logger.info("Arrived at acoustic bearing")

            # Switch to HQ camera for detection
            self.camera_manager.switch_to_hq()

            # Wait for camera to settle
            time.sleep(self.config.cameras.switching['settling_delay_ms'] / 1000.0)

            # Transition to DETECTING
            self._transition_to(SystemState.DETECTING, "Arrived at target position")

    def _detecting_behavior(self, state_duration: float):
        """DETECTING state behavior"""
        # Check for timeout
        if state_duration > self.config.state_machine.timeouts['detecting']:
            logger.info("Detection timeout, returning to listening")
            self._transition_to(SystemState.LISTENING, "Detection timeout")
            return

        # Capture frame and detect
        frame, camera_name = self.camera_manager.capture_frame()

        # Run detection
        detections = self.visual_detector.detect(frame)

        # Filter by confidence
        valid_detections = [
            d for d in detections
            if d.confidence >= self.config.yolo.detection_confidence
        ]

        if valid_detections:
            # Increment confirmation counter
            self.visual_confirmation_frames += 1

            # Update current target
            best = valid_detections[0]
            self.current_target = {
                'center_x': best.center[0],
                'center_y': best.center[1],
                'confidence': best.confidence,
                'class_name': best.class_name,
                'timestamp': time.time()
            }

            # Check if we have enough confirmation frames
            required_frames = self.config.state_machine.detection['visual_confirmation_frames']
            if self.visual_confirmation_frames >= required_frames:
                logger.info(f"Visual confirmation complete ({self.visual_confirmation_frames} frames)")

                # Check if drone is centered enough for GS camera switch
                frame_width = frame.shape[1]
                pixel_error = best.center[0] - (frame_width / 2)
                percent_off_center = abs(pixel_error) / (frame_width / 2) * 100
                center_threshold = self.config.cameras.switching.get('gs_center_threshold_percent', 40)

                # Only switch to GS camera if drone is reasonably centered
                if percent_off_center <= center_threshold:
                    self.camera_manager.switch_to_gs()
                    settling_delay = self.config.cameras.switching['settling_delay_ms'] / 1000.0
                    logger.debug(f"Camera switching - waiting {settling_delay*1000:.0f}ms for stabilization")
                    time.sleep(settling_delay)
                    logger.info(f"Switched to GS camera (target {percent_off_center:.1f}% off-center, threshold {center_threshold}%)")
                else:
                    # Stay on HQ camera - wider FOV safer for off-center tracking
                    logger.info(f"Staying on HQ camera - target {percent_off_center:.1f}% off-center (threshold {center_threshold}%)")

                # Reset tracking state to prevent using stale HQ camera target
                self.last_tracking_pwm = 0.0

                # Transition to TRACKING
                self._transition_to(SystemState.TRACKING, "Visual confirmation complete")
        else:
            # Reset confirmation counter if no detection
            self.visual_confirmation_frames = 0

    def _tracking_behavior(self, state_duration: float):
        """TRACKING state behavior"""
        # Check for timeout
        if state_duration > self.config.state_machine.timeouts['tracking']:
            logger.info("Tracking duration limit reached")
            self._transition_to(SystemState.LISTENING, "Tracking time limit")
            return

        # Capture frame and detect
        frame, camera_name = self.camera_manager.capture_frame()

        # Get actual frame dimensions (NOT hardcoded to GS camera!)
        # This is critical because frame could be from either HQ (1280x720) or GS (640x480)
        frame_height, frame_width = frame.shape[:2]

        # Run detection with lower threshold for tracking
        detections = self.visual_detector.detect(frame)

        # Filter by tracking confidence
        valid_detections = [
            d for d in detections
            if d.confidence >= self.config.yolo.tracking_confidence
        ]

        if valid_detections:
            # Target found - reset lost counter and timer
            self.tracking_lost_count = 0
            self.target_lost_start_time = None  # Reset reacquisition timer

            # Update current target
            best = valid_detections[0]

            # Validate detection is within frame bounds
            if (best.center[0] < 0 or best.center[0] >= frame_width or
                best.center[1] < 0 or best.center[1] >= frame_height):
                logger.warning(f"Detection out of bounds: center=({best.center[0]:.0f}, {best.center[1]:.0f}), "
                               f"frame=({frame_width}, {frame_height}) - treating as target lost")
                # Treat as target lost
                self.tracking_lost_count += 1
                # Continue to target lost logic below
            else:
                self.current_target = {
                    'center_x': best.center[0],
                    'center_y': best.center[1],
                    'confidence': best.confidence,
                    'class_name': best.class_name,
                    'timestamp': time.time()
                }

                # POSITION-BASED TRACKING (prevents overshoot with slow YOLO detection)
                # Convert pixel error to angle offset and pan to that position
                pixel_error = best.center[0] - (frame_width / 2)

                # Get camera FOV based on which camera is active
                if camera_name == 'hq':
                    fov_horizontal = self.config.cameras.hq.fov_h
                else:
                    fov_horizontal = self.config.cameras.gs.fov_h

                # Convert pixel error to angle offset with DAMPING
                # Damping factor (0.3-0.6) makes tracking iterative:
                # - Pan only partial way toward target
                # - Next detection corrects any lens distortion error
                # - Converges to center in 2-4 detections
                # - Prevents overshoot and handles fisheye lens naturally
                full_angle_offset = pixel_error * (fov_horizontal / frame_width)
                damping = self.config.tracking.position_damping
                angle_offset = full_angle_offset * damping

                # Apply deadzone (prevent jitter when nearly centered)
                deadzone_pixels = self.config.tracking.deadzone_pixels
                if abs(pixel_error) < deadzone_pixels:
                    angle_offset = 0.0
                    logger.debug(f"TRACKING: target centered (px_err={pixel_error:.0f} < deadzone={deadzone_pixels})")
                else:
                    # HYBRID APPROACH: Only send position command when appropriate
                    # This prevents oscillation from rapid-fire commands

                    current_time = time.time()
                    time_since_last_command = current_time - self.last_position_command_time

                    # Check if motor has settled at previous target
                    motor_settled = self.pan_controller.is_position_reached()

                    # Check if detection moved significantly since last command
                    pixel_error_change = abs(pixel_error - self.last_pixel_error)
                    significant_change = pixel_error_change > deadzone_pixels

                    # Decide whether to send new position command
                    # STRICT: Only command if motor is settled OR target moved significantly with timeout
                    # This prevents commanding while motor is mid-movement (which causes overshoot)
                    # Timeout increased to 2.0s to ensure motor fully settles before new commands
                    should_command = motor_settled or (significant_change and time_since_last_command > 2.0)

                    if should_command:
                        # Get current angle and calculate target
                        current_angle = self.pan_controller.get_current_angle()
                        target_angle = current_angle + angle_offset

                        # Command motor to pan to target position and STOP
                        self.pan_controller.set_position_target(target_angle)

                        # Update tracking state
                        self.last_position_command_time = current_time
                        self.last_pixel_error = pixel_error

                        logger.info(f"TRACKING: px_err={pixel_error:.0f}, full_offset={full_angle_offset:.2f}°, "
                                   f"damped={angle_offset:.2f}° (damping={damping:.2f}), "
                                   f"current={current_angle:.1f}°, target={target_angle:.1f}°, "
                                   f"detection@{best.center[0]:.0f}/{frame_width} "
                                   f"[settled={motor_settled}, change={pixel_error_change:.0f}px, dt={time_since_last_command:.2f}s]")
                    else:
                        # Skip this command - motor still moving or no significant change
                        logger.debug(f"TRACKING: skipping command (motor moving, px_err={pixel_error:.0f}, "
                                    f"change={pixel_error_change:.0f}px, dt={time_since_last_command:.2f}s)")

                # Update last detection time
                self.last_detection_time = time.time()

        else:
            # Target lost - increment counter
            self.tracking_lost_count += 1

            # Mark when target was first lost
            if self.target_lost_start_time is None:
                self.target_lost_start_time = time.time()
                logger.warning(f"Target lost - will wait {self.config.tracking.reacquisition_timeout_seconds}s for reacquisition")

            # Calculate time since target first lost
            time_since_lost = time.time() - self.target_lost_start_time

            # POSITION-BASED TRACKING: Don't spin continuously
            # Motor is already stopped at last position, just wait for next detection
            # If target moves back into view, detection will trigger new position command
            if time_since_lost < self.config.tracking.reacquisition_timeout_seconds:
                # Still within reacquisition window - hold position and wait
                # Pan controller is already in POSITION mode and holding angle
                logger.debug(f"REACQUISITION: holding position, waiting for detection ({time_since_lost:.1f}s elapsed)")
            else:
                # Reacquisition timeout - return to LISTENING
                logger.info(f"Target lost for {time_since_lost:.1f}s, returning to listening")

                # Stop pan motor
                self.pan_controller.set_idle()

                # Switch back to HQ camera
                self.camera_manager.switch_to_hq()

                # Transition to LISTENING
                self._transition_to(SystemState.LISTENING, "Target lost")

    def _manual_behavior(self):
        """MANUAL state behavior"""
        # In manual mode, pan controller is in MANUAL mode
        # Commands come from web interface or gamepad
        pass

    def _error_behavior(self):
        """ERROR state behavior"""
        # Stop all motion
        self.pan_controller.set_idle()

        # Log error state
        logger.error("System in ERROR state")

        # TODO: Implement error recovery

    def _transition_to(self, new_state: SystemState, reason: str):
        """
        Transition to new state

        Args:
            new_state: Target state
            reason: Reason for transition
        """
        if new_state == self.current_state:
            return

        old_state = self.current_state

        # Record transition
        transition = StateTransition(
            from_state=old_state,
            to_state=new_state,
            timestamp=time.time(),
            reason=reason
        )
        self.state_history.append(transition)

        # Trim history
        if len(self.state_history) > self.max_history:
            self.state_history.pop(0)

        # Exit old state
        self._exit_state(old_state)

        # Update state
        self.current_state = new_state
        self.state_entry_time = time.time()

        # Enter new state
        self._enter_state(new_state)

        logger.info(f"State transition: {old_state.value} → {new_state.value} ({reason})")

    def _enter_state(self, state: SystemState):
        """
        Handle state entry

        Args:
            state: State being entered
        """
        if state == SystemState.LISTENING:
            # Ensure pan is idle
            self.pan_controller.set_idle()

            # Use HQ camera
            self.camera_manager.switch_to_hq()

            # Clear pending detection
            self.pending_acoustic_detection = None

            # Reset confirmation counter
            self.visual_confirmation_frames = 0

        elif state == SystemState.PANNING:
            # Set pan target from acoustic detection
            if self.pending_acoustic_detection:
                target_azimuth = self.pending_acoustic_detection['azimuth']
                self.pan_controller.set_position_target(target_azimuth)

        elif state == SystemState.DETECTING:
            # Reset confirmation counter
            self.visual_confirmation_frames = 0

            # Ensure using HQ camera
            self.camera_manager.switch_to_hq()

        elif state == SystemState.TRACKING:
            # Switch pan to velocity mode
            self.pan_controller.set_velocity_target(0.0)

            # Camera is already set by behavior method based on center threshold
            # (Don't force switch to GS here - respect the decision made in LISTENING/DETECTING)

            # Reset lost counter
            self.tracking_lost_count = 0
            self.last_detection_time = time.time()

            # Reset position tracking state (prevents oscillation)
            # CRITICAL: Initialize to current time, NOT 0.0 (which would be 55 years ago!)
            self.last_position_command_time = time.time()
            self.last_pixel_error = 0.0

        elif state == SystemState.MANUAL:
            # Pan controller already in MANUAL mode
            pass

        elif state == SystemState.ERROR:
            # Stop all motion
            self.pan_controller.set_idle()

    def _exit_state(self, state: SystemState):
        """
        Handle state exit

        Args:
            state: State being exited
        """
        # Cleanup specific to exiting state
        pass

    def queue_acoustic_detection(self, azimuth: float, elevation: float, energy: float):
        """
        Queue an acoustic detection for processing

        Args:
            azimuth: Direction in degrees (0-360)
            elevation: Elevation in degrees
            energy: Signal energy
        """
        with self.lock:
            if self.current_state == SystemState.LISTENING:
                self.pending_acoustic_detection = {
                    'azimuth': azimuth,
                    'elevation': elevation,
                    'energy': energy,
                    'timestamp': time.time()
                }
                logger.debug(f"Queued acoustic detection: {azimuth:.1f}°, energy={energy:.2f}")

    def request_manual_mode(self):
        """Request manual mode"""
        with self.lock:
            self.manual_mode_requested = True
            logger.info("Manual mode requested")

    def request_auto_mode(self):
        """Request return to auto mode"""
        with self.lock:
            if self.current_state == SystemState.MANUAL:
                self.manual_mode_requested = False
                self._transition_to(SystemState.LISTENING, "Auto mode requested")

    def request_state(self, state_name: str):
        """
        Request transition to a specific state

        Args:
            state_name: Name of the target state (e.g., 'listening', 'manual', etc.)

        Returns:
            bool: True if transition was successful, False otherwise

        Raises:
            ValueError: If state_name is not a valid state
        """
        with self.lock:
            # Convert string to SystemState enum
            try:
                if isinstance(state_name, str):
                    target_state = SystemState(state_name.lower())
                else:
                    target_state = state_name
            except ValueError:
                valid_states = [s.value for s in SystemState]
                raise ValueError(f"Invalid state '{state_name}'. Valid states: {valid_states}")

            # Update manual mode flag based on target state
            if target_state == SystemState.MANUAL:
                self.manual_mode_requested = True
            else:
                self.manual_mode_requested = False

            # Perform transition
            self._transition_to(target_state, f"Manual state change to {target_state.value}")
            logger.info(f"Manual state change requested: {target_state.value}")

            return True

    def get_current_state(self) -> SystemState:
        """
        Get current state

        Returns:
            Current SystemState
        """
        with self.lock:
            return self.current_state

    def get_state_info(self) -> Dict[str, Any]:
        """
        Get detailed state information

        Returns:
            Dictionary with state information
        """
        with self.lock:
            return {
                'state': self.current_state.value,
                'state_duration': time.time() - self.state_entry_time,
                'current_target': self.current_target,
                'tracking_lost_count': self.tracking_lost_count,
                'last_detection_time': self.last_detection_time,
                'pending_acoustic': self.pending_acoustic_detection,
                'visual_confirmation_frames': self.visual_confirmation_frames
            }

    def get_state_history(self, count: int = 10) -> list:
        """
        Get recent state transition history

        Args:
            count: Number of recent transitions to return

        Returns:
            List of recent StateTransition objects
        """
        with self.lock:
            return self.state_history[-count:]

    def save_state(self, state_manager) -> None:
        """
        Save state machine state to StateManager.

        Args:
            state_manager: StateManager instance to save state to
        """
        with self.lock:
            state_manager.set_state_machine_state(
                current_state=self.current_state.value,
                current_target=self.current_target,
                pending_acoustic=self.pending_acoustic_detection,
                tracking_lost_count=self.tracking_lost_count
            )
            logger.info(f"State machine state saved: state={self.current_state.value}")

    def load_state(self, state_manager) -> bool:
        """
        Load state machine state from StateManager.

        Args:
            state_manager: StateManager instance to load state from

        Returns:
            True if state was loaded successfully, False otherwise
        """
        sm_state = state_manager.get_state_machine_state()
        if sm_state:
            with self.lock:
                # Restore state (but always start in LISTENING for safety)
                # We could restore the exact state, but it's safer to start fresh
                saved_state = sm_state.get('current_state', 'listening')
                logger.info(f"Saved state was '{saved_state}', but starting in LISTENING for safety")

                # Note: We intentionally don't restore the current state to avoid
                # resuming in the middle of a tracking or panning operation.
                # Instead, we start fresh in LISTENING mode.
                self.current_state = SystemState.LISTENING
                self.state_entry_time = time.time()

                # Restore target info (informational only, not used immediately)
                self.current_target = sm_state.get('current_target', None)
                self.pending_acoustic_detection = None  # Don't restore pending detections
                self.tracking_lost_count = 0  # Reset counters

                logger.info("State machine restored to LISTENING mode")

            return True
        else:
            logger.info("No saved state machine state found")
            return False
