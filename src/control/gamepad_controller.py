"""
Gamepad controller for PTDTS
Xbox controller support for manual control
"""

import time
import logging
import threading
from typing import Optional, Callable, Dict, Any
from enum import Enum

logger = logging.getLogger(__name__)

try:
    from inputs import get_gamepad, UnpluggedError
    GAMEPAD_AVAILABLE = True
except ImportError:
    logger.warning("inputs library not available - gamepad support disabled")
    GAMEPAD_AVAILABLE = False


class GamepadButton(Enum):
    """Xbox controller button mapping"""
    A = "BTN_SOUTH"
    B = "BTN_EAST"
    X = "BTN_WEST"
    Y = "BTN_NORTH"
    LB = "BTN_TL"
    RB = "BTN_TR"
    BACK = "BTN_SELECT"
    START = "BTN_START"
    XBOX = "BTN_MODE"
    L_STICK = "BTN_THUMBL"
    R_STICK = "BTN_THUMBR"


class GamepadAxis(Enum):
    """Xbox controller axis mapping"""
    LEFT_X = "ABS_X"
    LEFT_Y = "ABS_Y"
    RIGHT_X = "ABS_RX"
    RIGHT_Y = "ABS_RY"
    LEFT_TRIGGER = "ABS_Z"
    RIGHT_TRIGGER = "ABS_RZ"
    DPAD_X = "ABS_HAT0X"
    DPAD_Y = "ABS_HAT0Y"


class GamepadState:
    """Current gamepad state"""
    def __init__(self):
        # Axes (normalized -1.0 to 1.0)
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.left_trigger = 0.0  # 0.0 to 1.0
        self.right_trigger = 0.0  # 0.0 to 1.0
        self.dpad_x = 0.0
        self.dpad_y = 0.0

        # Buttons (bool)
        self.button_a = False
        self.button_b = False
        self.button_x = False
        self.button_y = False
        self.button_lb = False
        self.button_rb = False
        self.button_back = False
        self.button_start = False
        self.button_xbox = False
        self.button_l_stick = False
        self.button_r_stick = False


class GamepadController:
    """
    Xbox controller input handler for PTDTS manual control

    Controls:
        Left Stick X: Pan velocity (left/right)
        Right Stick Y: Tilt position (up/down)
        Left Trigger: Fine control (reduce sensitivity)
        Right Trigger: Turbo mode (increase sensitivity)
        A Button: Toggle manual/auto mode
        B Button: Emergency stop
        X Button: Center pan (0°)
        Y Button: Reset tilt (90° horizontal)
        Start Button: Quick center (pan 0°, tilt 90°)
        Back Button: Toggle listening state
        LB/RB: Step pan position
        D-Pad: Fine pan/tilt adjustments
    """

    def __init__(self, config):
        """
        Initialize gamepad controller

        Args:
            config: Configuration object with gamepad settings
        """
        if not GAMEPAD_AVAILABLE:
            raise RuntimeError("Gamepad support not available - inputs library not installed")

        self.config = config
        self.gamepad_config = config.gamepad

        # State
        self.state = GamepadState()
        self.running = False
        self.connected = False
        self.input_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # Callbacks for control actions
        self.callbacks: Dict[str, Callable] = {}

        # Button press tracking (for edge detection)
        self.last_button_state: Dict[str, bool] = {}

        # Deadzone and sensitivity
        self.stick_deadzone = self.gamepad_config.stick_deadzone
        self.trigger_deadzone = self.gamepad_config.trigger_deadzone
        self.pan_sensitivity = self.gamepad_config.pan_sensitivity
        self.tilt_sensitivity = self.gamepad_config.tilt_sensitivity
        self.fine_control_multiplier = self.gamepad_config.fine_control_multiplier
        self.turbo_multiplier = self.gamepad_config.turbo_multiplier

        # Control outputs (computed from gamepad state)
        self.pan_velocity_command = 0.0
        self.tilt_angle_command = 90.0
        self.tilt_angle_delta = 0.0

        logger.info("Gamepad controller initialized")

    def register_callback(self, event_name: str, callback: Callable):
        """
        Register callback for gamepad events

        Args:
            event_name: Event name (e.g., 'toggle_mode', 'emergency_stop', 'center_pan')
            callback: Callback function to execute
        """
        self.callbacks[event_name] = callback
        logger.debug(f"Registered callback for '{event_name}'")

    def start(self):
        """Start gamepad input thread"""
        if self.running:
            return

        self.running = True
        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()
        logger.info("Gamepad input thread started")

    def stop(self):
        """Stop gamepad input thread"""
        if not self.running:
            return

        self.running = False
        if self.input_thread:
            self.input_thread.join(timeout=2.0)
        logger.info("Gamepad input thread stopped")

    def is_connected(self) -> bool:
        """
        Check if gamepad is connected

        Returns:
            True if gamepad is connected
        """
        with self.lock:
            return self.connected

    def get_pan_velocity_command(self) -> float:
        """
        Get pan velocity command from gamepad

        Returns:
            Pan velocity in degrees per second
        """
        with self.lock:
            return self.pan_velocity_command

    def get_tilt_angle_command(self) -> float:
        """
        Get tilt angle command from gamepad

        Returns:
            Tilt angle in degrees
        """
        with self.lock:
            return self.tilt_angle_command

    def get_tilt_angle_delta(self) -> float:
        """
        Get incremental tilt angle adjustment

        Returns:
            Tilt angle delta in degrees
        """
        with self.lock:
            delta = self.tilt_angle_delta
            self.tilt_angle_delta = 0.0  # Reset after reading
            return delta

    def _input_loop(self):
        """Main gamepad input loop"""
        reconnect_delay = 2.0

        while self.running:
            try:
                # Attempt to connect to gamepad
                logger.info("Attempting to connect to gamepad...")

                with self.lock:
                    self.connected = True

                logger.info("Gamepad connected")

                # Process gamepad events
                while self.running:
                    try:
                        events = get_gamepad()
                        for event in events:
                            self._process_event(event)
                    except UnpluggedError:
                        logger.warning("Gamepad disconnected")
                        with self.lock:
                            self.connected = False
                        break
                    except Exception as e:
                        logger.error(f"Error reading gamepad: {e}")
                        time.sleep(0.1)

                # Wait before reconnecting
                if self.running:
                    logger.info(f"Reconnecting in {reconnect_delay}s...")
                    time.sleep(reconnect_delay)

            except Exception as e:
                logger.error(f"Gamepad connection error: {e}")
                with self.lock:
                    self.connected = False

                if self.running:
                    time.sleep(reconnect_delay)

    def _process_event(self, event):
        """
        Process gamepad event

        Args:
            event: Input event from gamepad
        """
        event_code = event.code
        event_state = event.state

        with self.lock:
            # Process axes
            if event_code == GamepadAxis.LEFT_X.value:
                self.state.left_x = self._normalize_axis(event_state, self.stick_deadzone)
            elif event_code == GamepadAxis.LEFT_Y.value:
                self.state.left_y = self._normalize_axis(event_state, self.stick_deadzone)
            elif event_code == GamepadAxis.RIGHT_X.value:
                self.state.right_x = self._normalize_axis(event_state, self.stick_deadzone)
            elif event_code == GamepadAxis.RIGHT_Y.value:
                self.state.right_y = self._normalize_axis(event_state, self.stick_deadzone)
            elif event_code == GamepadAxis.LEFT_TRIGGER.value:
                self.state.left_trigger = self._normalize_trigger(event_state, self.trigger_deadzone)
            elif event_code == GamepadAxis.RIGHT_TRIGGER.value:
                self.state.right_trigger = self._normalize_trigger(event_state, self.trigger_deadzone)
            elif event_code == GamepadAxis.DPAD_X.value:
                self.state.dpad_x = float(event_state)
            elif event_code == GamepadAxis.DPAD_Y.value:
                self.state.dpad_y = float(event_state)

            # Process buttons (with edge detection)
            elif event_code == GamepadButton.A.value:
                self.state.button_a = bool(event_state)
                if self._button_pressed('a', event_state):
                    self._trigger_callback('toggle_mode')
            elif event_code == GamepadButton.B.value:
                self.state.button_b = bool(event_state)
                if self._button_pressed('b', event_state):
                    self._trigger_callback('emergency_stop')
            elif event_code == GamepadButton.X.value:
                self.state.button_x = bool(event_state)
                if self._button_pressed('x', event_state):
                    self._trigger_callback('center_pan')
            elif event_code == GamepadButton.Y.value:
                self.state.button_y = bool(event_state)
                if self._button_pressed('y', event_state):
                    self._trigger_callback('reset_tilt')
            elif event_code == GamepadButton.LB.value:
                self.state.button_lb = bool(event_state)
                if self._button_pressed('lb', event_state):
                    self._trigger_callback('step_pan_left')
            elif event_code == GamepadButton.RB.value:
                self.state.button_rb = bool(event_state)
                if self._button_pressed('rb', event_state):
                    self._trigger_callback('step_pan_right')
            elif event_code == GamepadButton.START.value:
                self.state.button_start = bool(event_state)
                if self._button_pressed('start', event_state):
                    self._trigger_callback('quick_center')
            elif event_code == GamepadButton.BACK.value:
                self.state.button_back = bool(event_state)
                if self._button_pressed('back', event_state):
                    self._trigger_callback('toggle_listening')

            # Update control commands based on current state
            self._update_control_commands()

    def _button_pressed(self, button_name: str, state: int) -> bool:
        """
        Detect button press (rising edge)

        Args:
            button_name: Button identifier
            state: Current button state (1 = pressed, 0 = released)

        Returns:
            True if button was just pressed
        """
        pressed = state == 1 and not self.last_button_state.get(button_name, False)
        self.last_button_state[button_name] = (state == 1)
        return pressed

    def _normalize_axis(self, value: int, deadzone: float) -> float:
        """
        Normalize axis value to [-1.0, 1.0] with deadzone

        Args:
            value: Raw axis value (typically -32768 to 32767)
            deadzone: Deadzone threshold (0.0 to 1.0)

        Returns:
            Normalized value with deadzone applied
        """
        # Normalize to [-1.0, 1.0]
        normalized = value / 32768.0

        # Apply deadzone
        if abs(normalized) < deadzone:
            return 0.0

        # Scale to account for deadzone
        if normalized > 0:
            return (normalized - deadzone) / (1.0 - deadzone)
        else:
            return (normalized + deadzone) / (1.0 - deadzone)

    def _normalize_trigger(self, value: int, deadzone: float) -> float:
        """
        Normalize trigger value to [0.0, 1.0] with deadzone

        Args:
            value: Raw trigger value (typically 0 to 255)
            deadzone: Deadzone threshold (0.0 to 1.0)

        Returns:
            Normalized trigger value
        """
        # Normalize to [0.0, 1.0]
        normalized = value / 255.0

        # Apply deadzone
        if normalized < deadzone:
            return 0.0

        # Scale to account for deadzone
        return (normalized - deadzone) / (1.0 - deadzone)

    def _update_control_commands(self):
        """Update control commands based on current gamepad state"""
        # Calculate sensitivity multiplier
        sensitivity = 1.0

        # Fine control (left trigger)
        if self.state.left_trigger > 0.1:
            sensitivity *= self.fine_control_multiplier + (1.0 - self.fine_control_multiplier) * (1.0 - self.state.left_trigger)

        # Turbo mode (right trigger)
        if self.state.right_trigger > 0.1:
            turbo_factor = 1.0 + (self.turbo_multiplier - 1.0) * self.state.right_trigger
            sensitivity *= turbo_factor

        # Pan velocity from left stick X
        pan_input = self.state.left_x
        if abs(pan_input) > 0.01:
            self.pan_velocity_command = pan_input * self.pan_sensitivity * sensitivity
        else:
            self.pan_velocity_command = 0.0

        # Tilt control from right stick Y (inverted)
        tilt_input = -self.state.right_y  # Invert Y axis
        if abs(tilt_input) > 0.01:
            # Incremental tilt adjustment
            tilt_delta = tilt_input * self.tilt_sensitivity * sensitivity * 0.02  # Scale down for smooth control
            self.tilt_angle_delta += tilt_delta

        # D-pad for fine adjustments
        if abs(self.state.dpad_x) > 0.1:
            # Fine pan adjustment
            pan_step = self.state.dpad_x * self.gamepad_config.dpad_pan_step
            self.pan_velocity_command += pan_step

        if abs(self.state.dpad_y) > 0.1:
            # Fine tilt adjustment
            tilt_step = -self.state.dpad_y * self.gamepad_config.dpad_tilt_step
            self.tilt_angle_delta += tilt_step

        # Clamp pan velocity
        max_pan_velocity = self.config.pan_motor.max_velocity_dps
        self.pan_velocity_command = max(-max_pan_velocity, min(max_pan_velocity, self.pan_velocity_command))

    def _trigger_callback(self, event_name: str):
        """
        Trigger registered callback for event

        Args:
            event_name: Event name
        """
        if event_name in self.callbacks:
            try:
                self.callbacks[event_name]()
                logger.debug(f"Triggered callback: {event_name}")
            except Exception as e:
                logger.error(f"Error in callback '{event_name}': {e}")
        else:
            logger.debug(f"No callback registered for '{event_name}'")

    def get_state_dict(self) -> Dict[str, Any]:
        """
        Get current gamepad state as dictionary

        Returns:
            Dictionary with gamepad state
        """
        with self.lock:
            return {
                'connected': self.connected,
                'axes': {
                    'left_x': self.state.left_x,
                    'left_y': self.state.left_y,
                    'right_x': self.state.right_x,
                    'right_y': self.state.right_y,
                    'left_trigger': self.state.left_trigger,
                    'right_trigger': self.state.right_trigger,
                    'dpad_x': self.state.dpad_x,
                    'dpad_y': self.state.dpad_y
                },
                'buttons': {
                    'a': self.state.button_a,
                    'b': self.state.button_b,
                    'x': self.state.button_x,
                    'y': self.state.button_y,
                    'lb': self.state.button_lb,
                    'rb': self.state.button_rb,
                    'back': self.state.button_back,
                    'start': self.state.button_start
                },
                'commands': {
                    'pan_velocity': self.pan_velocity_command,
                    'tilt_angle': self.tilt_angle_command,
                    'tilt_delta': self.tilt_angle_delta
                }
            }
