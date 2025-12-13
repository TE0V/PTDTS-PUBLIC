"""
State Manager for PTDTS Runtime State Persistence

Handles saving and loading of system runtime state including:
- Encoder position (absolute zero reference)
- Pan motor controller state
- Tilt servo controller state
- State machine state
- Tracking targets and history

This allows the system to restore its position and state after reboot
without requiring recalibration or re-zeroing.
"""

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)


class StateManager:
    """Manages persistence of PTDTS runtime state"""

    DEFAULT_STATE_FILE = "state/runtime_state.json"

    def __init__(self, state_file: Optional[str] = None):
        """
        Initialize the StateManager.

        Args:
            state_file: Path to state file (default: state/runtime_state.json)
        """
        if state_file is None:
            # Use default path relative to project root
            project_root = Path(__file__).parent.parent.parent
            self.state_file = project_root / self.DEFAULT_STATE_FILE
        else:
            self.state_file = Path(state_file)

        # Ensure state directory exists
        self.state_file.parent.mkdir(parents=True, exist_ok=True)

        self.state_data: Dict[str, Any] = {}
        self._load()

    def _load(self) -> None:
        """Load state from disk"""
        if self.state_file.exists():
            try:
                with open(self.state_file, 'r') as f:
                    self.state_data = json.load(f)
                logger.info(f"Loaded runtime state from {self.state_file}")
            except Exception as e:
                logger.error(f"Failed to load state file: {e}")
                self.state_data = self._create_empty_state()
        else:
            logger.info(f"No existing state file found at {self.state_file}")
            self.state_data = self._create_empty_state()

    def _create_empty_state(self) -> Dict[str, Any]:
        """Create empty state structure"""
        return {
            "version": "1.0",
            "created": datetime.now().isoformat(),
            "last_updated": datetime.now().isoformat(),
            "encoder": {},
            "pan_controller": {},
            "tilt_controller": {},
            "state_machine": {},
        }

    def save(self) -> None:
        """Save current state to disk"""
        try:
            self.state_data["last_updated"] = datetime.now().isoformat()

            # Write to temporary file first, then rename (atomic operation)
            temp_file = self.state_file.with_suffix('.json.tmp')
            with open(temp_file, 'w') as f:
                json.dump(self.state_data, f, indent=2)

            # Atomic rename
            temp_file.replace(self.state_file)
            logger.info(f"Saved runtime state to {self.state_file}")
        except Exception as e:
            logger.error(f"Failed to save state file: {e}")

    # Encoder State
    def set_encoder_state(self, count: int, angle: float, velocity: float = 0.0) -> None:
        """
        Save encoder state (absolute zero reference).

        Args:
            count: Raw encoder count
            angle: Current angle in degrees (0-360)
            velocity: Current velocity in degrees/second
        """
        self.state_data["encoder"] = {
            "count": count,
            "angle": angle,
            "velocity": velocity,
            "timestamp": datetime.now().isoformat()
        }
        logger.debug(f"Set encoder state: count={count}, angle={angle:.2f}°, velocity={velocity:.2f} dps")

    def get_encoder_state(self) -> Optional[Dict[str, Any]]:
        """
        Get saved encoder state.

        Returns:
            Dictionary with count, angle, velocity, timestamp or None if not set
        """
        return self.state_data.get("encoder") if self.state_data.get("encoder") else None

    # Pan Controller State
    def set_pan_controller_state(self, mode: str, target_position: Optional[float] = None,
                                  target_velocity: Optional[float] = None,
                                  pid_integral: float = 0.0, pid_last_error: float = 0.0) -> None:
        """
        Save pan controller state.

        Args:
            mode: Control mode (IDLE, POSITION, VELOCITY, MANUAL)
            target_position: Target position in degrees (if applicable)
            target_velocity: Target velocity in degrees/second (if applicable)
            pid_integral: PID integral term
            pid_last_error: PID last error term
        """
        self.state_data["pan_controller"] = {
            "mode": mode,
            "target_position": target_position,
            "target_velocity": target_velocity,
            "pid_integral": pid_integral,
            "pid_last_error": pid_last_error,
            "timestamp": datetime.now().isoformat()
        }
        logger.debug(f"Set pan controller state: mode={mode}, target_pos={target_position}")

    def get_pan_controller_state(self) -> Optional[Dict[str, Any]]:
        """
        Get saved pan controller state.

        Returns:
            Dictionary with mode, targets, PID state or None if not set
        """
        return self.state_data.get("pan_controller") if self.state_data.get("pan_controller") else None

    # Tilt Controller State
    def set_tilt_controller_state(self, current_angle: float, target_angle: float,
                                   pid_integral: float = 0.0, pid_last_error: float = 0.0) -> None:
        """
        Save tilt controller state.

        Args:
            current_angle: Current servo angle in degrees (0-180)
            target_angle: Target servo angle in degrees (0-180)
            pid_integral: PID integral term (for closed-loop servo)
            pid_last_error: PID last error term (for closed-loop servo)
        """
        self.state_data["tilt_controller"] = {
            "current_angle": current_angle,
            "target_angle": target_angle,
            "pid_integral": pid_integral,
            "pid_last_error": pid_last_error,
            "timestamp": datetime.now().isoformat()
        }
        logger.debug(f"Set tilt controller state: current={current_angle:.2f}°, target={target_angle:.2f}°")

    def get_tilt_controller_state(self) -> Optional[Dict[str, Any]]:
        """
        Get saved tilt controller state.

        Returns:
            Dictionary with current_angle, target_angle, PID state or None if not set
        """
        return self.state_data.get("tilt_controller") if self.state_data.get("tilt_controller") else None

    # State Machine State
    def set_state_machine_state(self, current_state: str,
                                 current_target: Optional[Dict[str, Any]] = None,
                                 pending_acoustic: Optional[Dict[str, Any]] = None,
                                 tracking_lost_count: int = 0) -> None:
        """
        Save state machine state.

        Args:
            current_state: Current system state (LISTENING, PANNING, DETECTING, TRACKING, MANUAL, ERROR)
            current_target: Current tracking target dict (center_x, center_y, confidence, class_name, timestamp)
            pending_acoustic: Pending acoustic detection dict (azimuth, elevation, energy, timestamp)
            tracking_lost_count: Number of consecutive frames target was lost
        """
        self.state_data["state_machine"] = {
            "current_state": current_state,
            "current_target": current_target,
            "pending_acoustic": pending_acoustic,
            "tracking_lost_count": tracking_lost_count,
            "timestamp": datetime.now().isoformat()
        }
        logger.debug(f"Set state machine state: {current_state}")

    def get_state_machine_state(self) -> Optional[Dict[str, Any]]:
        """
        Get saved state machine state.

        Returns:
            Dictionary with current_state, targets, counters or None if not set
        """
        return self.state_data.get("state_machine") if self.state_data.get("state_machine") else None

    def clear_all_state(self) -> None:
        """Clear all saved state (reset to empty)"""
        self.state_data = self._create_empty_state()
        self.save()
        logger.info("Cleared all runtime state")

    def get_all_state(self) -> Dict[str, Any]:
        """Get all saved state data"""
        return self.state_data.copy()
