"""
Calibration Data Storage for PTDTS
Saves and loads calibration results to/from JSON file
"""

import json
import logging
from pathlib import Path
from typing import Dict, Any, Optional
from datetime import datetime

logger = logging.getLogger(__name__)


class CalibrationData:
    """
    Manages calibration data storage and retrieval
    Stores all calibration results in a JSON file
    """

    def __init__(self, filepath: str = "calibration/calibration.json"):
        """
        Initialize calibration data manager

        Args:
            filepath: Path to calibration JSON file
        """
        self.filepath = Path(filepath)
        self.data = self._load_or_create()

    def _load_or_create(self) -> Dict[str, Any]:
        """
        Load existing calibration data or create new

        Returns:
            Calibration data dictionary
        """
        if self.filepath.exists():
            try:
                with open(self.filepath, 'r') as f:
                    data = json.load(f)
                logger.info(f"Loaded calibration data from {self.filepath}")
                return data
            except Exception as e:
                logger.error(f"Failed to load calibration data: {e}")
                return self._create_default()
        else:
            logger.info("No existing calibration data, creating default")
            return self._create_default()

    def _create_default(self) -> Dict[str, Any]:
        """
        Create default calibration data structure

        Returns:
            Default calibration dictionary
        """
        return {
            "version": "1.0",
            "created": datetime.now().isoformat(),
            "last_updated": datetime.now().isoformat(),
            "motor": {},
            "camera_hq": {},
            "camera_gs": {},
            "servo": {},
            "acoustic": {}
        }

    def save(self):
        """Save calibration data to file"""
        try:
            # Update timestamp
            self.data["last_updated"] = datetime.now().isoformat()

            # Create directory if needed
            self.filepath.parent.mkdir(parents=True, exist_ok=True)

            # Write to file with indentation
            with open(self.filepath, 'w') as f:
                json.dump(self.data, f, indent=2)

            logger.info(f"Saved calibration data to {self.filepath}")

        except Exception as e:
            logger.error(f"Failed to save calibration data: {e}")
            raise

    def get(self, section: str) -> Dict[str, Any]:
        """
        Get calibration data for a section

        Args:
            section: Section name (motor, camera_hq, camera_gs, servo, acoustic)

        Returns:
            Section data dictionary
        """
        return self.data.get(section, {})

    def set(self, section: str, data: Dict[str, Any]):
        """
        Set calibration data for a section

        Args:
            section: Section name
            data: Data to store
        """
        self.data[section] = data
        self.data["last_updated"] = datetime.now().isoformat()

    def update(self, section: str, **kwargs):
        """
        Update specific values in a section

        Args:
            section: Section name
            **kwargs: Key-value pairs to update
        """
        if section not in self.data:
            self.data[section] = {}

        self.data[section].update(kwargs)
        self.data["last_updated"] = datetime.now().isoformat()

    # Motor calibration

    def set_motor_calibration(
        self,
        encoder_counts_per_360: float,
        measured_counts: int,
        rotations: int,
        direction: str = "CCW",
        notes: str = ""
    ):
        """
        Save motor encoder calibration

        Args:
            encoder_counts_per_360: Measured counts per 360° rotation
            measured_counts: Total counts measured
            rotations: Number of full rotations
            direction: Rotation direction (CW or CCW)
            notes: Additional notes
        """
        self.data["motor"] = {
            "encoder_counts_per_360": encoder_counts_per_360,
            "measured_counts": measured_counts,
            "rotations": rotations,
            "direction": direction,
            "calibrated_at": datetime.now().isoformat(),
            "notes": notes
        }

    def get_motor_calibration(self) -> Optional[float]:
        """
        Get motor encoder counts per 360°

        Returns:
            Counts per 360° or None if not calibrated
        """
        motor_data = self.data.get("motor", {})
        return motor_data.get("encoder_counts_per_360")

    # Camera FOV calibration

    def set_camera_fov(
        self,
        camera: str,
        horizontal_fov: float,
        vertical_fov: float,
        object_width_mm: float,
        distance_mm: float,
        resolution: tuple,
        notes: str = ""
    ):
        """
        Save camera FOV calibration

        Args:
            camera: Camera name (hq or gs)
            horizontal_fov: Horizontal FOV in degrees
            vertical_fov: Vertical FOV in degrees
            object_width_mm: Calibration object width
            distance_mm: Calibration distance
            resolution: Camera resolution (width, height)
            notes: Additional notes
        """
        section = f"camera_{camera}"
        self.data[section] = {
            "horizontal_fov_deg": horizontal_fov,
            "vertical_fov_deg": vertical_fov,
            "object_width_mm": object_width_mm,
            "distance_mm": distance_mm,
            "resolution": list(resolution),
            "calibrated_at": datetime.now().isoformat(),
            "notes": notes
        }

    def get_camera_fov(self, camera: str) -> Optional[tuple]:
        """
        Get camera FOV

        Args:
            camera: Camera name (hq or gs)

        Returns:
            (horizontal_fov, vertical_fov) or None if not calibrated
        """
        section = f"camera_{camera}"
        camera_data = self.data.get(section, {})

        h_fov = camera_data.get("horizontal_fov_deg")
        v_fov = camera_data.get("vertical_fov_deg")

        if h_fov is not None and v_fov is not None:
            return (h_fov, v_fov)
        return None

    # Servo calibration

    def set_servo_calibration(
        self,
        voltage_min: float,
        voltage_max: float,
        voltage_0deg: float,
        voltage_180deg: float,
        linearity_error: float,
        notes: str = ""
    ):
        """
        Save servo feedback calibration

        Args:
            voltage_min: Minimum voltage measured
            voltage_max: Maximum voltage measured
            voltage_0deg: Voltage at 0°
            voltage_180deg: Voltage at 180°
            linearity_error: Maximum linearity error percentage
            notes: Additional notes
        """
        self.data["servo"] = {
            "voltage_min": voltage_min,
            "voltage_max": voltage_max,
            "voltage_0deg": voltage_0deg,
            "voltage_180deg": voltage_180deg,
            "linearity_error_percent": linearity_error,
            "calibrated_at": datetime.now().isoformat(),
            "notes": notes
        }

    def get_servo_calibration(self) -> Optional[tuple]:
        """
        Get servo voltage range

        Returns:
            (voltage_min, voltage_max) or None if not calibrated
        """
        servo_data = self.data.get("servo", {})

        v_min = servo_data.get("voltage_min")
        v_max = servo_data.get("voltage_max")

        if v_min is not None and v_max is not None:
            return (v_min, v_max)
        return None

    # Acoustic calibration

    def set_acoustic_calibration(
        self,
        offset_deg: float,
        method: str = "single_position",
        std_deviation: Optional[float] = None,
        num_samples: int = 0,
        notes: str = ""
    ):
        """
        Save acoustic alignment calibration

        Args:
            offset_deg: Acoustic offset in degrees
            method: Calibration method (single_position, multi_position)
            std_deviation: Standard deviation of measurements (if multi-position)
            num_samples: Number of samples used
            notes: Additional notes
        """
        self.data["acoustic"] = {
            "offset_deg": offset_deg,
            "method": method,
            "std_deviation": std_deviation,
            "num_samples": num_samples,
            "calibrated_at": datetime.now().isoformat(),
            "notes": notes
        }

    def get_acoustic_calibration(self) -> Optional[float]:
        """
        Get acoustic offset

        Returns:
            Offset in degrees or None if not calibrated
        """
        acoustic_data = self.data.get("acoustic", {})
        return acoustic_data.get("offset_deg")

    # Summary and export

    def get_summary(self) -> str:
        """
        Get human-readable summary of calibration data

        Returns:
            Formatted summary string
        """
        lines = []
        lines.append("=" * 70)
        lines.append("PTDTS CALIBRATION DATA SUMMARY")
        lines.append("=" * 70)
        lines.append(f"Version: {self.data.get('version', 'unknown')}")
        lines.append(f"Last Updated: {self.data.get('last_updated', 'unknown')}")
        lines.append("")

        # Motor
        motor = self.data.get("motor", {})
        if motor:
            lines.append("MOTOR ENCODER:")
            lines.append(f"  Counts per 360°: {motor.get('encoder_counts_per_360', 'N/A')}")
            lines.append(f"  Calibrated: {motor.get('calibrated_at', 'N/A')}")
        else:
            lines.append("MOTOR ENCODER: Not calibrated")
        lines.append("")

        # Cameras
        for cam in ["hq", "gs"]:
            section = f"camera_{cam}"
            camera = self.data.get(section, {})
            if camera:
                lines.append(f"{cam.upper()} CAMERA:")
                lines.append(f"  Horizontal FOV: {camera.get('horizontal_fov_deg', 'N/A')}°")
                lines.append(f"  Vertical FOV: {camera.get('vertical_fov_deg', 'N/A')}°")
                lines.append(f"  Calibrated: {camera.get('calibrated_at', 'N/A')}")
            else:
                lines.append(f"{cam.upper()} CAMERA: Not calibrated")
            lines.append("")

        # Servo
        servo = self.data.get("servo", {})
        if servo:
            lines.append("SERVO FEEDBACK:")
            lines.append(f"  Voltage range: {servo.get('voltage_min', 'N/A')} - {servo.get('voltage_max', 'N/A')} V")
            lines.append(f"  Linearity error: {servo.get('linearity_error_percent', 'N/A')}%")
            lines.append(f"  Calibrated: {servo.get('calibrated_at', 'N/A')}")
        else:
            lines.append("SERVO FEEDBACK: Not calibrated")
        lines.append("")

        # Acoustic
        acoustic = self.data.get("acoustic", {})
        if acoustic:
            lines.append("ACOUSTIC ALIGNMENT:")
            lines.append(f"  Offset: {acoustic.get('offset_deg', 'N/A')}°")
            lines.append(f"  Method: {acoustic.get('method', 'N/A')}")
            lines.append(f"  Calibrated: {acoustic.get('calibrated_at', 'N/A')}")
        else:
            lines.append("ACOUSTIC ALIGNMENT: Not calibrated")

        lines.append("=" * 70)

        return "\n".join(lines)

    def export_to_config_format(self) -> Dict[str, Any]:
        """
        Export calibration data in config.yaml format

        Returns:
            Dictionary with config.yaml compatible structure
        """
        config_data = {}

        # Motor
        motor_counts = self.get_motor_calibration()
        if motor_counts:
            config_data["encoder_counts_per_360"] = motor_counts

        # Cameras
        hq_fov = self.get_camera_fov("hq")
        if hq_fov:
            config_data["camera_hq_fov"] = {
                "horizontal": hq_fov[0],
                "vertical": hq_fov[1]
            }

        gs_fov = self.get_camera_fov("gs")
        if gs_fov:
            config_data["camera_gs_fov"] = {
                "horizontal": gs_fov[0],
                "vertical": gs_fov[1]
            }

        # Servo
        servo_range = self.get_servo_calibration()
        if servo_range:
            config_data["servo_feedback"] = {
                "voltage_min": servo_range[0],
                "voltage_max": servo_range[1]
            }

        # Acoustic
        acoustic_offset = self.get_acoustic_calibration()
        if acoustic_offset:
            config_data["acoustic_offset"] = acoustic_offset

        return config_data

    def print_config_updates(self):
        """Print config.yaml update instructions"""
        config_data = self.export_to_config_format()

        print("\n" + "=" * 70)
        print("CONFIG.YAML UPDATES")
        print("=" * 70)
        print("\nAdd or update these values in config/config.yaml:\n")

        for key, value in config_data.items():
            if isinstance(value, dict):
                print(f"{key}:")
                for sub_key, sub_value in value.items():
                    print(f"  {sub_key}: {sub_value}")
            else:
                print(f"{key}: {value}")

        print("\n" + "=" * 70)


# Convenience function for command-line use
def view_calibration_data(filepath: str = "calibration/calibration.json"):
    """
    View calibration data summary from command line

    Args:
        filepath: Path to calibration file
    """
    cal_data = CalibrationData(filepath)
    print(cal_data.get_summary())
    print()
    cal_data.print_config_updates()


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        filepath = "calibration/calibration.json"

    view_calibration_data(filepath)
