"""
Configuration loader for PTDTS
Loads and validates configuration from YAML file
"""

import os
import yaml
import logging
from pathlib import Path
from typing import Any, Dict, Optional
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class PanMotorConfig:
    """Pan motor configuration"""
    encoder_counts_per_360: int = -2139  # For non-quadrature mode (x4 mode would be -8556)
    gear_ratio: float = 135.0
    gpio_in1: int = 27
    gpio_in2: int = 22
    # Note: PMODE/SLEEP hardwired to 3.3V - no software control needed
    encoder_spi_bus: int = 0
    encoder_spi_device: int = 0
    encoder_mode: int = 0  # Non-quadrature (quadrature modes freeze LS7366R)
    max_velocity_dps: float = 180.0
    max_acceleration_dps2: float = 360.0
    max_pwm: float = 0.70
    min_pwm: float = 0.30
    startup_boost: float = 0.35
    control_rate_hz: int = 50
    position_pid: Dict[str, float] = field(default_factory=lambda: {'kp': 0.8, 'ki': 0.05, 'kd': 0.1, 'deadzone_degrees': 2.0})
    velocity_pid: Dict[str, float] = field(default_factory=lambda: {'kp': 0.015, 'ki': 0.002, 'kd': 0.001})
    panning: Dict[str, float] = field(default_factory=lambda: {'approach_distance_degrees': 30.0, 'arrival_threshold_degrees': 5.0, 'decel_factor': 0.3})


@dataclass
class TiltServoConfig:
    """Tilt servo configuration"""
    gpio_pwm: int = 18  # Hardware PWM pin (required for Pi 5)
    pwm_frequency: int = 333
    pwm_min: int = 500
    pwm_max: int = 2500
    closed_loop: bool = False
    ads1115_address: int = 0x48
    ads1115_channel: int = 0
    voltage_min: float = 0.5
    voltage_max: float = 4.5
    min_angle: int = 0
    max_angle: int = 90
    default_angle: int = 45
    position_pid: Dict[str, float] = field(default_factory=lambda: {'kp': 2.0, 'ki': 0.1, 'kd': 0.3, 'deadzone_degrees': 1.0})
    slew_rate_dps: float = 15.0
    acceleration_profile: str = 's_curve'


@dataclass
class CameraConfig:
    """Camera configuration"""
    enabled: bool = True
    index: int = 0
    resolution: tuple = (640, 480)
    framerate: int = 30
    format: str = "RGB888"
    fov_h: float = 60.0
    fov_v: float = 40.0
    detection_confidence: Optional[float] = None
    tracking_confidence: Optional[float] = None
    controls: Optional[Dict[str, Any]] = None  # Camera controls (exposure, gain, etc.)


@dataclass
class CamerasConfig:
    """Dual camera configuration"""
    hq: CameraConfig = None
    gs: CameraConfig = None
    switching: Dict[str, Any] = field(default_factory=lambda: {'settling_delay_ms': 500, 'gs_center_threshold_percent': 15})


@dataclass
class AcousticConfig:
    """Acoustic detection configuration"""
    enabled: bool = True
    mic_radius_mm: float = 46.5
    mic_count: int = 4
    odas_port: int = 9000
    odas_config_path: str = "/opt/odas/config/ptdts_respeaker.cfg"
    energy_threshold: float = 0.6
    frequency_min: float = 100.0
    frequency_max: float = 500.0
    signature_enabled: bool = False


@dataclass
class YOLOConfig:
    """YOLO detection configuration"""
    model_path: str = "models/yolov11n-UAV-finetune_ncnn_model"
    device: str = "cpu"
    detection_confidence: float = 0.4
    tracking_confidence: float = 0.3
    iou_threshold: float = 0.45
    classes: Optional[list] = None
    max_det: int = 10
    verbose: bool = False


@dataclass
class TrackingConfig:
    """Tracking configuration"""
    position_damping: float = 0.4  # Damping factor for position-based tracking (0.3-0.6)
    use_direct_pwm: bool = True  # Use direct PWM control (true) or velocity PID (false)
    pixel_to_velocity_gain: float = 0.4
    deadzone_pixels: int = 40  # Increased from 20 to prevent jitter
    max_velocity_dps: float = 180.0
    velocity_deadband: float = 30.0
    loss_timeout_seconds: float = 10.0  # Increased from 2.0 to 10.0
    reacquisition_timeout_seconds: float = 3.0  # Continue tracking in last direction
    continue_last_direction: bool = True  # Keep moving when target temporarily lost


@dataclass
class StateMachineConfig:
    """State machine configuration"""
    initial_state: str = "LISTENING"
    timeouts: Dict[str, float] = field(default_factory=lambda: {'panning': 10.0, 'detecting': 5.0, 'tracking': 300.0})
    detection: Dict[str, Any] = field(default_factory=lambda: {'min_acoustic_confidence': 0.6, 'min_visual_confidence': 0.4, 'visual_confirmation_frames': 3})


@dataclass
class WebConfig:
    """Web interface configuration"""
    host: str = "0.0.0.0"
    port: int = 5000
    debug: bool = False
    video: Dict[str, int] = field(default_factory=lambda: {'quality': 85, 'max_fps': 30})
    telemetry_rate_hz: int = 10
    manual: Dict[str, Any] = field(default_factory=lambda: {'pan_velocity_dps': 30.0, 'tilt_velocity_dps': 20.0, 'keyboard_enabled': True, 'gamepad_enabled': True})


@dataclass
class LoggingConfig:
    """Logging configuration"""
    console_level: str = "INFO"
    file_enabled: bool = True
    file_level: str = "DEBUG"
    file_path: str = "logs/ptdts.log"
    file_max_bytes: int = 10485760
    file_backup_count: int = 5
    telemetry_enabled: bool = True
    telemetry_path: str = "logs/telemetry.csv"
    telemetry_rate_hz: int = 10
    detections_enabled: bool = True
    detections_path: str = "logs/detections.csv"


@dataclass
class CoordinatesConfig:
    """Coordinate system configuration"""
    origin_lat: float = 0.0
    origin_lon: float = 0.0
    origin_alt: float = 0.0
    azimuth_zero: str = "north"
    elevation_zero: str = "horizontal"
    default_range_meters: float = 50.0


@dataclass
class CalibrationConfig:
    """Calibration data"""
    encoder_offset: int = 0
    servo_offset: int = 0
    camera_offsets: Dict[str, list] = field(default_factory=lambda: {'hq': [0.0, 0.0], 'gs': [0.0, 0.0]})
    acoustic_offset: float = 0.0


@dataclass
class PerformanceConfig:
    """Performance monitoring configuration"""
    monitoring_enabled: bool = True
    stats_interval_seconds: int = 10
    main_loop_rate_hz: int = 15  # Main detection/state loop rate (lower than motor control for performance)
    warn_loop_overrun: bool = True
    warn_detection_latency_ms: int = 100
    warn_cpu_temp_celsius: int = 75


@dataclass
class GamepadConfig:
    """Gamepad controller configuration"""
    enabled: bool = False  # Default disabled
    stick_deadzone: float = 0.15
    trigger_deadzone: float = 0.1
    pan_sensitivity: float = 120.0
    tilt_sensitivity: float = 60.0
    fine_control_multiplier: float = 0.3
    turbo_multiplier: float = 2.0
    dpad_pan_step: float = 5.0
    dpad_tilt_step: float = 2.0
    quick_center_pan: float = 0.0
    quick_center_tilt: float = 90.0
    pan_step_angle: float = 15.0


@dataclass
class Config:
    """Main configuration class"""
    simulation_mode: bool = False
    pan_motor: PanMotorConfig = field(default_factory=PanMotorConfig)
    tilt_servo: TiltServoConfig = field(default_factory=TiltServoConfig)
    cameras: CamerasConfig = field(default_factory=CamerasConfig)
    acoustic: AcousticConfig = field(default_factory=AcousticConfig)
    yolo: YOLOConfig = field(default_factory=YOLOConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    state_machine: StateMachineConfig = field(default_factory=StateMachineConfig)
    web: WebConfig = field(default_factory=WebConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    coordinates: CoordinatesConfig = field(default_factory=CoordinatesConfig)
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    performance: PerformanceConfig = field(default_factory=PerformanceConfig)
    gamepad: GamepadConfig = field(default_factory=GamepadConfig)


def _dict_to_dataclass(cls, data: Dict):
    """Convert dictionary to dataclass recursively"""
    if not isinstance(data, dict):
        return data

    field_types = {f.name: f.type for f in cls.__dataclass_fields__.values()}
    kwargs = {}

    for key, value in data.items():
        if key not in field_types:
            logger.warning(f"Unknown config key: {key}")
            continue

        field_type = field_types[key]

        # Handle nested dataclasses
        if hasattr(field_type, '__dataclass_fields__'):
            kwargs[key] = _dict_to_dataclass(field_type, value)
        # Handle lists of tuples (for camera resolution)
        elif isinstance(value, list) and field_type == tuple:
            kwargs[key] = tuple(value)
        else:
            kwargs[key] = value

    return cls(**kwargs)


def load_config(config_path: Optional[str] = None) -> Config:
    """
    Load configuration from YAML file

    Args:
        config_path: Path to configuration file. If None, searches for config.yaml
                     in standard locations.

    Returns:
        Config object with loaded settings

    Raises:
        FileNotFoundError: If configuration file not found
        yaml.YAMLError: If configuration file is invalid
    """
    # Search for config file
    if config_path is None:
        search_paths = [
            Path('config/config.yaml'),
            Path('config.yaml'),
            Path('/etc/ptdts/config.yaml'),
        ]

        for path in search_paths:
            if path.exists():
                config_path = str(path)
                break
        else:
            logger.warning("No config file found, using defaults")
            return Config()

    logger.info(f"Loading configuration from: {config_path}")

    # Load YAML
    with open(config_path, 'r') as f:
        config_data = yaml.safe_load(f)

    if config_data is None:
        logger.warning("Empty config file, using defaults")
        return Config()

    # Convert to Config object
    config = Config()

    # Top-level simple fields
    if 'simulation_mode' in config_data:
        config.simulation_mode = config_data['simulation_mode']

    # Nested configurations
    if 'pan_motor' in config_data:
        config.pan_motor = _dict_to_dataclass(PanMotorConfig, config_data['pan_motor'])

    if 'tilt_servo' in config_data:
        config.tilt_servo = _dict_to_dataclass(TiltServoConfig, config_data['tilt_servo'])

    if 'cameras' in config_data:
        cameras_data = config_data['cameras']
        config.cameras = CamerasConfig()
        if 'hq' in cameras_data:
            config.cameras.hq = _dict_to_dataclass(CameraConfig, cameras_data['hq'])
        if 'gs' in cameras_data:
            config.cameras.gs = _dict_to_dataclass(CameraConfig, cameras_data['gs'])
        if 'switching' in cameras_data:
            config.cameras.switching = cameras_data['switching']

    if 'acoustic' in config_data:
        config.acoustic = _dict_to_dataclass(AcousticConfig, config_data['acoustic'])

    if 'yolo' in config_data:
        config.yolo = _dict_to_dataclass(YOLOConfig, config_data['yolo'])

    if 'tracking' in config_data:
        config.tracking = _dict_to_dataclass(TrackingConfig, config_data['tracking'])

    if 'state_machine' in config_data:
        config.state_machine = _dict_to_dataclass(StateMachineConfig, config_data['state_machine'])

    if 'web' in config_data:
        config.web = _dict_to_dataclass(WebConfig, config_data['web'])

    if 'logging' in config_data:
        config.logging = _dict_to_dataclass(LoggingConfig, config_data['logging'])

    if 'coordinates' in config_data:
        config.coordinates = _dict_to_dataclass(CoordinatesConfig, config_data['coordinates'])

    if 'calibration' in config_data:
        config.calibration = _dict_to_dataclass(CalibrationConfig, config_data['calibration'])

    if 'performance' in config_data:
        config.performance = _dict_to_dataclass(PerformanceConfig, config_data['performance'])

    if 'gamepad' in config_data:
        config.gamepad = _dict_to_dataclass(GamepadConfig, config_data['gamepad'])

    logger.info("Configuration loaded successfully")
    return config


def save_config(config: Config, config_path: str):
    """
    Save configuration to YAML file

    Args:
        config: Config object to save
        config_path: Path to save configuration file
    """
    # Convert dataclass to dictionary (recursively)
    from dataclasses import asdict

    config_dict = asdict(config)

    # Ensure directory exists
    os.makedirs(os.path.dirname(config_path), exist_ok=True)

    # Save YAML
    with open(config_path, 'w') as f:
        yaml.dump(config_dict, f, default_flow_style=False, sort_keys=False)

    logger.info(f"Configuration saved to: {config_path}")
