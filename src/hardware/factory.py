"""
Hardware factory for PTDTS
Creates hardware instances based on configuration (real or simulated)
"""

import logging
from typing import Tuple, Optional
from .interfaces import (
    EncoderInterface,
    MotorInterface,
    ServoInterface,
    CameraInterface,
    AudioInterface
)
from .encoder import LS7366REncoder, SimulatedEncoder
from .motor import DRV8874Motor, SimulatedMotor
from .servo import AxonServo, AxonServoClosedLoop, SimulatedServo
from .camera import Picamera2Camera, SimulatedCamera, DualCameraManager
from .acoustic import ODASAcousticDetector, SimulatedAcoustic

logger = logging.getLogger(__name__)


class HardwareFactory:
    """
    Factory for creating hardware instances
    Automatically chooses between real and simulated hardware based on config
    """

    @staticmethod
    def create_encoder(config) -> EncoderInterface:
        """
        Create encoder instance

        Args:
            config: Configuration object

        Returns:
            Encoder instance (real or simulated)
        """
        if config.simulation_mode:
            logger.info("Creating simulated encoder")
            return SimulatedEncoder(
                counts_per_360=config.pan_motor.encoder_counts_per_360
            )
        else:
            logger.info("Creating LS7366R encoder")
            return LS7366REncoder(
                spi_bus=config.pan_motor.encoder_spi_bus,
                spi_device=config.pan_motor.encoder_spi_device,
                counts_per_360=config.pan_motor.encoder_counts_per_360,
                mode=config.pan_motor.encoder_mode
            )

    @staticmethod
    def create_motor(config, encoder: Optional[EncoderInterface] = None) -> MotorInterface:
        """
        Create motor instance

        Args:
            config: Configuration object
            encoder: Encoder instance (for simulated motor)

        Returns:
            Motor instance (real or simulated)
        """
        if config.simulation_mode:
            logger.info("Creating simulated motor")
            return SimulatedMotor(
                max_pwm=config.pan_motor.max_pwm,
                encoder=encoder
            )
        else:
            logger.info("Creating DRV8874 motor")
            return DRV8874Motor(
                gpio_in1=config.pan_motor.gpio_in1,
                gpio_in2=config.pan_motor.gpio_in2,
                max_pwm=config.pan_motor.max_pwm
            )

    @staticmethod
    def create_servo(config) -> ServoInterface:
        """
        Create servo instance

        Args:
            config: Configuration object

        Returns:
            Servo instance (real or simulated)
        """
        if config.simulation_mode:
            logger.info("Creating simulated servo")
            return SimulatedServo(
                min_angle=config.tilt_servo.min_angle,
                max_angle=config.tilt_servo.max_angle,
                slew_rate_dps=config.tilt_servo.slew_rate_dps
            )
        else:
            # Choose between open-loop and closed-loop
            if config.tilt_servo.closed_loop:
                logger.info("Creating Axon servo (closed-loop)")
                return AxonServoClosedLoop(
                    gpio_pwm=config.tilt_servo.gpio_pwm,
                    pwm_frequency=config.tilt_servo.pwm_frequency,
                    pwm_min=config.tilt_servo.pwm_min,
                    pwm_max=config.tilt_servo.pwm_max,
                    min_angle=config.tilt_servo.min_angle,
                    max_angle=config.tilt_servo.max_angle,
                    ads1115_address=config.tilt_servo.ads1115_address,
                    ads1115_channel=config.tilt_servo.ads1115_channel,
                    voltage_min=config.tilt_servo.voltage_min,
                    voltage_max=config.tilt_servo.voltage_max,
                    kp=config.tilt_servo.position_pid['kp'],
                    ki=config.tilt_servo.position_pid['ki'],
                    kd=config.tilt_servo.position_pid['kd'],
                    deadzone_degrees=config.tilt_servo.position_pid['deadzone_degrees']
                )
            else:
                logger.info("Creating Axon servo (open-loop)")
                return AxonServo(
                    gpio_pwm=config.tilt_servo.gpio_pwm,
                    pwm_frequency=config.tilt_servo.pwm_frequency,
                    pwm_min=config.tilt_servo.pwm_min,
                    pwm_max=config.tilt_servo.pwm_max,
                    min_angle=config.tilt_servo.min_angle,
                    max_angle=config.tilt_servo.max_angle,
                    slew_rate_dps=config.tilt_servo.slew_rate_dps,
                    acceleration_profile=getattr(config.tilt_servo, 'acceleration_profile', 's_curve')
                )

    @staticmethod
    def create_cameras(config) -> DualCameraManager:
        """
        Create dual camera system

        Args:
            config: Configuration object

        Returns:
            DualCameraManager instance
        """
        if config.simulation_mode:
            logger.info("Creating simulated cameras")
            hq_camera = SimulatedCamera(
                camera_index=config.cameras.hq.index,
                resolution=config.cameras.hq.resolution,
                framerate=config.cameras.hq.framerate
            )
            # Create simulated GS camera (even if disabled, for compatibility)
            gs_camera = SimulatedCamera(
                camera_index=config.cameras.gs.index,
                resolution=config.cameras.gs.resolution,
                framerate=config.cameras.gs.framerate
            )
        else:
            logger.info("Creating Picamera2 cameras")
            hq_camera = Picamera2Camera(
                camera_index=config.cameras.hq.index,
                resolution=config.cameras.hq.resolution,
                framerate=config.cameras.hq.framerate,
                format=config.cameras.hq.format,
                controls=config.cameras.hq.controls
            )

            # Only create real GS camera if enabled, otherwise use simulated
            if config.cameras.gs.enabled:
                logger.info("Creating GS camera (enabled)")
                gs_camera = Picamera2Camera(
                    camera_index=config.cameras.gs.index,
                    resolution=config.cameras.gs.resolution,
                    framerate=config.cameras.gs.framerate,
                    format=config.cameras.gs.format,
                    controls=config.cameras.gs.controls
                )
            else:
                logger.info("GS camera disabled - using simulated placeholder")
                gs_camera = SimulatedCamera(
                    camera_index=config.cameras.gs.index,
                    resolution=config.cameras.gs.resolution,
                    framerate=config.cameras.gs.framerate
                )

        return DualCameraManager(
            hq_camera=hq_camera,
            gs_camera=gs_camera,
            settling_delay_ms=config.cameras.switching['settling_delay_ms']
        )

    @staticmethod
    def create_acoustic_detector(config) -> AudioInterface:
        """
        Create acoustic detector instance

        Args:
            config: Configuration object

        Returns:
            AudioInterface instance (real or simulated)
        """
        if not config.acoustic.enabled:
            logger.info("Acoustic detection disabled")
            # Return dummy detector that does nothing
            return SimulatedAcoustic(detection_rate_hz=0.0)

        if config.simulation_mode:
            logger.info("Creating simulated acoustic detector")
            return SimulatedAcoustic(
                energy_threshold=config.acoustic.energy_threshold,
                detection_rate_hz=0.1  # One detection every ~10 seconds
            )
        else:
            logger.info("Creating ODAS acoustic detector")
            return ODASAcousticDetector(
                odas_port=config.acoustic.odas_port,
                mic_radius_mm=config.acoustic.mic_radius_mm,
                mic_count=config.acoustic.mic_count,
                energy_threshold=config.acoustic.energy_threshold,
                frequency_min=config.acoustic.frequency_min,
                frequency_max=config.acoustic.frequency_max,
                odas_config_path=config.acoustic.odas_config_path
            )
