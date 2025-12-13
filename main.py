#!/usr/bin/env python3
"""
PTDTS - Pan Tilt Drone Detection and Tracking System
Main application entry point
"""

import sys
import time
import signal
import logging
import argparse
import threading
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.utils.config_loader import load_config
from src.utils.logger import setup_logging
from src.utils.timing_monitor import LoopTimingMonitor
from src.utils.system_monitor import SystemMonitor
from src.hardware.factory import HardwareFactory
from src.control.pan_controller import PanController
from src.control.tilt_controller import TiltController
from src.control.gamepad_controller import GamepadController, GAMEPAD_AVAILABLE
from src.detection.visual_detector import VisualDetector
from src.hardware.camera import DualCameraManager
from src.state.state_machine import StateMachine
from src.state.state_manager import StateManager
from src.web.app import create_app

logger = logging.getLogger(__name__)


class PTDTS:
    """
    Main PTDTS application
    Coordinates all subsystems
    """

    def __init__(self, config_path: str = None):
        """
        Initialize PTDTS application

        Args:
            config_path: Path to config file (default: config/config.yaml)
        """
        # Load configuration
        self.config = load_config(config_path)

        # Setup logging
        setup_logging(self.config)

        logger.info("=" * 60)
        logger.info("PTDTS - Pan Tilt Drone Detection and Tracking System")
        logger.info("=" * 60)
        logger.info(f"Simulation mode: {self.config.simulation_mode}")

        # Hardware components
        self.encoder = None
        self.pan_motor = None
        self.tilt_servo = None
        self.hq_camera = None
        self.gs_camera = None
        self.camera_manager = None
        self.acoustic_detector = None

        # Controllers
        self.pan_controller = None
        self.tilt_controller = None
        self.gamepad_controller = None

        # Detection
        self.visual_detector = None

        # State machine
        self.state_machine = None

        # State manager (for persistence)
        self.state_manager = None

        # Web interface
        self.web_app = None

        # Control flags
        self.running = False
        self.main_thread = None

        # Main loop timing monitor
        self.main_loop_timing = None

        # System monitor
        self.system_monitor = SystemMonitor(update_interval_s=5.0)

        # Initialize components
        self._initialize_hardware()
        self._initialize_controllers()
        self._initialize_detection()
        self._initialize_state_machine()
        self._initialize_state_manager()
        self._initialize_gamepad()
        self._initialize_web_interface()

        logger.info("PTDTS initialization complete")

    def _initialize_hardware(self):
        """Initialize hardware components"""
        logger.info("Initializing hardware...")

        # Create encoder
        self.encoder = HardwareFactory.create_encoder(self.config)

        # Create pan motor
        self.pan_motor = HardwareFactory.create_motor(self.config, self.encoder)

        # Create tilt servo
        self.tilt_servo = HardwareFactory.create_servo(self.config)

        # Create camera manager (includes both cameras)
        self.camera_manager = HardwareFactory.create_cameras(self.config)

        # Start cameras
        self.camera_manager.start_cameras()

        # Store individual camera references for convenience
        self.hq_camera = self.camera_manager.hq_camera
        self.gs_camera = self.camera_manager.gs_camera

        # Create acoustic detector
        self.acoustic_detector = HardwareFactory.create_acoustic_detector(self.config)

        logger.info("Hardware initialization complete")

    def _initialize_controllers(self):
        """Initialize control systems"""
        logger.info("Initializing controllers...")

        # Pan controller
        self.pan_controller = PanController(
            motor=self.pan_motor,
            encoder=self.encoder,
            config=self.config
        )

        # Tilt controller
        self.tilt_controller = TiltController(
            servo=self.tilt_servo,
            config=self.config
        )

        logger.info("Controllers initialized")

    def _initialize_detection(self):
        """Initialize detection systems"""
        logger.info("Initializing detection systems...")

        # Visual detector (YOLO)
        try:
            self.visual_detector = VisualDetector(
                model_path=self.config.yolo.model_path,
                confidence_threshold=self.config.yolo.detection_confidence,
                iou_threshold=self.config.yolo.iou_threshold,
                classes=self.config.yolo.classes,
                device=self.config.yolo.device
            )
            logger.info("Visual detector initialized")
        except Exception as e:
            logger.error(f"Failed to initialize visual detector: {e}")
            logger.warning("Continuing without visual detection")
            self.visual_detector = None

        # Acoustic detector already initialized in hardware

        logger.info("Detection systems initialized")

    def _initialize_state_machine(self):
        """Initialize state machine"""
        logger.info("Initializing state machine...")

        self.state_machine = StateMachine(
            pan_controller=self.pan_controller,
            tilt_controller=self.tilt_controller,
            camera_manager=self.camera_manager,
            visual_detector=self.visual_detector,
            acoustic_detector=self.acoustic_detector,
            config=self.config
        )

        logger.info("State machine initialized")

    def _initialize_state_manager(self):
        """Initialize state manager and restore saved state"""
        logger.info("Initializing state manager...")

        # Create state manager
        self.state_manager = StateManager()

        # Restore saved state
        logger.info("Restoring saved state...")
        pan_restored = self.pan_controller.load_state(self.state_manager)
        tilt_restored = self.tilt_controller.load_state(self.state_manager)
        sm_restored = self.state_machine.load_state(self.state_manager)

        if pan_restored or tilt_restored or sm_restored:
            logger.info("State restored successfully from previous session")
        else:
            logger.info("No previous state found, starting with defaults")

        logger.info("State manager initialized")

    def _initialize_gamepad(self):
        """Initialize gamepad controller"""
        # Check if gamepad is enabled in config
        if not hasattr(self.config, 'gamepad') or not getattr(self.config.gamepad, 'enabled', False):
            logger.info("Gamepad controller disabled in config")
            return

        if not GAMEPAD_AVAILABLE:
            logger.warning("Gamepad support not available (inputs library not installed)")
            return

        logger.info("Initializing gamepad controller...")

        try:
            self.gamepad_controller = GamepadController(self.config)

            # Register callbacks for gamepad events
            self.gamepad_controller.register_callback('toggle_mode', self._gamepad_toggle_mode)
            self.gamepad_controller.register_callback('emergency_stop', self._gamepad_emergency_stop)
            self.gamepad_controller.register_callback('center_pan', self._gamepad_center_pan)
            self.gamepad_controller.register_callback('reset_tilt', self._gamepad_reset_tilt)
            self.gamepad_controller.register_callback('quick_center', self._gamepad_quick_center)
            self.gamepad_controller.register_callback('toggle_listening', self._gamepad_toggle_listening)
            self.gamepad_controller.register_callback('step_pan_left', self._gamepad_step_pan_left)
            self.gamepad_controller.register_callback('step_pan_right', self._gamepad_step_pan_right)

            logger.info("Gamepad controller initialized successfully")
        except Exception as e:
            logger.warning(f"Gamepad controller not available (no device connected): {e}")
            logger.info("System will run without gamepad support - use web interface for manual control")
            self.gamepad_controller = None

    # Gamepad callback methods
    def _gamepad_toggle_mode(self):
        """Toggle between manual and auto mode"""
        current_state = self.state_machine.get_current_state().value
        if current_state == 'manual':
            logger.info("Gamepad: Switching to auto mode")
            self.state_machine.request_auto_mode()
        else:
            logger.info("Gamepad: Switching to manual mode")
            self.state_machine.request_manual_mode()

    def _gamepad_emergency_stop(self):
        """Emergency stop - halt all motion"""
        logger.warning("Gamepad: EMERGENCY STOP")
        self.pan_controller.set_idle()
        self.tilt_controller.set_target_angle(self.tilt_controller.current_angle)

    def _gamepad_center_pan(self):
        """Center pan to 0 degrees"""
        logger.info("Gamepad: Centering pan to 0°")
        target = self.config.gamepad.quick_center_pan
        self.pan_controller.set_position_target(target)

    def _gamepad_reset_tilt(self):
        """Reset tilt to horizontal (90 degrees)"""
        logger.info("Gamepad: Resetting tilt to 90°")
        target = self.config.gamepad.quick_center_tilt
        self.tilt_controller.set_target_angle(target)

    def _gamepad_quick_center(self):
        """Quick center both pan and tilt"""
        logger.info("Gamepad: Quick center (pan 0°, tilt 90°)")
        pan_target = self.config.gamepad.quick_center_pan
        tilt_target = self.config.gamepad.quick_center_tilt
        self.pan_controller.set_position_target(pan_target)
        self.tilt_controller.set_target_angle(tilt_target)

    def _gamepad_toggle_listening(self):
        """Toggle listening state"""
        logger.info("Gamepad: Toggling listening state")
        current_state = self.state_machine.get_current_state().value
        if current_state != 'listening':
            self.state_machine.request_auto_mode()  # Go back to listening

    def _gamepad_step_pan_left(self):
        """Step pan left by configured amount"""
        step = self.config.gamepad.pan_step_angle
        current_angle = self.pan_controller.get_current_angle()
        target = (current_angle - step) % 360
        logger.info(f"Gamepad: Step pan left to {target:.1f}°")
        self.pan_controller.set_position_target(target)

    def _gamepad_step_pan_right(self):
        """Step pan right by configured amount"""
        step = self.config.gamepad.pan_step_angle
        current_angle = self.pan_controller.get_current_angle()
        target = (current_angle + step) % 360
        logger.info(f"Gamepad: Step pan right to {target:.1f}°")
        self.pan_controller.set_position_target(target)

    def _initialize_web_interface(self):
        """Initialize web interface"""
        logger.info("Initializing web interface...")

        self.web_app = create_app(
            state_machine=self.state_machine,
            pan_controller=self.pan_controller,
            tilt_controller=self.tilt_controller,
            camera_manager=self.camera_manager,
            visual_detector=self.visual_detector,
            config=self.config,
            system_monitor=self.system_monitor,
            gamepad_controller=self.gamepad_controller
        )

        logger.info("Web interface initialized")

    def start(self):
        """Start PTDTS system"""
        logger.info("Starting PTDTS...")

        self.running = True

        # Start system monitor
        self.system_monitor.start()

        # Start pan controller in separate thread
        self.pan_controller.start()

        # Start acoustic detector if enabled
        if self.acoustic_detector and self.config.acoustic.enabled:
            self.acoustic_detector.start()

        # Start gamepad controller if enabled
        if self.gamepad_controller:
            try:
                self.gamepad_controller.start()
                logger.info("Gamepad controller started")
            except Exception as e:
                logger.error(f"Failed to start gamepad controller: {e}")

        # Start main control loop in separate thread
        self.main_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.main_thread.start()

        logger.info("PTDTS started")

        # Start web interface (blocking)
        logger.info(f"Starting web interface on {self.config.web.host}:{self.config.web.port}")
        try:
            self.web_app.run(
                host=self.config.web.host,
                port=self.config.web.port,
                debug=self.config.web.debug
            )
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received")
            self.stop()

    def _main_loop(self):
        """Main control loop"""
        logger.info("Main control loop started")

        # Use separate main loop rate (lower than motor control for performance)
        loop_rate = self.config.performance.main_loop_rate_hz
        loop_period = 1.0 / loop_rate

        # Initialize timing monitor
        self.main_loop_timing = LoopTimingMonitor(
            target_rate_hz=loop_rate,
            window_size=100
        )

        last_loop_time = time.time()

        while self.running:
            loop_start = time.time()

            try:
                # Start timing measurement
                self.main_loop_timing.start_iteration()

                # Update state machine
                self.state_machine.update()

                # Apply gamepad control if in manual mode and gamepad connected
                # Only override manual control if gamepad is actively being used
                if self.gamepad_controller and self.gamepad_controller.is_connected():
                    current_state = self.state_machine.get_current_state().value
                    if current_state == 'manual':
                        # Get gamepad commands
                        pan_velocity = self.gamepad_controller.get_pan_velocity_command()
                        tilt_delta = self.gamepad_controller.get_tilt_angle_delta()

                        # Apply pan velocity command only if gamepad stick is actively moved
                        # This prevents gamepad from overriding web UI commands when idle
                        if abs(pan_velocity) > 0.1:
                            self.pan_controller.set_manual_velocity(pan_velocity)
                        # Note: Do NOT set to 0.0 when gamepad is idle, to allow web UI control

                        # Apply tilt delta command only if gamepad is actively moved
                        if abs(tilt_delta) > 0.01:
                            current_tilt = self.tilt_controller.current_angle
                            new_tilt = max(
                                self.config.tilt_servo.min_angle,
                                min(self.config.tilt_servo.max_angle, current_tilt + tilt_delta)
                            )
                            self.tilt_controller.set_target_angle(new_tilt)

                # Update tilt controller
                self.tilt_controller.update()

                # End timing measurement
                self.main_loop_timing.end_iteration()

                # Record actual loop period
                current_time = time.time()
                loop_period_actual = current_time - last_loop_time
                self.main_loop_timing.record_loop_period(loop_period_actual)
                last_loop_time = current_time

                # Sleep for remainder of loop period
                elapsed = time.time() - loop_start
                sleep_time = loop_period - elapsed

                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    logger.warning(f"Loop overrun: {elapsed:.3f}s (target: {loop_period:.3f}s)")

            except Exception as e:
                logger.error(f"Error in main loop: {e}", exc_info=True)
                time.sleep(0.1)  # Prevent tight loop on error

        logger.info("Main control loop stopped")

    def stop(self):
        """Stop PTDTS system"""
        logger.info("Stopping PTDTS...")

        # Save state before shutting down
        if self.state_manager:
            try:
                logger.info("Saving system state...")
                if self.pan_controller:
                    self.pan_controller.save_state(self.state_manager)
                if self.tilt_controller:
                    self.tilt_controller.save_state(self.state_manager)
                if self.state_machine:
                    self.state_machine.save_state(self.state_manager)
                # Write to disk
                self.state_manager.save()
                logger.info("System state saved successfully")
            except Exception as e:
                logger.error(f"Error saving state: {e}", exc_info=True)

        # Stop main loop
        self.running = False

        if self.main_thread:
            self.main_thread.join(timeout=2.0)

        # Stop controllers
        if self.pan_controller:
            self.pan_controller.stop()

        # Stop gamepad controller
        if self.gamepad_controller:
            self.gamepad_controller.stop()

        # Stop acoustic detector
        if self.acoustic_detector:
            self.acoustic_detector.stop()

        # Stop system monitor
        if self.system_monitor:
            self.system_monitor.stop()

        # Stop cameras
        if self.camera_manager:
            self.camera_manager.stop_cameras()
            self.camera_manager.close()

        logger.info("PTDTS stopped")

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop()


def signal_handler(signum, frame):
    """Handle shutdown signals"""
    logger.info(f"Received signal {signum}, shutting down...")
    sys.exit(0)


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="PTDTS - Pan Tilt Drone Detection and Tracking System")
    parser.add_argument(
        "-c", "--config",
        default="config/config.yaml",
        help="Path to configuration file (default: config/config.yaml)"
    )
    parser.add_argument(
        "-s", "--simulation",
        action="store_true",
        help="Force simulation mode"
    )
    parser.add_argument(
        "--host",
        help="Web interface host (overrides config)"
    )
    parser.add_argument(
        "--port",
        type=int,
        help="Web interface port (overrides config)"
    )

    args = parser.parse_args()

    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # Create and start PTDTS
        with PTDTS(config_path=args.config) as ptdts:
            # Override config if specified
            if args.simulation:
                ptdts.config.simulation_mode = True
                logger.info("Simulation mode forced via command line")

            if args.host:
                ptdts.config.web.host = args.host

            if args.port:
                ptdts.config.web.port = args.port

            # Start system
            ptdts.start()

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)

    logger.info("PTDTS shutdown complete")


if __name__ == "__main__":
    main()
