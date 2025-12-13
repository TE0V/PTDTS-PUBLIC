"""
Camera implementations for PTDTS
Picamera2 for Raspberry Pi cameras and simulated camera
"""

import time
import logging
import threading
import numpy as np
from typing import Tuple, Optional, List
from collections import deque
from .interfaces import CameraInterface

logger = logging.getLogger(__name__)


class FramerateMonitor:
    """
    Monitor actual framerate by tracking frame capture timestamps
    """

    def __init__(self, window_size: int = 30):
        """
        Initialize framerate monitor

        Args:
            window_size: Number of frames to average for FPS calculation
        """
        self.window_size = window_size
        self.frame_timestamps: deque = deque(maxlen=window_size)
        self.frame_count = 0
        self.actual_fps = 0.0
        self.capture_latency_ms = 0.0
        self.lock = threading.Lock()

    def record_frame(self, capture_start_time: Optional[float] = None):
        """
        Record a frame capture event

        Args:
            capture_start_time: Time when capture was initiated (for latency measurement)
        """
        with self.lock:
            current_time = time.time()
            self.frame_timestamps.append(current_time)
            self.frame_count += 1

            # Calculate capture latency if start time provided
            if capture_start_time:
                self.capture_latency_ms = (current_time - capture_start_time) * 1000

            # Calculate actual FPS from timestamps
            if len(self.frame_timestamps) >= 2:
                time_span = self.frame_timestamps[-1] - self.frame_timestamps[0]
                if time_span > 0:
                    self.actual_fps = (len(self.frame_timestamps) - 1) / time_span

    def get_actual_fps(self) -> float:
        """
        Get actual measured framerate

        Returns:
            Actual FPS (averaged over window)
        """
        with self.lock:
            return self.actual_fps

    def get_capture_latency_ms(self) -> float:
        """
        Get last frame capture latency

        Returns:
            Latency in milliseconds
        """
        with self.lock:
            return self.capture_latency_ms

    def get_frame_count(self) -> int:
        """
        Get total frame count

        Returns:
            Number of frames captured
        """
        with self.lock:
            return self.frame_count


class Picamera2Camera(CameraInterface):
    """
    Raspberry Pi camera via Picamera2 library
    Supports HQ and Global Shutter cameras
    """

    def __init__(
        self,
        camera_index: int = 0,
        resolution: Tuple[int, int] = (1280, 720),
        framerate: int = 30,
        format: str = "RGB888",
        controls: Optional[dict] = None
    ):
        """
        Initialize Picamera2 camera

        Args:
            camera_index: Camera index (0 or 1)
            resolution: (width, height) tuple
            framerate: Target framerate
            format: Pixel format (RGB888, BGR888, etc.)
            controls: Optional dict of camera controls (exposure, gain, etc.)
        """
        self.camera_index = camera_index
        self.resolution = resolution
        self.framerate = framerate
        self.format = format
        self.controls = controls or {}

        self.is_started = False
        self.picam2 = None
        self.fps_monitor = FramerateMonitor(window_size=30)

        try:
            from picamera2 import Picamera2

            # Initialize camera
            self.picam2 = Picamera2(camera_index)

            # Configure camera with minimal buffering to reduce latency/desync
            config = self.picam2.create_preview_configuration(
                main={"size": resolution, "format": format},
                buffer_count=2  # Minimal buffering (2 is minimum for double-buffering)
            )
            self.picam2.configure(config)

            # Apply camera controls if provided (for manual exposure, gain, etc.)
            if self.controls:
                logger.info(f"Applying camera controls: {self.controls}")
                self.picam2.set_controls(self.controls)

            logger.info(f"Picamera2 initialized: camera{camera_index}, {resolution[0]}x{resolution[1]} @ {framerate}fps, buffer_count=2")

        except ImportError:
            logger.error("Picamera2 library not available")
            raise
        except Exception as e:
            logger.error(f"Failed to initialize Picamera2: {e}")
            raise

    def start(self):
        """Start camera"""
        if not self.is_started:
            self.picam2.start()
            time.sleep(2)  # Camera warm-up
            self.is_started = True
            logger.debug(f"Camera {self.camera_index} started")

    def stop(self):
        """Stop camera"""
        if self.is_started:
            self.picam2.stop()
            self.is_started = False
            logger.debug(f"Camera {self.camera_index} stopped")

    def capture_array(self) -> np.ndarray:
        """
        Capture a frame as numpy array

        Returns:
            Frame as numpy array (H, W, 3) in RGB format
        """
        if not self.is_started:
            logger.warning("Camera not started, starting now")
            self.start()

        capture_start = time.time()

        try:
            frame = self.picam2.capture_array()
            self.fps_monitor.record_frame(capture_start)
            return frame

        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            # Return black frame as fallback
            return np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)

    def get_resolution(self) -> Tuple[int, int]:
        """
        Get camera resolution

        Returns:
            (width, height) tuple
        """
        return self.resolution

    def get_framerate(self) -> int:
        """
        Get configured camera framerate

        Returns:
            Configured framerate in FPS
        """
        return self.framerate

    def get_actual_fps(self) -> float:
        """
        Get actual measured framerate

        Returns:
            Actual FPS (averaged over recent frames)
        """
        return self.fps_monitor.get_actual_fps()

    def get_capture_latency_ms(self) -> float:
        """
        Get frame capture latency

        Returns:
            Latency in milliseconds
        """
        return self.fps_monitor.get_capture_latency_ms()

    def get_frame_count(self) -> int:
        """
        Get total frames captured

        Returns:
            Frame count
        """
        return self.fps_monitor.get_frame_count()

    def get_performance_metrics(self) -> dict:
        """
        Get comprehensive camera performance metrics

        Returns:
            Dictionary with performance information
        """
        actual_fps = self.get_actual_fps()
        configured_fps = self.framerate
        fps_deviation = ((actual_fps - configured_fps) / configured_fps * 100) if configured_fps > 0 else 0

        return {
            'configured_fps': configured_fps,
            'actual_fps': actual_fps,
            'fps_deviation_percent': fps_deviation,
            'capture_latency_ms': self.get_capture_latency_ms(),
            'frame_count': self.get_frame_count(),
            'dropped_frames': max(0, int((configured_fps - actual_fps) * (self.get_frame_count() / actual_fps))) if actual_fps > 0 else 0
        }

    def close(self):
        """Clean up resources"""
        if self.is_started:
            self.stop()
        if self.picam2:
            self.picam2.close()
        logger.info(f"Camera {self.camera_index} closed")


class SimulatedCamera(CameraInterface):
    """
    Simulated camera for testing without hardware
    Generates synthetic frames with moving test pattern
    """

    def __init__(
        self,
        camera_index: int = 0,
        resolution: Tuple[int, int] = (1280, 720),
        framerate: int = 30,
        format: str = "RGB888"
    ):
        """
        Initialize simulated camera

        Args:
            camera_index: Camera index (for identification)
            resolution: (width, height) tuple
            framerate: Target framerate
            format: Pixel format (not used in simulation)
        """
        self.camera_index = camera_index
        self.resolution = resolution
        self.framerate = framerate
        self.format = format

        self.is_started = False
        self.frame_count = 0
        self.fps_monitor = FramerateMonitor(window_size=30)

        # Test pattern parameters
        self.pattern_offset = 0
        self.pattern_speed = 2  # pixels per frame

        logger.info(f"Simulated camera initialized: camera{camera_index}, {resolution[0]}x{resolution[1]} @ {framerate}fps")

    def start(self):
        """Start camera"""
        self.is_started = True
        logger.debug(f"Simulated camera {self.camera_index} started")

    def stop(self):
        """Stop camera"""
        self.is_started = False
        logger.debug(f"Simulated camera {self.camera_index} stopped")

    def capture_array(self) -> np.ndarray:
        """
        Capture a simulated frame

        Returns:
            Synthetic frame as numpy array (H, W, 3) in RGB format
        """
        if not self.is_started:
            logger.warning("Simulated camera not started, starting now")
            self.start()

        capture_start = time.time()
        width, height = self.resolution
        frame = np.zeros((height, width, 3), dtype=np.uint8)

        # Generate test pattern: moving gradient with grid
        for y in range(height):
            for x in range(width):
                # Moving gradient
                r = int((x + self.pattern_offset) % 256)
                g = int((y + self.pattern_offset) % 256)
                b = 128

                frame[y, x] = [r, g, b]

        # Draw grid lines
        grid_spacing = 50
        for i in range(0, width, grid_spacing):
            frame[:, i] = [255, 255, 255]
        for i in range(0, height, grid_spacing):
            frame[i, :] = [255, 255, 255]

        # Draw center crosshair
        center_x = width // 2
        center_y = height // 2
        crosshair_size = 30
        frame[center_y - crosshair_size:center_y + crosshair_size, center_x - 2:center_x + 2] = [0, 255, 0]
        frame[center_y - 2:center_y + 2, center_x - crosshair_size:center_x + crosshair_size] = [0, 255, 0]

        # Draw text overlay with frame info
        try:
            import cv2
            text = f"Camera {self.camera_index} - Frame {self.frame_count}"
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"{width}x{height} @ {self.framerate}fps", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        except:
            pass  # If OpenCV not available, skip text

        # Update pattern
        self.pattern_offset = (self.pattern_offset + self.pattern_speed) % 256
        self.frame_count += 1

        # Simulate framerate delay
        time.sleep(1.0 / self.framerate)

        # Record frame for FPS monitoring
        self.fps_monitor.record_frame(capture_start)

        return frame

    def get_resolution(self) -> Tuple[int, int]:
        """
        Get camera resolution

        Returns:
            (width, height) tuple
        """
        return self.resolution

    def get_framerate(self) -> int:
        """
        Get configured camera framerate

        Returns:
            Configured framerate in FPS
        """
        return self.framerate

    def get_actual_fps(self) -> float:
        """
        Get actual measured framerate

        Returns:
            Actual FPS (averaged over recent frames)
        """
        return self.fps_monitor.get_actual_fps()

    def get_capture_latency_ms(self) -> float:
        """
        Get frame capture latency

        Returns:
            Latency in milliseconds
        """
        return self.fps_monitor.get_capture_latency_ms()

    def get_frame_count(self) -> int:
        """
        Get total frames captured

        Returns:
            Frame count
        """
        return self.fps_monitor.get_frame_count()

    def get_performance_metrics(self) -> dict:
        """
        Get comprehensive camera performance metrics

        Returns:
            Dictionary with performance information
        """
        actual_fps = self.get_actual_fps()
        configured_fps = self.framerate
        fps_deviation = ((actual_fps - configured_fps) / configured_fps * 100) if configured_fps > 0 else 0

        return {
            'configured_fps': configured_fps,
            'actual_fps': actual_fps,
            'fps_deviation_percent': fps_deviation,
            'capture_latency_ms': self.get_capture_latency_ms(),
            'frame_count': self.get_frame_count(),
            'dropped_frames': 0  # Simulated camera doesn't drop frames
        }

    def close(self):
        """Clean up (no-op for simulation)"""
        self.is_started = False
        logger.info(f"Simulated camera {self.camera_index} closed")


class DualCameraManager:
    """
    Manager for dual camera system (HQ + GS)
    Handles camera switching and synchronization
    """

    def __init__(
        self,
        hq_camera: CameraInterface,
        gs_camera: CameraInterface,
        settling_delay_ms: int = 500
    ):
        """
        Initialize dual camera manager

        Args:
            hq_camera: High-quality camera (wide FOV, detection)
            gs_camera: Global shutter camera (tracking)
            settling_delay_ms: Delay after camera switch before using frames
        """
        self.hq_camera = hq_camera
        self.gs_camera = gs_camera
        self.settling_delay_ms = settling_delay_ms

        self.active_camera = hq_camera  # Start with HQ
        self.last_switch_time = 0
        self.lock = threading.Lock()
        self.manual_override = False  # Manual camera control override flag

        logger.info("Dual camera manager initialized")

    def start_cameras(self):
        """Start both cameras"""
        self.hq_camera.start()
        self.gs_camera.start()
        logger.info("Both cameras started")

    def stop_cameras(self):
        """Stop both cameras"""
        self.hq_camera.stop()
        self.gs_camera.stop()
        logger.info("Both cameras stopped")

    def switch_to_hq(self, force: bool = False):
        """
        Switch to HQ camera (detection mode)

        Args:
            force: If True, switch even if manual override is active
        """
        with self.lock:
            if self.manual_override and not force:
                logger.debug("HQ camera switch blocked by manual override")
                return
            if self.active_camera != self.hq_camera:
                self.active_camera = self.hq_camera
                self.last_switch_time = time.time()
                logger.debug("Switched to HQ camera (detection mode)")

    def switch_to_gs(self, force: bool = False):
        """
        Switch to GS camera (tracking mode)

        Args:
            force: If True, switch even if manual override is active
        """
        with self.lock:
            if self.manual_override and not force:
                logger.debug("GS camera switch blocked by manual override")
                return
            if self.active_camera != self.gs_camera:
                self.active_camera = self.gs_camera
                self.last_switch_time = time.time()
                logger.debug("Switched to GS camera (tracking mode)")

    def set_manual_camera(self, camera_name: str):
        """
        Manually select camera and enable manual override

        Args:
            camera_name: "hq" or "gs"

        Raises:
            ValueError: If camera_name is not valid
        """
        with self.lock:
            if camera_name.lower() == "hq":
                self.active_camera = self.hq_camera
                self.last_switch_time = time.time()
                self.manual_override = True
                logger.info("Manually switched to HQ camera (override enabled)")
            elif camera_name.lower() == "gs":
                self.active_camera = self.gs_camera
                self.last_switch_time = time.time()
                self.manual_override = True
                logger.info("Manually switched to GS camera (override enabled)")
            else:
                raise ValueError(f"Invalid camera name '{camera_name}'. Must be 'hq' or 'gs'")

    def disable_manual_override(self):
        """Disable manual camera override, allowing automatic camera switching"""
        with self.lock:
            self.manual_override = False
            logger.info("Manual camera override disabled")

    def is_manual_override_active(self) -> bool:
        """
        Check if manual camera override is active

        Returns:
            True if manual override is enabled
        """
        with self.lock:
            return self.manual_override

    def is_settled(self) -> bool:
        """
        Check if camera has settled after switch

        Returns:
            True if enough time has passed since last switch
        """
        with self.lock:
            elapsed_ms = (time.time() - self.last_switch_time) * 1000
            return elapsed_ms > self.settling_delay_ms

    def capture_frame(self) -> Tuple[np.ndarray, str]:
        """
        Capture frame from active camera

        Returns:
            (frame, camera_name) tuple
        """
        with self.lock:
            frame = self.active_camera.capture_array()
            camera_name = "hq" if self.active_camera == self.hq_camera else "gs"
            return frame, camera_name

    def get_active_camera_name(self) -> str:
        """
        Get name of active camera

        Returns:
            "hq" or "gs"
        """
        with self.lock:
            return "hq" if self.active_camera == self.hq_camera else "gs"

    def close(self):
        """Clean up both cameras"""
        self.hq_camera.close()
        self.gs_camera.close()
        logger.info("Dual camera manager closed")
