"""
Acoustic detection implementations for PTDTS
ODAS integration for ReSpeaker array and simulated acoustic detector
"""

import time
import socket
import json
import logging
import threading
import subprocess
import numpy as np
from typing import List, Dict, Optional
from collections import deque
from .interfaces import AudioInterface

logger = logging.getLogger(__name__)


class ODASAcousticDetector(AudioInterface):
    """
    ODAS (Open embeddeD Audition System) integration
    Receives sound source localization data from ODAS server
    """

    def __init__(
        self,
        odas_port: int = 9000,
        mic_radius_mm: float = 46.5,
        mic_count: int = 4,
        energy_threshold: float = 0.6,
        frequency_min: float = 100.0,
        frequency_max: float = 500.0,
        max_detections: int = 10,
        odas_config_path: Optional[str] = None
    ):
        """
        Initialize ODAS acoustic detector

        Args:
            odas_port: Port for ODAS socket connection
            mic_radius_mm: Microphone array radius in mm
            mic_count: Number of microphones
            energy_threshold: Minimum energy threshold for detection
            frequency_min: Minimum frequency of interest (Hz)
            frequency_max: Maximum frequency of interest (Hz)
            max_detections: Maximum number of detections to keep
            odas_config_path: Path to ODAS configuration file
        """
        self.odas_port = odas_port
        self.mic_radius_mm = mic_radius_mm
        self.mic_count = mic_count
        self.energy_threshold = energy_threshold
        self.frequency_min = frequency_min
        self.frequency_max = frequency_max
        self.max_detections = max_detections
        self.odas_config_path = odas_config_path

        # Detection buffer (thread-safe)
        self.detections = deque(maxlen=max_detections)
        self.lock = threading.Lock()

        # Socket connection
        self.socket = None
        self.running = False
        self.listener_thread: Optional[threading.Thread] = None

        # ODAS process
        self.odas_process: Optional[subprocess.Popen] = None

        logger.info(f"ODAS acoustic detector initialized: port={odas_port}, threshold={energy_threshold}")

    def _generate_odas_config(self) -> str:
        """
        Generate ODAS configuration file

        Returns:
            Path to generated config file
        """
        # Convert mic radius from mm to meters
        mic_radius_m = self.mic_radius_mm / 1000.0

        # Generate mic positions (circular array)
        mic_positions = []
        for i in range(self.mic_count):
            angle = (2 * np.pi * i) / self.mic_count
            x = mic_radius_m * np.cos(angle)
            y = mic_radius_m * np.sin(angle)
            z = 0.0
            mic_positions.append([x, y, z])

        # ODAS configuration (simplified version)
        config = {
            "microphones": {
                "nChannels": self.mic_count,
                "gain": [1.0] * self.mic_count,
                "mics": [
                    {"x": pos[0], "y": pos[1], "z": pos[2]}
                    for pos in mic_positions
                ]
            },
            "ssl": {
                "enabled": True,
                "nChannels": self.mic_count,
                "nPots": 4,
                "nTracks": 2,
                "threshold": self.energy_threshold,
                "frequency": {
                    "min": self.frequency_min,
                    "max": self.frequency_max
                }
            },
            "sst": {
                "enabled": True,
                "nChannels": self.mic_count,
                "nTracks": 2
            }
        }

        # Save to file
        config_path = "/tmp/ptdts_odas_config.json"
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)

        logger.debug(f"Generated ODAS config: {config_path}")
        return config_path

    def _start_odas_server(self):
        """Start ODAS server process"""
        if self.odas_config_path is None:
            self.odas_config_path = self._generate_odas_config()

        try:
            # Start ODAS (assuming odaslive is in PATH)
            self.odas_process = subprocess.Popen(
                ['odaslive', '-c', self.odas_config_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            logger.info("ODAS server started")
            time.sleep(2)  # Give ODAS time to initialize

        except FileNotFoundError:
            logger.warning("ODAS not found, acoustic detection will not be available")
            self.odas_process = None
        except Exception as e:
            logger.error(f"Failed to start ODAS: {e}")
            self.odas_process = None

    def start(self):
        """Start acoustic detection"""
        if not self.running:
            # Start ODAS server
            self._start_odas_server()

            # Start socket listener
            self.running = True
            self.listener_thread = threading.Thread(target=self._listener_loop, daemon=True)
            self.listener_thread.start()

            logger.info("Acoustic detection started")

    def stop(self):
        """Stop acoustic detection"""
        if self.running:
            self.running = False

            # Stop listener thread
            if self.listener_thread:
                self.listener_thread.join(timeout=2.0)

            # Close socket
            if self.socket:
                self.socket.close()
                self.socket = None

            # Stop ODAS process
            if self.odas_process:
                self.odas_process.terminate()
                self.odas_process.wait(timeout=5.0)
                self.odas_process = None

            logger.info("Acoustic detection stopped")

    def _listener_loop(self):
        """Socket listener thread"""
        while self.running:
            try:
                # Connect to ODAS socket
                if self.socket is None:
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.socket.connect(('localhost', self.odas_port))
                    logger.debug(f"Connected to ODAS on port {self.odas_port}")

                # Receive data
                data = self.socket.recv(4096)
                if not data:
                    logger.warning("ODAS connection closed")
                    self.socket = None
                    time.sleep(1.0)
                    continue

                # Parse JSON
                try:
                    msg = json.loads(data.decode('utf-8'))
                    self._process_odas_message(msg)
                except json.JSONDecodeError:
                    logger.debug("Failed to parse ODAS JSON")

            except ConnectionRefusedError:
                logger.debug("ODAS not available, retrying...")
                self.socket = None
                time.sleep(2.0)
            except (OSError, socket.error) as e:
                # Handle socket errors (including errno 107 - Transport endpoint not connected)
                logger.warning(f"ODAS connection error: {e}, reconnecting...")
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                    self.socket = None
                time.sleep(2.0)
            except Exception as e:
                logger.error(f"Error in ODAS listener: {e}")
                # Reset socket on any error to allow reconnection
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                    self.socket = None
                time.sleep(1.0)

    def _process_odas_message(self, msg: Dict):
        """
        Process ODAS message and extract detections

        Args:
            msg: Parsed ODAS JSON message
        """
        if 'src' not in msg:
            return

        sources = msg['src']

        with self.lock:
            for source in sources:
                # Extract source data
                x = source.get('x', 0.0)
                y = source.get('y', 0.0)
                z = source.get('z', 0.0)
                energy = source.get('E', 0.0)

                # Filter by energy threshold
                if energy < self.energy_threshold:
                    continue

                # Convert Cartesian to spherical coordinates
                # Azimuth: 0° = North (+Y), 90° = East (+X)
                azimuth = np.degrees(np.arctan2(x, y))
                if azimuth < 0:
                    azimuth += 360

                # Elevation: 0° = horizon
                r = np.sqrt(x**2 + y**2 + z**2)
                if r > 0:
                    elevation = np.degrees(np.arcsin(z / r))
                else:
                    elevation = 0.0

                # Create detection
                detection = {
                    'azimuth': azimuth,
                    'elevation': elevation,
                    'energy': energy,
                    'timestamp': time.time()
                }

                # Add to buffer
                self.detections.append(detection)

    def get_detections(self) -> List[Dict]:
        """
        Get recent acoustic detections

        Returns:
            List of detection dictionaries
        """
        with self.lock:
            return list(self.detections)

    def is_running(self) -> bool:
        """
        Check if acoustic detection is running

        Returns:
            True if running
        """
        return self.running

    def close(self):
        """Clean up resources"""
        self.stop()
        logger.info("ODAS acoustic detector closed")


class SimulatedAcoustic(AudioInterface):
    """
    Simulated acoustic detector for testing without hardware
    Generates synthetic detections at random intervals
    """

    def __init__(
        self,
        energy_threshold: float = 0.6,
        detection_rate_hz: float = 0.1,  # Average detections per second
        max_detections: int = 10
    ):
        """
        Initialize simulated acoustic detector

        Args:
            energy_threshold: Minimum energy threshold
            detection_rate_hz: Average detection rate
            max_detections: Maximum detections to keep
        """
        self.energy_threshold = energy_threshold
        self.detection_rate_hz = detection_rate_hz
        self.max_detections = max_detections

        self.detections = deque(maxlen=max_detections)
        self.lock = threading.Lock()

        self.running = False
        self.simulator_thread: Optional[threading.Thread] = None

        logger.info(f"Simulated acoustic detector initialized: rate={detection_rate_hz}Hz")

    def start(self):
        """Start simulated acoustic detection"""
        if not self.running:
            self.running = True
            self.simulator_thread = threading.Thread(target=self._simulator_loop, daemon=True)
            self.simulator_thread.start()
            logger.info("Simulated acoustic detection started")

    def stop(self):
        """Stop simulated acoustic detection"""
        if self.running:
            self.running = False
            if self.simulator_thread:
                self.simulator_thread.join(timeout=2.0)
            logger.info("Simulated acoustic detection stopped")

    def _simulator_loop(self):
        """Simulation thread - generates random detections"""
        while self.running:
            try:
                # Random delay between detections
                mean_interval = 1.0 / self.detection_rate_hz
                interval = np.random.exponential(mean_interval)
                time.sleep(interval)

                # Generate random detection
                azimuth = np.random.uniform(0, 360)
                elevation = np.random.uniform(-30, 30)  # Usually drones are near horizon
                energy = np.random.uniform(self.energy_threshold, 1.0)

                detection = {
                    'azimuth': azimuth,
                    'elevation': elevation,
                    'energy': energy,
                    'timestamp': time.time()
                }

                with self.lock:
                    self.detections.append(detection)

                logger.debug(f"Simulated detection: azimuth={azimuth:.1f}°, elevation={elevation:.1f}°, energy={energy:.2f}")

            except Exception as e:
                logger.error(f"Error in acoustic simulator: {e}")
                time.sleep(1.0)

    def get_detections(self) -> List[Dict]:
        """
        Get recent acoustic detections

        Returns:
            List of detection dictionaries
        """
        with self.lock:
            return list(self.detections)

    def is_running(self) -> bool:
        """
        Check if running

        Returns:
            True if running
        """
        return self.running

    def close(self):
        """Clean up resources"""
        self.stop()
        logger.info("Simulated acoustic detector closed")
