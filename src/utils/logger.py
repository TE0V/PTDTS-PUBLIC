"""
Logging configuration for PTDTS
Sets up console and file logging with rotation
"""

import logging
import logging.handlers
import sys
import os
from pathlib import Path
from typing import Optional
import colorlog


def setup_logging(config=None) -> logging.Logger:
    """
    Set up logging with console and file handlers

    Args:
        config: Configuration object with logging settings (optional)

    Returns:
        Root logger
    """
    # Extract logging settings from config
    if config is not None:
        console_level = config.logging.console_level
        file_enabled = config.logging.file_enabled
        file_level = config.logging.file_level
        file_path = config.logging.file_path
        file_max_bytes = config.logging.file_max_bytes
        file_backup_count = config.logging.file_backup_count
    else:
        # Defaults
        console_level = "INFO"
        file_enabled = True
        file_level = "DEBUG"
        file_path = "logs/ptdts.log"
        file_max_bytes = 10485760  # 10MB
        file_backup_count = 5

    # Convert string levels to logging constants
    console_level_int = getattr(logging, console_level.upper(), logging.INFO)
    file_level_int = getattr(logging, file_level.upper(), logging.DEBUG)

    # Create root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)  # Capture all, filter in handlers

    # Remove any existing handlers
    root_logger.handlers = []

    # Console handler with color
    console_handler = colorlog.StreamHandler(sys.stdout)
    console_handler.setLevel(console_level_int)

    console_formatter = colorlog.ColoredFormatter(
        '%(log_color)s%(asctime)s - %(name)-20s - %(levelname)-8s%(reset)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        log_colors={
            'DEBUG': 'cyan',
            'INFO': 'green',
            'WARNING': 'yellow',
            'ERROR': 'red',
            'CRITICAL': 'red,bg_white',
        }
    )
    console_handler.setFormatter(console_formatter)
    root_logger.addHandler(console_handler)

    # File handler with rotation
    if file_enabled:
        # Ensure log directory exists
        log_dir = os.path.dirname(file_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

        file_handler = logging.handlers.RotatingFileHandler(
            file_path,
            maxBytes=file_max_bytes,
            backupCount=file_backup_count
        )
        file_handler.setLevel(file_level_int)

        file_formatter = logging.Formatter(
            '%(asctime)s - %(name)-25s - %(levelname)-8s - %(funcName)-20s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(file_formatter)
        root_logger.addHandler(file_handler)

    return root_logger


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name

    Args:
        name: Logger name (typically __name__)

    Returns:
        Logger instance
    """
    return logging.getLogger(name)


class TelemetryLogger:
    """
    CSV logger for telemetry data
    Writes timestamped telemetry to CSV file
    """

    def __init__(self, file_path: str, rate_hz: int = 10):
        """
        Initialize telemetry logger

        Args:
            file_path: Path to CSV file
            rate_hz: Logging rate in Hz
        """
        self.file_path = file_path
        self.rate_hz = rate_hz
        self.file_handle: Optional[object] = None
        self.headers_written = False

        # Ensure directory exists
        log_dir = os.path.dirname(file_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

    def open(self):
        """Open CSV file for writing"""
        # Check if file exists to determine if we need headers
        file_exists = os.path.exists(self.file_path)

        self.file_handle = open(self.file_path, 'a', buffering=1)  # Line buffered

        self.headers_written = file_exists

    def write_headers(self, headers: list):
        """Write CSV headers"""
        if not self.headers_written and self.file_handle:
            self.file_handle.write(','.join(headers) + '\n')
            self.headers_written = True

    def write_row(self, row: list):
        """Write a data row to CSV"""
        if self.file_handle:
            self.file_handle.write(','.join(map(str, row)) + '\n')

    def close(self):
        """Close CSV file"""
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None

    def __enter__(self):
        """Context manager entry"""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()


class DetectionLogger:
    """
    CSV logger for detection events
    Writes timestamped detection data
    """

    def __init__(self, file_path: str):
        """
        Initialize detection logger

        Args:
            file_path: Path to CSV file
        """
        self.file_path = file_path
        self.file_handle: Optional[object] = None
        self.headers_written = False

        # Ensure directory exists
        log_dir = os.path.dirname(file_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

    def open(self):
        """Open CSV file for writing"""
        # Check if file exists
        file_exists = os.path.exists(self.file_path)

        self.file_handle = open(self.file_path, 'a', buffering=1)

        self.headers_written = file_exists

        # Write headers if new file
        if not self.headers_written:
            headers = [
                'timestamp',
                'source',  # 'acoustic' or 'visual'
                'azimuth',
                'elevation',
                'confidence',
                'classification',
                'position_x',
                'position_y',
                'position_z'
            ]
            self.file_handle.write(','.join(headers) + '\n')
            self.headers_written = True

    def log_detection(
        self,
        timestamp: float,
        source: str,
        azimuth: float,
        elevation: float,
        confidence: float,
        classification: str = "unknown",
        position: tuple = (0.0, 0.0, 0.0)
    ):
        """
        Log a detection event

        Args:
            timestamp: Unix timestamp
            source: Detection source ('acoustic' or 'visual')
            azimuth: Azimuth in degrees
            elevation: Elevation in degrees
            confidence: Detection confidence (0-1)
            classification: Object classification
            position: 3D position (x, y, z) in meters
        """
        if self.file_handle:
            row = [
                timestamp,
                source,
                azimuth,
                elevation,
                confidence,
                classification,
                position[0],
                position[1],
                position[2]
            ]
            self.file_handle.write(','.join(map(str, row)) + '\n')

    def close(self):
        """Close CSV file"""
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None

    def __enter__(self):
        """Context manager entry"""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()
