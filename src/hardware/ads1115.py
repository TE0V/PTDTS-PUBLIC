"""
ADS1115 16-bit ADC interface
For reading analog position feedback from Axon servo
"""

import time
import logging
import threading
from typing import Optional

logger = logging.getLogger(__name__)


class ADS1115Reader:
    """
    ADS1115 16-bit ADC via I2C
    Reads analog voltage from servo position feedback
    """

    def __init__(
        self,
        i2c_address: int = 0x48,
        channel: int = 0,
        gain: int = 1,  # ±4.096V range
        sample_rate: int = 860  # Samples per second
    ):
        """
        Initialize ADS1115 ADC

        Args:
            i2c_address: I2C address (default 0x48)
            channel: ADC channel (0-3)
            gain: PGA gain setting
                  1 = ±4.096V
                  2 = ±2.048V
                  4 = ±1.024V
                  8 = ±0.512V
                 16 = ±0.256V
            sample_rate: Sampling rate in SPS (8, 16, 32, 64, 128, 250, 475, 860)
        """
        self.i2c_address = i2c_address
        self.channel = channel
        self.gain = gain
        self.sample_rate = sample_rate

        self.voltage = 0.0
        self.lock = threading.Lock()

        # Moving average filter
        self.filter_size = 5
        self.voltage_buffer = []

        # Error tracking
        self.error_count = 0
        self.last_error_time = None
        self.consecutive_errors = 0

        try:
            import board
            import busio
            import adafruit_ads1x15.ads1115 as ADS
            from adafruit_ads1x15.analog_in import AnalogIn

            # Initialize I2C
            i2c = busio.I2C(board.SCL, board.SDA)

            # Initialize ADS1115
            self.ads = ADS.ADS1115(i2c, address=i2c_address)

            # Set gain
            self.ads.gain = gain

            # Set data rate
            self.ads.data_rate = sample_rate

            # Create analog input channel
            if channel == 0:
                self.chan = AnalogIn(self.ads, ADS.P0)
            elif channel == 1:
                self.chan = AnalogIn(self.ads, ADS.P1)
            elif channel == 2:
                self.chan = AnalogIn(self.ads, ADS.P2)
            elif channel == 3:
                self.chan = AnalogIn(self.ads, ADS.P3)
            else:
                raise ValueError(f"Invalid channel: {channel}")

            logger.info(f"ADS1115 initialized: address=0x{i2c_address:02X}, channel={channel}, gain={gain}")

        except ImportError:
            logger.warning("ADS1115 library not available, running in dummy mode")
            self.ads = None
            self.chan = None
        except Exception as e:
            logger.error(f"Failed to initialize ADS1115: {e}")
            raise

    def read_voltage(self) -> float:
        """
        Read voltage from ADC with filtering

        Returns:
            Voltage in volts
        """
        with self.lock:
            if self.chan is None:
                # Dummy mode (for testing without hardware)
                return 2.5

            try:
                # Read raw voltage
                raw_voltage = self.chan.voltage

                # Add to buffer
                self.voltage_buffer.append(raw_voltage)

                # Limit buffer size
                if len(self.voltage_buffer) > self.filter_size:
                    self.voltage_buffer.pop(0)

                # Calculate moving average
                self.voltage = sum(self.voltage_buffer) / len(self.voltage_buffer)

                # Reset consecutive error count on successful read
                self.consecutive_errors = 0

                return self.voltage

            except Exception as e:
                # Track error
                self.error_count += 1
                self.consecutive_errors += 1
                self.last_error_time = time.time()

                if self.consecutive_errors == 1:
                    logger.error(f"I2C error reading ADS1115: {e}")
                elif self.consecutive_errors >= 10:
                    logger.critical(f"ADS1115 I2C communication failure: {self.consecutive_errors} consecutive errors")

                return self.voltage

    def read_raw(self) -> int:
        """
        Read raw ADC value

        Returns:
            Raw 16-bit ADC value
        """
        if self.chan is None:
            return 32768  # Midpoint

        try:
            return self.chan.value
        except Exception as e:
            logger.error(f"Error reading raw ADS1115: {e}")
            return 0

    def voltage_to_angle(
        self,
        voltage_min: float = 0.5,
        voltage_max: float = 4.5,
        angle_min: float = 0.0,
        angle_max: float = 180.0
    ) -> float:
        """
        Convert voltage to angle

        Args:
            voltage_min: Voltage at minimum angle
            voltage_max: Voltage at maximum angle
            angle_min: Minimum angle (degrees)
            angle_max: Maximum angle (degrees)

        Returns:
            Angle in degrees
        """
        voltage = self.read_voltage()

        # Clamp voltage
        voltage = max(voltage_min, min(voltage_max, voltage))

        # Linear mapping
        voltage_range = voltage_max - voltage_min
        angle_range = angle_max - angle_min

        angle = ((voltage - voltage_min) / voltage_range) * angle_range + angle_min

        return angle

    def get_error_status(self) -> dict:
        """
        Get I2C communication error status

        Returns:
            Dictionary with error information
        """
        with self.lock:
            return {
                'total_errors': self.error_count,
                'consecutive_errors': self.consecutive_errors,
                'last_error_time': self.last_error_time,
                'communication_ok': self.consecutive_errors < 5
            }

    def close(self):
        """Clean up resources"""
        logger.info("ADS1115 closed")


class SimulatedADS1115(ADS1115Reader):
    """
    Simulated ADS1115 for testing without hardware
    Returns voltage corresponding to simulated servo position
    """

    def __init__(self, *args, **kwargs):
        """Initialize simulated ADC"""
        # Don't call parent __init__ to avoid hardware initialization
        self.i2c_address = kwargs.get('i2c_address', 0x48)
        self.channel = kwargs.get('channel', 0)
        self.gain = kwargs.get('gain', 1)
        self.sample_rate = kwargs.get('sample_rate', 860)

        self.voltage = 2.5  # Midpoint (90°)
        self.lock = threading.Lock()

        self.voltage_buffer = []
        self.filter_size = 5

        logger.info(f"Simulated ADS1115 initialized: channel={self.channel}")

    def set_simulated_voltage(self, voltage: float):
        """
        Set simulated voltage (called by simulated servo)

        Args:
            voltage: Simulated voltage
        """
        with self.lock:
            self.voltage = voltage

    def read_voltage(self) -> float:
        """
        Read simulated voltage

        Returns:
            Simulated voltage in volts
        """
        with self.lock:
            return self.voltage
