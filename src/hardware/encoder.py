"""
Encoder implementations for PTDTS
LS7366R hardware encoder and simulated encoder
"""

import time
import threading
import logging
import math
from typing import Optional
from .interfaces import EncoderInterface

logger = logging.getLogger(__name__)


class LS7366REncoder(EncoderInterface):
    """
    LS7366R quadrature encoder counter via SPI
    Hardware-based counting eliminates missed edges
    """

    # LS7366R Register addresses
    REG_MDR0 = 0x08
    REG_MDR1 = 0x10
    REG_DTR = 0x18
    REG_CNTR = 0x20
    REG_OTR = 0x28
    REG_STR = 0x30

    # Commands
    CMD_CLEAR_CNTR = 0x20
    CMD_CLEAR_STR = 0x30
    CMD_READ = 0x40
    CMD_WRITE = 0x80
    CMD_LOAD_CNTR = 0xE0
    CMD_LOAD_OTR = 0xE4

    def __init__(
        self,
        spi_bus: int = 0,
        spi_device: int = 0,
        counts_per_360: int = -8556,
        mode: int = 0  # Non-quadrature mode (all quadrature modes freeze the encoder)
    ):
        """
        Initialize LS7366R encoder

        Args:
            spi_bus: SPI bus number (0 or 1)
            spi_device: SPI device number (CS pin)
            counts_per_360: Encoder counts per 360° rotation
            mode: Encoder mode (0=non-quadrature, 1=x1, 2=x2, 4=x4)
                  NOTE: Currently only mode 0 (non-quadrature) works reliably.
                  This parameter is ignored and non-quadrature is always used.
        """
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.counts_per_360 = counts_per_360
        self.mode = mode

        # Velocity estimation
        self.last_count = 0
        self.last_time = time.time()
        self.velocity = 0.0
        self.velocity_alpha = 0.7  # Low-pass filter coefficient

        # Error tracking
        self.error_count = 0
        self.last_error_time = None
        self.consecutive_errors = 0

        self.lock = threading.Lock()

        # Initialize SPI
        try:
            import spidev
            self.spi = spidev.SpiDev()
            self.spi.open(spi_bus, spi_device)
            self.spi.max_speed_hz = 50000  # 50 kHz (reduced from 1 MHz for reliability)
            self.spi.mode = 0  # CPOL=0, CPHA=0

            logger.info(f"LS7366R encoder initialized on SPI {spi_bus}.{spi_device}")

            # Configure LS7366R
            self._configure()

        except Exception as e:
            logger.error(f"Failed to initialize LS7366R: {e}")
            raise

    def _configure(self):
        """Configure LS7366R registers"""
        # MDR0: Configure counting mode
        # NOTE: Using non-quadrature mode (0x00) as quadrature modes were freezing
        # Bit 7-6: 0b00 = non-quadrature mode
        # Bit 5-4: 0b00 = non-quadrature (counts single channel edges)
        # Bit 3-2: 0b00 = free-running count mode
        # Bit 1-0: 0b00 = disable index

        # Using non-quadrature mode regardless of 'mode' parameter
        # This is the only mode that works reliably on this hardware
        mdr0 = 0b00000000  # Non-quadrature mode

        # Clear counter first
        self.spi.xfer2([self.CMD_CLEAR_CNTR])
        time.sleep(0.1)  # Critical delay for LS7366R to process

        # Write MDR0 configuration only (MDR1 not needed for basic operation)
        self.spi.xfer2([self.CMD_WRITE | self.REG_MDR0, mdr0])
        time.sleep(0.1)  # Critical delay for LS7366R to process

        logger.debug(f"LS7366R configured: non-quadrature mode, MDR0=0x{mdr0:02X}")

    def get_count(self) -> int:
        """
        Get current encoder count from LS7366R

        Returns:
            32-bit signed encoder count
        """
        with self.lock:
            try:
                # Read CNTR register (4 bytes) - single transaction method
                # Send read command + 4 dummy bytes, receive 5 bytes total
                data = self.spi.xfer2([self.CMD_READ | self.REG_CNTR, 0, 0, 0, 0])

                # Convert to signed 32-bit integer (skip first byte which is echo)
                count = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]

                # Handle sign extension
                if count & 0x80000000:
                    count -= 0x100000000

                # Update velocity estimation
                current_time = time.time()
                dt = current_time - self.last_time

                if dt > 0.001:  # Avoid division by zero
                    # Calculate instantaneous velocity
                    count_diff = count - self.last_count
                    # Negate to match physical direction: CW rotation = positive velocity
                    inst_velocity = -(count_diff / self.counts_per_360) * 360.0 / dt

                    # Low-pass filter
                    self.velocity = self.velocity_alpha * self.velocity + (1 - self.velocity_alpha) * inst_velocity

                    self.last_count = count
                    self.last_time = current_time

                # Reset consecutive error count on successful read
                self.consecutive_errors = 0

                return count

            except Exception as e:
                # Track error
                self.error_count += 1
                self.consecutive_errors += 1
                self.last_error_time = time.time()

                if self.consecutive_errors == 1:
                    logger.error(f"SPI error reading LS7366R: {e}")
                elif self.consecutive_errors >= 10:
                    logger.critical(f"LS7366R SPI communication failure: {self.consecutive_errors} consecutive errors")

                return self.last_count

    def get_angle(self) -> float:
        """
        Get current angle in degrees (0-360)

        Returns:
            Angle in degrees
        """
        count = self.get_count()
        # Negate to match physical direction: CW rotation = count increase = angle increase
        angle = -(count / self.counts_per_360) * 360.0
        return angle % 360.0

    def get_velocity(self) -> float:
        """
        Get current angular velocity in degrees per second

        Returns:
            Angular velocity in deg/sec
        """
        with self.lock:
            return self.velocity

    def reset(self):
        """Reset encoder count to zero"""
        with self.lock:
            self.spi.xfer2([self.CMD_CLEAR_CNTR])
            self.last_count = 0
            self.velocity = 0.0
            logger.debug("LS7366R counter reset")

    def set_count(self, count: int):
        """
        Set encoder count to specific value (for state restoration).

        Args:
            count: Encoder count value to set
        """
        with self.lock:
            try:
                # Load count value into DTR (Data Transfer Register)
                # Write 4 bytes (MSB first)
                byte3 = (count >> 24) & 0xFF
                byte2 = (count >> 16) & 0xFF
                byte1 = (count >> 8) & 0xFF
                byte0 = count & 0xFF

                self.spi.xfer2([self.CMD_WRITE | self.REG_DTR, byte3, byte2, byte1, byte0])

                # Transfer DTR to CNTR
                self.spi.xfer2([self.CMD_LOAD_CNTR])

                self.last_count = count
                self.velocity = 0.0
                logger.debug(f"LS7366R counter set to {count}")

            except Exception as e:
                logger.error(f"Error setting LS7366R count: {e}")

    def get_error_status(self) -> dict:
        """
        Get SPI communication error status

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
        """Clean up SPI resources"""
        if hasattr(self, 'spi'):
            self.spi.close()
            logger.info("LS7366R encoder closed")


class SimulatedEncoder(EncoderInterface):
    """
    Simulated encoder for testing without hardware
    Integrates velocity commands to estimate position
    """

    def __init__(self, counts_per_360: int = -8556):
        """
        Initialize simulated encoder

        Args:
            counts_per_360: Encoder counts per 360° rotation
        """
        self.counts_per_360 = counts_per_360
        self.count = 0
        self.velocity = 0.0  # deg/sec
        self.last_update_time = time.time()
        self.lock = threading.Lock()

        logger.info("Simulated encoder initialized")

    def set_velocity(self, velocity_dps: float):
        """
        Set simulated velocity (called by simulated motor)

        Args:
            velocity_dps: Velocity in degrees per second
        """
        with self.lock:
            # Update position based on elapsed time
            current_time = time.time()
            dt = current_time - self.last_update_time

            # Integrate velocity to get position change
            angle_change = self.velocity * dt
            # Negate to match physical direction: positive angle change = positive count change
            count_change = int(-(angle_change / 360.0) * self.counts_per_360)
            self.count += count_change

            # Update velocity
            self.velocity = velocity_dps
            self.last_update_time = current_time

    def get_count(self) -> int:
        """
        Get current encoder count

        Returns:
            Simulated encoder count
        """
        with self.lock:
            # Update position based on current velocity
            current_time = time.time()
            dt = current_time - self.last_update_time

            angle_change = self.velocity * dt
            # Negate to match physical direction: positive angle change = positive count change
            count_change = int(-(angle_change / 360.0) * self.counts_per_360)
            self.count += count_change

            self.last_update_time = current_time

            return self.count

    def get_angle(self) -> float:
        """
        Get current angle in degrees (0-360)

        Returns:
            Simulated angle in degrees
        """
        count = self.get_count()
        # Negate to match physical direction: CW rotation = count increase = angle increase
        angle = -(count / self.counts_per_360) * 360.0
        return angle % 360.0

    def get_velocity(self) -> float:
        """
        Get current angular velocity in degrees per second

        Returns:
            Simulated velocity in deg/sec
        """
        with self.lock:
            return self.velocity

    def reset(self):
        """Reset encoder count to zero"""
        with self.lock:
            self.count = 0
            self.velocity = 0.0
            logger.debug("Simulated encoder reset")

    def set_count(self, count: int):
        """
        Set encoder count to specific value (for state restoration).

        Args:
            count: Encoder count value to set
        """
        with self.lock:
            self.count = count
            self.velocity = 0.0
            logger.debug(f"Simulated encoder count set to {count}")

    def close(self):
        """Clean up (no-op for simulation)"""
        logger.info("Simulated encoder closed")
