"""
System monitoring utilities
Tracks CPU temperature, memory usage, and other system metrics
"""

import os
import time
import logging
import threading
from typing import Optional

logger = logging.getLogger(__name__)


class SystemMonitor:
    """
    Monitor system health metrics
    CPU temperature, memory, disk, etc.
    """

    def __init__(self, update_interval_s: float = 5.0):
        """
        Initialize system monitor

        Args:
            update_interval_s: How often to update metrics in seconds
        """
        self.update_interval_s = update_interval_s

        # System metrics
        self.cpu_temp_c = 0.0
        self.cpu_usage_percent = 0.0
        self.memory_usage_percent = 0.0
        self.disk_usage_percent = 0.0
        self.throttled_status = 0

        # Monitoring thread
        self.running = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # Temperature thresholds (Raspberry Pi)
        self.temp_warning_c = 70.0
        self.temp_critical_c = 80.0

        # Check if thermal zone exists
        self.thermal_zone_path = "/sys/class/thermal/thermal_zone0/temp"
        self.has_thermal_zone = os.path.exists(self.thermal_zone_path)

        if not self.has_thermal_zone:
            logger.warning("CPU thermal zone not found, temperature monitoring disabled")

        logger.info("System monitor initialized")

    def start(self):
        """Start monitoring thread"""
        if not self.running:
            self.running = True
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
            logger.info("System monitoring started")

    def stop(self):
        """Stop monitoring thread"""
        if self.running:
            self.running = False
            if self.monitor_thread:
                self.monitor_thread.join(timeout=2.0)
            logger.info("System monitoring stopped")

    def _read_cpu_temp(self) -> float:
        """
        Read CPU temperature from thermal zone

        Returns:
            Temperature in Celsius
        """
        if not self.has_thermal_zone:
            return 0.0

        try:
            with open(self.thermal_zone_path, 'r') as f:
                temp_millidegrees = int(f.read().strip())
                temp_celsius = temp_millidegrees / 1000.0
                return temp_celsius
        except Exception as e:
            logger.error(f"Error reading CPU temperature: {e}")
            return 0.0

    def _read_cpu_usage(self) -> float:
        """
        Read CPU usage percentage

        Returns:
            CPU usage as percentage (0-100)
        """
        try:
            # Read /proc/stat for CPU usage
            with open('/proc/stat', 'r') as f:
                cpu_line = f.readline()
                # Parse: cpu  user nice system idle iowait irq softirq
                parts = cpu_line.split()
                if parts[0] == 'cpu':
                    idle = int(parts[4])
                    total = sum(int(x) for x in parts[1:8])

                    # Calculate usage (simplified, single sample)
                    if hasattr(self, '_last_total'):
                        idle_delta = idle - self._last_idle
                        total_delta = total - self._last_total
                        usage = 100.0 * (1.0 - idle_delta / total_delta) if total_delta > 0 else 0.0
                    else:
                        usage = 0.0

                    self._last_idle = idle
                    self._last_total = total
                    return usage
        except Exception as e:
            logger.error(f"Error reading CPU usage: {e}")

        return 0.0

    def _read_memory_usage(self) -> float:
        """
        Read memory usage percentage

        Returns:
            Memory usage as percentage (0-100)
        """
        try:
            with open('/proc/meminfo', 'r') as f:
                lines = f.readlines()

            mem_total = 0
            mem_available = 0

            for line in lines:
                if line.startswith('MemTotal:'):
                    mem_total = int(line.split()[1])
                elif line.startswith('MemAvailable:'):
                    mem_available = int(line.split()[1])

            if mem_total > 0:
                mem_used = mem_total - mem_available
                return (mem_used / mem_total) * 100.0
        except Exception as e:
            logger.error(f"Error reading memory usage: {e}")

        return 0.0

    def _read_disk_usage(self) -> float:
        """
        Read disk usage percentage for root filesystem

        Returns:
            Disk usage as percentage (0-100)
        """
        try:
            import shutil
            stat = shutil.disk_usage('/')
            return (stat.used / stat.total) * 100.0
        except Exception as e:
            logger.error(f"Error reading disk usage: {e}")
            return 0.0

    def _read_throttle_status(self) -> int:
        """
        Read throttle status (Raspberry Pi specific)

        Returns:
            Throttle status bits
        """
        try:
            # Use vcgencmd to get throttle status
            import subprocess
            result = subprocess.run(
                ['vcgencmd', 'get_throttled'],
                capture_output=True,
                text=True,
                timeout=1.0
            )

            if result.returncode == 0:
                # Parse output: throttled=0x50000
                output = result.stdout.strip()
                if 'throttled=' in output:
                    hex_value = output.split('=')[1]
                    return int(hex_value, 16)
        except Exception as e:
            # vcgencmd may not be available
            pass

        return 0

    def _monitor_loop(self):
        """Background monitoring loop"""
        while self.running:
            try:
                # Read all metrics
                cpu_temp = self._read_cpu_temp()
                cpu_usage = self._read_cpu_usage()
                mem_usage = self._read_memory_usage()
                disk_usage = self._read_disk_usage()
                throttled = self._read_throttle_status()

                # Update stored values
                with self.lock:
                    self.cpu_temp_c = cpu_temp
                    self.cpu_usage_percent = cpu_usage
                    self.memory_usage_percent = mem_usage
                    self.disk_usage_percent = disk_usage
                    self.throttled_status = throttled

                # Check for temperature warnings
                if cpu_temp > self.temp_critical_c:
                    logger.error(f"CRITICAL: CPU temperature {cpu_temp:.1f}°C exceeds critical threshold!")
                elif cpu_temp > self.temp_warning_c:
                    logger.warning(f"WARNING: CPU temperature {cpu_temp:.1f}°C exceeds warning threshold")

                # Check for throttling
                if throttled != 0:
                    if throttled & 0x1:
                        logger.warning("CPU under-voltage detected")
                    if throttled & 0x2:
                        logger.warning("CPU frequency capped")
                    if throttled & 0x4:
                        logger.warning("CPU currently throttled")
                    if throttled & 0x8:
                        logger.warning("CPU soft temperature limit active")

                # Sleep until next update
                time.sleep(self.update_interval_s)

            except Exception as e:
                logger.error(f"Error in system monitor loop: {e}")
                time.sleep(1.0)

    def get_cpu_temperature(self) -> float:
        """
        Get current CPU temperature

        Returns:
            Temperature in Celsius
        """
        with self.lock:
            return self.cpu_temp_c

    def get_cpu_usage(self) -> float:
        """
        Get current CPU usage

        Returns:
            Usage as percentage (0-100)
        """
        with self.lock:
            return self.cpu_usage_percent

    def get_memory_usage(self) -> float:
        """
        Get current memory usage

        Returns:
            Usage as percentage (0-100)
        """
        with self.lock:
            return self.memory_usage_percent

    def get_disk_usage(self) -> float:
        """
        Get current disk usage

        Returns:
            Usage as percentage (0-100)
        """
        with self.lock:
            return self.disk_usage_percent

    def is_throttled(self) -> bool:
        """
        Check if CPU is currently throttled

        Returns:
            True if throttled
        """
        with self.lock:
            return (self.throttled_status & 0x4) != 0

    def get_metrics(self) -> dict:
        """
        Get all system metrics

        Returns:
            Dictionary with system metrics
        """
        with self.lock:
            return {
                'cpu_temperature_c': self.cpu_temp_c,
                'cpu_usage_percent': self.cpu_usage_percent,
                'memory_usage_percent': self.memory_usage_percent,
                'disk_usage_percent': self.disk_usage_percent,
                'throttled': (self.throttled_status & 0x4) != 0,
                'throttle_status': self.throttled_status,
                'temp_warning': self.cpu_temp_c > self.temp_warning_c,
                'temp_critical': self.cpu_temp_c > self.temp_critical_c
            }
