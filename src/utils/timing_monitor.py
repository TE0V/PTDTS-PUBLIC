"""
Control loop timing monitoring utilities
Tracks loop frequency, jitter, and overruns
"""

import time
import threading
from typing import Optional
from collections import deque


class LoopTimingMonitor:
    """
    Monitor control loop timing performance
    Tracks frequency, jitter, and overruns
    """

    def __init__(
        self,
        target_rate_hz: float,
        window_size: int = 100
    ):
        """
        Initialize timing monitor

        Args:
            target_rate_hz: Target loop rate in Hz
            window_size: Number of loop iterations to average
        """
        self.target_rate_hz = target_rate_hz
        self.target_period_s = 1.0 / target_rate_hz
        self.window_size = window_size

        # Timing statistics
        self.loop_times: deque = deque(maxlen=window_size)
        self.actual_rate_hz = 0.0
        self.jitter_ms = 0.0
        self.max_loop_time_ms = 0.0
        self.overrun_count = 0
        self.total_iterations = 0

        # For measuring individual loop execution time
        self.last_loop_start = None
        self.last_iteration_time = time.time()

        self.lock = threading.Lock()

    def start_iteration(self):
        """
        Mark the start of a loop iteration
        Call this at the beginning of each loop
        """
        self.last_loop_start = time.time()

    def end_iteration(self):
        """
        Mark the end of a loop iteration
        Call this at the end of each loop
        """
        if self.last_loop_start is None:
            return

        with self.lock:
            current_time = time.time()

            # Calculate loop execution time
            loop_execution_time = current_time - self.last_loop_start
            loop_execution_ms = loop_execution_time * 1000

            # Calculate period since last iteration
            period = current_time - self.last_iteration_time
            self.last_iteration_time = current_time

            # Store timing data
            self.loop_times.append(loop_execution_ms)
            self.total_iterations += 1

            # Update max loop time
            if loop_execution_ms > self.max_loop_time_ms:
                self.max_loop_time_ms = loop_execution_ms

            # Check for overrun
            if loop_execution_time > self.target_period_s:
                self.overrun_count += 1

            # Calculate actual rate (from period between iterations)
            if len(self.loop_times) >= 2:
                # Calculate average period
                avg_period = sum(self.loop_times) / len(self.loop_times) / 1000.0
                if avg_period > 0:
                    # Note: This is execution time, not period
                    # For actual rate, we should track inter-iteration time
                    pass

            # Calculate jitter (standard deviation of loop times)
            if len(self.loop_times) >= 2:
                mean_time = sum(self.loop_times) / len(self.loop_times)
                variance = sum((t - mean_time) ** 2 for t in self.loop_times) / len(self.loop_times)
                self.jitter_ms = variance ** 0.5

    def record_loop_period(self, period_s: float):
        """
        Record the actual period between loop iterations
        This gives a more accurate measurement of actual loop rate

        Args:
            period_s: Time since last loop iteration in seconds
        """
        with self.lock:
            if period_s > 0:
                self.actual_rate_hz = 1.0 / period_s

    def get_actual_rate_hz(self) -> float:
        """
        Get actual measured loop rate

        Returns:
            Actual rate in Hz
        """
        with self.lock:
            return self.actual_rate_hz

    def get_jitter_ms(self) -> float:
        """
        Get loop timing jitter

        Returns:
            Jitter in milliseconds (standard deviation)
        """
        with self.lock:
            return self.jitter_ms

    def get_max_loop_time_ms(self) -> float:
        """
        Get maximum loop execution time

        Returns:
            Max execution time in milliseconds
        """
        with self.lock:
            return self.max_loop_time_ms

    def get_overrun_count(self) -> int:
        """
        Get number of loop overruns

        Returns:
            Count of iterations that exceeded target period
        """
        with self.lock:
            return self.overrun_count

    def get_overrun_rate(self) -> float:
        """
        Get loop overrun rate as percentage

        Returns:
            Percentage of iterations that overran (0-100)
        """
        with self.lock:
            if self.total_iterations > 0:
                return (self.overrun_count / self.total_iterations) * 100.0
            return 0.0

    def get_avg_loop_time_ms(self) -> float:
        """
        Get average loop execution time

        Returns:
            Average execution time in milliseconds
        """
        with self.lock:
            if len(self.loop_times) > 0:
                return sum(self.loop_times) / len(self.loop_times)
            return 0.0

    def get_metrics(self) -> dict:
        """
        Get comprehensive timing metrics

        Returns:
            Dictionary with timing information
        """
        with self.lock:
            rate_deviation = 0.0
            if self.target_rate_hz > 0 and self.actual_rate_hz > 0:
                rate_deviation = ((self.actual_rate_hz - self.target_rate_hz) / self.target_rate_hz) * 100.0

            # Calculate average loop time directly (avoid recursive lock)
            avg_loop_time_ms = 0.0
            if len(self.loop_times) > 0:
                avg_loop_time_ms = sum(self.loop_times) / len(self.loop_times)

            # Calculate overrun rate directly (avoid recursive lock)
            overrun_rate_percent = 0.0
            if self.total_iterations > 0:
                overrun_rate_percent = (self.overrun_count / self.total_iterations) * 100.0

            return {
                'target_rate_hz': self.target_rate_hz,
                'actual_rate_hz': self.actual_rate_hz,
                'rate_deviation_percent': rate_deviation,
                'avg_loop_time_ms': avg_loop_time_ms,
                'max_loop_time_ms': self.max_loop_time_ms,
                'max_iteration_ms': self.max_loop_time_ms,  # Alias for compatibility
                'jitter_ms': self.jitter_ms,
                'overrun_count': self.overrun_count,
                'overrun_rate_percent': overrun_rate_percent,
                'total_iterations': self.total_iterations
            }

    def reset(self):
        """Reset all timing statistics"""
        with self.lock:
            self.loop_times.clear()
            self.actual_rate_hz = 0.0
            self.jitter_ms = 0.0
            self.max_loop_time_ms = 0.0
            self.overrun_count = 0
            self.total_iterations = 0
