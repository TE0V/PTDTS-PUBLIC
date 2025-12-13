#!/usr/bin/env python3
"""
Test script for Axon MAX MK2 servo with analog position feedback
Tests smooth S-curve motion and verifies actual vs commanded position

Hardware:
- Axon MAX MK2 servo on GPIO 18 (Pin 12 - hardware PWM)
- ADS1115 ADC on I2C (address 0x48, channel A0)
- Analog position feedback wire from servo to ADC

Usage:
    sudo python3 test_tilt_servo.py

Controls:
    w/s     - Increase/decrease target angle by 5°
    a/d     - Quick jump to 0° / 90° (test limits)
    h       - Home to 45° (center)
    0-9     - Jump to specific angles (0° to 90° in 10° steps)
    space   - Print current status
    r       - Reset and re-home
    q       - Quit

Features:
- Shows commanded vs actual position (from analog feedback)
- Measures slew rate and motion time
- Verifies 0-90° limits
- Displays smooth S-curve motion
"""

import sys
import time
import logging
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from utils.config_loader import load_config
from hardware.servo import AxonServo
from hardware.ads1115 import ADS1115Reader

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Disable debug spam from gpiozero
logging.getLogger('gpiozero').setLevel(logging.WARNING)


class ServoTester:
    """Test harness for Axon servo with analog feedback"""

    def __init__(self):
        """Initialize servo and ADC"""
        # Load config
        self.config = load_config()

        # Initialize servo (open-loop mode with smooth motion)
        logger.info("Initializing Axon MAX MK2 servo...")
        self.servo = AxonServo(
            gpio_pwm=self.config.tilt_servo.gpio_pwm,
            pwm_frequency=self.config.tilt_servo.pwm_frequency,
            pwm_min=self.config.tilt_servo.pwm_min,
            pwm_max=self.config.tilt_servo.pwm_max,
            min_angle=self.config.tilt_servo.min_angle,
            max_angle=self.config.tilt_servo.max_angle,
            slew_rate_dps=self.config.tilt_servo.slew_rate_dps,
            acceleration_profile=getattr(self.config.tilt_servo, 'acceleration_profile', 's_curve')
        )

        # Initialize ADS1115 for position feedback
        logger.info("Initializing ADS1115 ADC for position feedback...")
        try:
            self.adc = ADS1115Reader(
                i2c_address=self.config.tilt_servo.ads1115_address,
                channel=self.config.tilt_servo.ads1115_channel,
                gain=1,  # ±4.096V range
                sample_rate=860  # Max sample rate
            )
            self.adc_available = True
            logger.info("✓ ADS1115 initialized successfully")
        except Exception as e:
            logger.warning(f"ADS1115 not available: {e}")
            logger.warning("Running without analog feedback monitoring")
            self.adc = None
            self.adc_available = False

        # Motion tracking
        self.target_angle = self.servo.get_angle()
        self.motion_start_time = None
        self.motion_start_angle = None

        logger.info("\n" + "="*60)
        logger.info("Axon MAX MK2 Servo Test - Ready!")
        logger.info("="*60)
        logger.info(f"Config: {self.config.tilt_servo.min_angle}° to {self.config.tilt_servo.max_angle}°")
        logger.info(f"Slew Rate: {self.config.tilt_servo.slew_rate_dps}°/s")
        logger.info(f"Profile: {getattr(self.config.tilt_servo, 'acceleration_profile', 's_curve')}")
        logger.info(f"PWM Range: {self.config.tilt_servo.pwm_min}-{self.config.tilt_servo.pwm_max}µs @ {self.config.tilt_servo.pwm_frequency}Hz")
        logger.info(f"Analog Feedback: {'ENABLED' if self.adc_available else 'DISABLED'}")
        logger.info("="*60 + "\n")

    def read_actual_angle(self) -> float:
        """
        Read actual servo position from analog feedback

        Returns:
            Actual angle in degrees (or 0.0 if ADC not available)
        """
        if not self.adc_available:
            return 0.0

        try:
            angle = self.adc.voltage_to_angle(
                voltage_min=self.config.tilt_servo.voltage_min,
                voltage_max=self.config.tilt_servo.voltage_max,
                angle_min=self.config.tilt_servo.min_angle,
                angle_max=self.config.tilt_servo.max_angle
            )
            return angle
        except Exception as e:
            logger.error(f"Error reading ADC: {e}")
            return 0.0

    def read_voltage(self) -> float:
        """
        Read raw voltage from ADC

        Returns:
            Voltage in volts (or 0.0 if ADC not available)
        """
        if not self.adc_available:
            return 0.0

        try:
            return self.adc.read_voltage()
        except Exception as e:
            logger.error(f"Error reading voltage: {e}")
            return 0.0

    def set_target(self, angle: float):
        """
        Set target angle and start motion tracking

        Args:
            angle: Target angle in degrees
        """
        # Clamp to limits
        angle = max(self.config.tilt_servo.min_angle,
                   min(self.config.tilt_servo.max_angle, angle))

        # Track motion
        self.motion_start_angle = self.servo.get_angle()
        self.motion_start_time = time.time()
        self.target_angle = angle

        # Command servo
        self.servo.set_angle(angle)

        logger.info(f"→ Target set: {angle:.1f}° (from {self.motion_start_angle:.1f}°)")

    def print_status(self):
        """Print current servo status"""
        commanded = self.servo.get_angle()
        actual = self.read_actual_angle()
        voltage = self.read_voltage()

        # Calculate error
        if self.adc_available:
            error = commanded - actual
            error_str = f"Error: {error:+6.2f}°"
        else:
            error_str = "Error: N/A"

        # Motion progress
        if self.motion_start_time:
            elapsed = time.time() - self.motion_start_time
            distance = abs(self.target_angle - self.motion_start_angle)
            if distance > 0.1:
                progress = abs(commanded - self.motion_start_angle) / distance * 100.0
            else:
                progress = 100.0
        else:
            elapsed = 0.0
            progress = 100.0

        # Check if arrived
        at_target = abs(commanded - self.target_angle) < 0.1

        print(f"\r"
              f"Target: {self.target_angle:5.1f}° | "
              f"Cmd: {commanded:5.1f}° | "
              f"Actual: {actual:5.1f}° ({voltage:4.2f}V) | "
              f"{error_str} | "
              f"Progress: {progress:3.0f}% | "
              f"Time: {elapsed:4.1f}s | "
              f"{'✓' if at_target else '→'}",
              end='', flush=True)

    def run_automated_test(self):
        """Run automated test sequence"""
        logger.info("\n" + "="*60)
        logger.info("AUTOMATED TEST SEQUENCE")
        logger.info("="*60 + "\n")

        test_positions = [
            (45, "Center position"),
            (0, "Minimum angle (looking down)"),
            (90, "Maximum angle (horizontal)"),
            (45, "Return to center"),
            (30, "Test 30°"),
            (60, "Test 60°"),
            (45, "Final center")
        ]

        for angle, description in test_positions:
            logger.info(f"\nTest: {description} → {angle}°")
            self.set_target(angle)

            # Wait for motion to complete
            start_time = time.time()
            while abs(self.servo.get_angle() - angle) > 0.1:
                self.print_status()
                time.sleep(0.05)

                # Timeout after 15 seconds
                if time.time() - start_time > 15.0:
                    logger.warning("\nMotion timeout!")
                    break

            # Final position
            print()  # New line after progress
            time.sleep(0.5)  # Let it settle

            commanded = self.servo.get_angle()
            actual = self.read_actual_angle()
            voltage = self.read_voltage()
            elapsed = time.time() - start_time

            logger.info(f"✓ Arrived: Cmd={commanded:.2f}° Actual={actual:.2f}° ({voltage:.3f}V) Time={elapsed:.2f}s")

            time.sleep(1.0)  # Pause between tests

        logger.info("\n" + "="*60)
        logger.info("AUTOMATED TEST COMPLETE")
        logger.info("="*60 + "\n")

    def run_interactive(self):
        """Run interactive keyboard control"""
        import termios
        import tty

        logger.info("\n" + "="*60)
        logger.info("INTERACTIVE CONTROL MODE")
        logger.info("="*60)
        logger.info("Controls:")
        logger.info("  w/s     - Increase/decrease target by 5°")
        logger.info("  a/d     - Jump to 0° / 90°")
        logger.info("  h       - Home to 45°")
        logger.info("  0-9     - Jump to 0°, 10°, 20°, ... 90°")
        logger.info("  space   - Print detailed status")
        logger.info("  r       - Reset and re-home")
        logger.info("  q       - Quit")
        logger.info("="*60 + "\n")

        # Setup terminal for non-blocking input
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)

            while True:
                # Update display
                self.print_status()

                # Check for input (non-blocking)
                import select
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)

                    print()  # New line after status

                    if key == 'q':
                        logger.info("Quitting...")
                        break
                    elif key == 'w':
                        new_target = min(self.target_angle + 5.0, self.config.tilt_servo.max_angle)
                        self.set_target(new_target)
                    elif key == 's':
                        new_target = max(self.target_angle - 5.0, self.config.tilt_servo.min_angle)
                        self.set_target(new_target)
                    elif key == 'a':
                        self.set_target(0.0)
                    elif key == 'd':
                        self.set_target(90.0)
                    elif key == 'h':
                        self.set_target(45.0)
                    elif key == 'r':
                        logger.info("Resetting to home position...")
                        self.set_target(45.0)
                    elif key == ' ':
                        print()
                        logger.info("="*60)
                        logger.info(f"Target Angle:     {self.target_angle:.2f}°")
                        logger.info(f"Commanded Angle:  {self.servo.get_angle():.2f}°")
                        if self.adc_available:
                            logger.info(f"Actual Angle:     {self.read_actual_angle():.2f}°")
                            logger.info(f"Feedback Voltage: {self.read_voltage():.3f}V")
                            logger.info(f"Position Error:   {self.servo.get_angle() - self.read_actual_angle():+.2f}°")
                        logger.info(f"Slew Rate:        {self.config.tilt_servo.slew_rate_dps}°/s")
                        logger.info("="*60)
                    elif key.isdigit():
                        angle = int(key) * 10.0
                        if angle <= 90.0:
                            self.set_target(angle)

                time.sleep(0.02)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print()  # Clean line after exit

    def close(self):
        """Clean up resources"""
        logger.info("\nCleaning up...")
        if self.servo:
            self.servo.close()
        if self.adc:
            self.adc.close()
        logger.info("Done!")


def main():
    """Main entry point"""
    tester = None

    try:
        tester = ServoTester()

        # Ask user for test mode
        print("\nTest Mode:")
        print("  1 - Automated test sequence")
        print("  2 - Interactive manual control")
        print("  3 - Both (automated first, then interactive)")
        print()

        # Get input with timeout/default
        import select
        print("Select mode (1/2/3) [default: 2]: ", end='', flush=True)

        # Wait 5 seconds for input
        ready, _, _ = select.select([sys.stdin], [], [], 5.0)
        if ready:
            choice = sys.stdin.readline().strip()
        else:
            choice = '2'
            print('2 (timeout, using default)')

        if not choice:
            choice = '2'

        if choice == '1':
            tester.run_automated_test()
        elif choice == '2':
            tester.run_interactive()
        elif choice == '3':
            tester.run_automated_test()
            input("\nPress Enter to start interactive mode...")
            tester.run_interactive()
        else:
            logger.error(f"Invalid choice: {choice}")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        if tester:
            tester.close()


if __name__ == '__main__':
    main()
