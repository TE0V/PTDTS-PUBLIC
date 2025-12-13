#!/usr/bin/env python3
"""
Pan Motor Manual Control Test Script
Direct control of pan motor with real-time encoder feedback

This script bypasses all PID/controller logic and provides simple
manual control to test motor direction, encoder accuracy, and basic operation.

Controls:
  w/s - Increase/decrease PWM power
  a/d - Rotate left/right at current power
  SPACE - Stop motor
  r - Reset encoder to zero
  q - Quit

Author: Claude Code
"""

import sys
import time
import select
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

from src.utils.config_loader import load_config
import logging

# Minimal logging setup
logging.basicConfig(level=logging.WARNING)

def clear_input():
    """Clear any pending input"""
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            break
        sys.stdin.read(1)

def get_key_nonblocking():
    """Get keyboard input without blocking"""
    ready, _, _ = select.select([sys.stdin], [], [], 0.05)
    if ready:
        return sys.stdin.read(1).lower()
    return None

def main():
    print("="*70)
    print("PAN MOTOR MANUAL CONTROL TEST")
    print("="*70)

    # Load config
    print("\n[1/4] Loading configuration...")
    config = load_config()
    config.simulation_mode = False

    print(f"  Encoder counts per 360°: {config.pan_motor.encoder_counts_per_360}")
    print(f"  Encoder mode: {config.pan_motor.encoder_mode} (non-quadrature)")
    print(f"  Max PWM: {config.pan_motor.max_pwm}")

    # Initialize encoder
    print("\n[2/4] Initializing LS7366R encoder...")
    try:
        import spidev
        spi = spidev.SpiDev()
        spi.open(config.pan_motor.encoder_spi_bus, config.pan_motor.encoder_spi_device)
        spi.max_speed_hz = 50000  # 50 kHz
        spi.mode = 0

        # Configure LS7366R (non-quadrature mode)
        spi.xfer2([0x20])  # Clear counter
        time.sleep(0.1)
        spi.xfer2([0x88, 0x00])  # MDR0 = non-quadrature
        time.sleep(0.1)
        print("  ✓ Encoder initialized")
    except Exception as e:
        print(f"  ✗ Failed to initialize encoder: {e}")
        return

    # Initialize motor
    print("\n[3/4] Initializing DRV8874 motor driver...")
    try:
        from gpiozero import PWMOutputDevice

        motor_in1 = PWMOutputDevice(config.pan_motor.gpio_in1, frequency=1000)
        motor_in2 = PWMOutputDevice(config.pan_motor.gpio_in2, frequency=1000)

        # Stop motor
        motor_in1.value = 0
        motor_in2.value = 0
        print("  ✓ Motor driver initialized")
        print(f"    IN1: GPIO {config.pan_motor.gpio_in1}")
        print(f"    IN2: GPIO {config.pan_motor.gpio_in2}")
    except Exception as e:
        print(f"  ✗ Failed to initialize motor: {e}")
        spi.close()
        return

    # Helper functions
    def read_encoder():
        """Read encoder count and calculate angle"""
        data = spi.xfer2([0x60, 0, 0, 0, 0])  # Read counter
        count = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]
        if count & 0x80000000:
            count -= 0x100000000

        # Negate to match encoder.py behavior: CW rotation = count increase = angle increase
        angle = -(count / config.pan_motor.encoder_counts_per_360) * 360.0
        return count, angle

    def set_motor_pwm(pwm):
        """Set motor PWM (-1.0 to 1.0)"""
        pwm = max(-config.pan_motor.max_pwm, min(config.pan_motor.max_pwm, pwm))

        if pwm > 0:
            # Clockwise (IN1 PWM, IN2 GND)
            motor_in1.value = abs(pwm)
            motor_in2.value = 0
        elif pwm < 0:
            # Counter-clockwise (IN1 GND, IN2 PWM)
            motor_in1.value = 0
            motor_in2.value = abs(pwm)
        else:
            # Stop
            motor_in1.value = 0
            motor_in2.value = 0

    def stop_motor():
        """Stop motor immediately"""
        motor_in1.value = 0
        motor_in2.value = 0

    def reset_encoder():
        """Reset encoder to zero"""
        spi.xfer2([0x20])  # Clear counter
        time.sleep(0.05)

    # Setup terminal for non-blocking input
    import termios
    import tty

    old_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())

        print("\n[4/4] Starting manual control mode")
        print("="*70)
        print("\nCONTROLS:")
        print("  w/s   - Increase/decrease power (±10%)")
        print("  a/d   - Rotate LEFT/RIGHT at current power")
        print("  SPACE - STOP motor immediately")
        print("  r     - Reset encoder to zero")
        print("  q     - Quit")
        print("\nDIRECTION TEST:")
        print("  'a' (left)  should DECREASE encoder count (CCW rotation)")
        print("  'd' (right) should INCREASE encoder count (CW rotation)")
        print("="*70)

        current_pwm = 0.0
        power_level = 0.2  # Start at 20% power
        running = True
        last_count = 0
        last_time = time.time()

        clear_input()

        while running:
            # Get keyboard input
            key = get_key_nonblocking()

            if key == 'q':
                print("\n\nQuitting...")
                running = False
                break
            elif key == 'w':
                power_level = min(config.pan_motor.max_pwm, power_level + 0.1)
                print(f"\n>>> Power level: {power_level*100:.0f}%")
            elif key == 's':
                power_level = max(0.1, power_level - 0.1)
                print(f"\n>>> Power level: {power_level*100:.0f}%")
            elif key == 'a':
                current_pwm = -power_level
                print(f"\n>>> Rotating LEFT at {power_level*100:.0f}% power")
            elif key == 'd':
                current_pwm = power_level
                print(f"\n>>> Rotating RIGHT at {power_level*100:.0f}% power")
            elif key == ' ':
                current_pwm = 0
                print(f"\n>>> STOPPED")
            elif key == 'r':
                reset_encoder()
                print(f"\n>>> Encoder reset to zero")

            # Set motor
            set_motor_pwm(current_pwm)

            # Read encoder
            count, angle = read_encoder()

            # Calculate velocity
            current_time = time.time()
            dt = current_time - last_time
            if dt > 0.1:  # Update every 100ms
                count_diff = count - last_count
                # Negate to match encoder.py behavior: CW rotation = positive velocity
                velocity_dps = -(count_diff / config.pan_motor.encoder_counts_per_360) * 360.0 / dt
                last_count = count
                last_time = current_time

                # Display status
                direction = "→" if current_pwm > 0 else "←" if current_pwm < 0 else "●"
                print(f"\r{direction} PWM:{current_pwm:+.2f} | Count:{count:6d} | Angle:{angle:7.1f}° | Velocity:{velocity_dps:+6.1f}°/s  ", end='', flush=True)

            time.sleep(0.02)  # 50Hz update

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    finally:
        # Restore terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # Cleanup
        print("\n\nCleaning up...")
        stop_motor()
        motor_in1.close()
        motor_in2.close()
        spi.close()
        print("Done.")

if __name__ == "__main__":
    main()
