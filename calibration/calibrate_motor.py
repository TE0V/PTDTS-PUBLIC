#!/usr/bin/env python3
"""
Motor Encoder Calibration Tool for PTDTS
Measures encoder counts per 360° rotation

Usage:
    python3 calibration/calibrate_motor.py

This tool will:
1. Reset encoder to zero
2. Prompt you to manually rotate the motor exactly 360° (one full rotation)
3. Measure the encoder counts
4. Calculate encoder_counts_per_360 for config.yaml
"""

import sys
import time
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.utils.config_loader import load_config
from src.utils.logger import setup_logging
from src.hardware.factory import HardwareFactory
import logging

logger = logging.getLogger(__name__)


def calibrate_motor_encoder():
    """
    Calibrate motor encoder counts per 360° rotation
    """
    print("\n" + "="*60)
    print("PTDTS Motor Encoder Calibration")
    print("="*60 + "\n")

    # Load config
    config = load_config()
    setup_logging(config)

    # Force simulation mode off for calibration
    config.simulation_mode = False

    print("Initializing hardware...")
    try:
        # Create encoder
        encoder = HardwareFactory.create_encoder(config)

        # Create motor
        motor = HardwareFactory.create_motor(config, encoder)

        print("Hardware initialized successfully.\n")

    except Exception as e:
        print(f"ERROR: Failed to initialize hardware: {e}")
        print("\nMake sure you're running on Raspberry Pi with hardware connected.")
        return

    try:
        # Reset encoder
        print("Resetting encoder to zero...")
        encoder.reset()
        time.sleep(0.5)
        initial_count = encoder.get_count()
        print(f"Initial encoder count: {initial_count}\n")

        print("CALIBRATION PROCEDURE:")
        print("-" * 60)
        print("1. Position a marker on the motor shaft (tape, marker, etc.)")
        print("2. Note the starting position of the marker")
        print("3. Press ENTER when ready to start")
        print("-" * 60)
        input()

        # Method 1: Passive rotation (user manually rotates)
        print("\nMETHOD 1: Manual Rotation (Recommended)")
        print("-" * 60)
        print("Manually rotate the motor shaft EXACTLY one full rotation (360°)")
        print("Make sure the marker returns to its starting position.")
        print("Press ENTER when you've completed one full rotation...")
        input()

        count_manual = encoder.get_count() - initial_count
        print(f"\nManual rotation result: {count_manual} counts\n")

        # Method 2: Motor-driven rotation (if user wants to verify)
        print("METHOD 2: Motor-Driven Rotation (Optional Verification)")
        print("-" * 60)
        print("This will drive the motor slowly for calibration.")
        print("Watch carefully and press CTRL+C when marker completes 360°")
        print("\nWould you like to perform motor-driven calibration? (y/n): ", end='')

        if input().lower().strip() == 'y':
            encoder.reset()
            time.sleep(0.5)

            print("\nStarting motor rotation at slow speed...")
            print("Press CTRL+C when the marker returns to start position!\n")

            # Slow rotation
            motor.set_pwm(0.15)  # Very slow speed

            try:
                while True:
                    current_count = encoder.get_count()
                    current_angle = (current_count / count_manual) * 360 if count_manual != 0 else 0
                    print(f"\rCurrent: {current_count:6d} counts  |  Est. angle: {current_angle:6.1f}°", end='', flush=True)
                    time.sleep(0.1)
            except KeyboardInterrupt:
                motor.stop()
                print("\n\nMotor stopped.")
                count_motor = encoder.get_count()
                print(f"Motor-driven result: {count_motor} counts\n")
        else:
            count_motor = None

        # Display results
        print("\n" + "="*60)
        print("CALIBRATION RESULTS")
        print("="*60)
        print(f"Manual rotation:      {count_manual:6d} counts per 360°")
        if count_motor is not None:
            print(f"Motor-driven:         {count_motor:6d} counts per 360°")
            avg_count = (count_manual + count_motor) // 2
            print(f"Average:              {avg_count:6d} counts per 360°")
            print(f"\nDifference: {abs(count_manual - count_motor)} counts ({abs(count_manual - count_motor)/abs(count_manual)*100:.1f}%)")
            recommended = avg_count
        else:
            recommended = count_manual

        print("\n" + "="*60)
        print("CONFIGURATION UPDATE")
        print("="*60)
        print(f"\nUpdate config/config.yaml with:")
        print(f"\n  encoder_counts_per_360: {recommended}")

        # Additional info
        print("\n" + "="*60)
        print("TECHNICAL DETAILS")
        print("="*60)
        print(f"Gear ratio: {config.pan_motor.gear_ratio}:1")

        # Display encoder mode with proper description
        if config.pan_motor.encoder_mode == 0:
            print("Encoder mode: Non-quadrature (single channel)")
        elif config.pan_motor.encoder_mode == 1:
            print("Encoder mode: 1x Quadrature")
        elif config.pan_motor.encoder_mode == 2:
            print("Encoder mode: 2x Quadrature")
        elif config.pan_motor.encoder_mode == 4:
            print("Encoder mode: 4x Quadrature")
        else:
            print(f"Encoder mode: Unknown ({config.pan_motor.encoder_mode})")

        # Expected calculation - handle each mode correctly
        # Non-quadrature mode counts 1 edge per CPR, quadrature modes count more
        if config.pan_motor.encoder_mode == 0:
            # Non-quadrature: 64 CPR * 135:1 gear ratio = 8640 counts/360°
            expected = -64 * config.pan_motor.gear_ratio
        elif config.pan_motor.encoder_mode == 1:
            # x1 quadrature: 64 CPR * 2 * 135:1 = 17280 counts/360°
            expected = -64 * 2 * config.pan_motor.gear_ratio
        elif config.pan_motor.encoder_mode == 2:
            # x2 quadrature: 64 CPR * 2 * 135:1 = 17280 counts/360°
            expected = -64 * 2 * config.pan_motor.gear_ratio
        elif config.pan_motor.encoder_mode == 4:
            # x4 quadrature: 64 CPR * 4 * 135:1 = 34560 counts/360°
            expected = -64 * 4 * config.pan_motor.gear_ratio
        else:
            expected = None

        if expected is not None:
            print(f"Expected value (theoretical): {expected:.0f} counts")
            if recommended != 0 and expected != 0:
                print(f"Measured vs Expected: {abs(recommended/expected - 1)*100:.1f}% difference")
        else:
            print("Expected value: Unknown (unsupported encoder mode)")

        print("\n" + "="*60)

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user.")
    except Exception as e:
        logger.error(f"Calibration error: {e}", exc_info=True)
    finally:
        # Cleanup
        print("\nCleaning up...")
        motor.stop()
        print("Done.\n")


if __name__ == "__main__":
    try:
        calibrate_motor_encoder()
    except KeyboardInterrupt:
        print("\n\nExiting...")
        sys.exit(0)
