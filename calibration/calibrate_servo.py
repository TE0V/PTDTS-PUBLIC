#!/usr/bin/env python3
"""
Servo Feedback Calibration Tool for PTDTS
Calibrates ADS1115 analog feedback for Axon servo (optional closed-loop mode)

Usage:
    python3 calibration/calibrate_servo.py

This tool will:
1. Move servo through its full range (0° to 180°)
2. Measure ADS1115 voltage at each position
3. Calculate voltage_min and voltage_max
4. Verify linearity of feedback
5. Update config.yaml with calibrated values
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


def calibrate_servo_feedback():
    """
    Calibrate servo analog feedback from ADS1115
    """
    print("\n" + "="*60)
    print("PTDTS Servo Feedback Calibration")
    print("="*60 + "\n")

    # Load config
    config = load_config()
    setup_logging(config)

    # Check if closed-loop is enabled
    if not config.tilt_servo.closed_loop:
        print("NOTE: Servo closed-loop mode is currently DISABLED in config.")
        print("This calibration is optional unless you plan to enable closed-loop.\n")

    print("This calibration requires:")
    print("  - Axon servo connected with analog feedback wire")
    print("  - ADS1115 ADC connected (I2C)")
    print("  - Servo able to move through full range\n")

    input("Press ENTER to continue...")

    # Force simulation mode off
    config.simulation_mode = False

    print("\nInitializing hardware...")
    try:
        # Create servo
        servo = HardwareFactory.create_servo(config)

        # Try to import ADS1115
        try:
            from src.hardware.ads1115 import ADS1115
            adc = ADS1115(
                address=config.tilt_servo.ads1115_address,
                channel=config.tilt_servo.ads1115_channel
            )
            has_adc = True
        except Exception as e:
            print(f"\nWARNING: Could not initialize ADS1115: {e}")
            print("Proceeding with servo movement only (no feedback measurement).")
            has_adc = False

        print("Hardware initialized.\n")

    except Exception as e:
        print(f"ERROR: Failed to initialize hardware: {e}")
        print("\nMake sure you're running on Raspberry Pi with hardware connected.")
        return

    try:
        # Calibration points
        angles = [0, 30, 60, 90, 120, 150, 180]
        measurements = []

        print("CALIBRATION PROCEDURE:")
        print("-" * 60)
        print("The servo will move to each angle and measure feedback voltage.")
        print("Please ensure the servo can move freely without obstruction.")
        print("-" * 60)
        print()

        input("Press ENTER to start calibration...")

        for angle in angles:
            print(f"\nMoving to {angle:3d}°...", end='', flush=True)
            servo.set_angle(angle)
            time.sleep(1.0)  # Wait for servo to settle

            if has_adc:
                voltage = adc.read_voltage()
                measurements.append({'angle': angle, 'voltage': voltage})
                print(f" Voltage: {voltage:.3f}V")
            else:
                print(" (No ADC feedback)")

        # Return to center
        print("\nReturning to center (90°)...")
        servo.set_angle(90.0)

        if has_adc and len(measurements) > 0:
            # Display results
            print("\n" + "="*60)
            print("CALIBRATION RESULTS")
            print("="*60)
            print(f"\n{'Angle (°)':>10} | {'Voltage (V)':>12}")
            print("-" * 26)
            for m in measurements:
                print(f"{m['angle']:>10} | {m['voltage']:>12.3f}")

            # Calculate min/max
            voltages = [m['voltage'] for m in measurements]
            v_min = min(voltages)
            v_max = max(voltages)
            v_range = v_max - v_min

            print("\n" + "="*60)
            print("SUMMARY")
            print("="*60)
            print(f"Minimum voltage (0°):   {v_min:.3f}V")
            print(f"Maximum voltage (180°): {v_max:.3f}V")
            print(f"Voltage range:          {v_range:.3f}V")
            print(f"Voltage per degree:     {v_range/180:.4f}V/°")

            # Check linearity
            print("\n" + "="*60)
            print("LINEARITY CHECK")
            print("="*60)

            # Calculate expected voltages assuming linear
            errors = []
            for m in measurements:
                expected_v = v_min + (m['angle'] / 180.0) * v_range
                error = abs(m['voltage'] - expected_v)
                error_pct = (error / v_range) * 100
                errors.append(error_pct)
                print(f"{m['angle']:3d}°: Measured {m['voltage']:.3f}V, Expected {expected_v:.3f}V, Error {error_pct:.2f}%")

            max_error = max(errors)
            avg_error = sum(errors) / len(errors)

            print(f"\nMaximum error: {max_error:.2f}%")
            print(f"Average error: {avg_error:.2f}%")

            if max_error < 5.0:
                print("Linearity: EXCELLENT (< 5% error)")
            elif max_error < 10.0:
                print("Linearity: GOOD (< 10% error)")
            else:
                print("Linearity: POOR (> 10% error) - Check wiring and servo")

            # Configuration update
            print("\n" + "="*60)
            print("CONFIGURATION UPDATE")
            print("="*60)
            print(f"\nUpdate config/config.yaml in tilt_servo section:")
            print(f"\n  voltage_min: {v_min:.2f}")
            print(f"  voltage_max: {v_max:.2f}")
            print(f"  closed_loop: true  # Enable if you want closed-loop control")

        else:
            print("\nNo feedback measurements available.")
            print("Servo movement test completed successfully.")

        print("\n" + "="*60)

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted.")
    except Exception as e:
        logger.error(f"Calibration error: {e}", exc_info=True)
    finally:
        # Return to center position
        servo.set_angle(90.0)
        print("\nDone.\n")


if __name__ == "__main__":
    try:
        calibrate_servo_feedback()
    except KeyboardInterrupt:
        print("\n\nExiting...")
        sys.exit(0)
