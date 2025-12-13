#!/usr/bin/env python3
"""
Encoder Direction Verification Script

This script verifies that the encoder hardware and software direction match correctly:
- CW rotation (positive PWM) → encoder counts INCREASE → angle increases (via negation in encoder.py)
- CCW rotation (negative PWM) → encoder counts DECREASE → angle decreases (via negation in encoder.py)

The angle calculation in encoder.py negates the raw counts to match expected direction.
This test verifies the entire chain works correctly.
"""

import sys
import time
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from src.utils.config_loader import load_config
from src.hardware.factory import HardwareFactory

def main():
    print("=" * 70)
    print("ENCODER DIRECTION VERIFICATION TEST")
    print("=" * 70)
    print()
    print("This test verifies encoder direction matches motor direction.")
    print()
    print("EXPECTED BEHAVIOR (with encoder.py negation):")
    print("  - Motor turns CLOCKWISE (when viewed from encoder side)")
    print("  - Raw encoder counts INCREASE (hardware behavior)")
    print("  - Calculated angle INCREASES (via negation in software)")
    print()
    print("This test verifies the complete encoder→angle pipeline works correctly.")
    print("=" * 70)
    print()

    # Load configuration
    config = load_config()
    config.simulation_mode = False  # Force real hardware
    print(f"Current config: encoder_counts_per_360 = {config.pan_motor.encoder_counts_per_360}")
    print()

    # Create hardware
    encoder = HardwareFactory.create_encoder(config)
    motor = HardwareFactory.create_motor(config)

    try:
        print("STEP 1: Manual rotation test")
        print("-" * 70)
        print("Starting encoder count:", encoder.get_count())
        print("Starting angle:", f"{encoder.get_angle():.2f}°")
        print()
        print("INSTRUCTIONS:")
        print("1. Manually rotate the motor shaft CLOCKWISE (when viewed from encoder)")
        print("2. Watch the counts and angle below")
        print("3. Press Ctrl+C when done")
        print()
        print("Monitoring... (Ctrl+C to continue)")
        print()

        start_count = encoder.get_count()
        start_angle = encoder.get_angle()

        while True:
            count = encoder.get_count()
            angle = encoder.get_angle()
            count_change = count - start_count
            angle_change = angle - start_angle

            # Determine direction (counts SHOULD increase for CW with encoder.py negation)
            if count_change > 10:
                direction = "✓ INCREASING (correct - encoder.py negates for angle)"
            elif count_change < -10:
                direction = "✗ DECREASING (WRONG - check wiring or encoder.py)"
            else:
                direction = "  No significant movement yet"

            print(f"\rCount: {count:6d} (Δ{count_change:+5d})  |  "
                  f"Angle: {angle:7.2f}° (Δ{angle_change:+6.2f}°)  |  {direction}",
                  end='', flush=True)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n")
        print("-" * 70)

        final_count = encoder.get_count()
        final_angle = encoder.get_angle()
        total_count_change = final_count - start_count
        total_angle_change = final_angle - start_angle

        print()
        print("RESULTS:")
        print(f"  Total count change: {total_count_change:+d}")
        print(f"  Total angle change: {total_angle_change:+.2f}°")
        print()

        if abs(total_count_change) < 50:
            print("⚠️  WARNING: Not enough rotation detected.")
            print("   Please rotate at least 90 degrees for accurate test.")
        elif total_count_change > 0:
            print("✓ PASS: Encoder counts INCREASED during clockwise rotation")
            print("  This is CORRECT (encoder.py negates this to positive angle).")
            print()
            if total_angle_change > 0:
                print("✓ PASS: Angle INCREASED as expected")
                print("  Complete encoder pipeline working correctly!")
            else:
                print("✗ FAIL: Angle should have increased but decreased")
                print("  BUG in encoder.py angle calculation - check negation logic")
        else:
            print("✗ FAIL: Encoder counts DECREASED during clockwise rotation")
            print("  This is WRONG - encoder wiring may be backwards!")
            print()
            print("TROUBLESHOOTING:")
            print("  1. Check encoder A/B wiring")
            print("  2. Swap A and B channels")
            print("  3. Check encoder.py negation logic")
            print()

        print()
        print("STEP 2: Motor-driven test (optional)")
        print("-" * 70)
        response = input("Test with motor power? This will rotate the motor. (y/n): ")

        if response.lower() == 'y':
            print()
            print("Applying positive PWM (+0.40)...")
            print("Motor should turn CLOCKWISE")
            print("Watching for 3 seconds...")
            print()

            start_count = encoder.get_count()
            start_angle = encoder.get_angle()

            motor.set_pwm(0.40)
            time.sleep(3.0)
            motor.set_pwm(0.0)

            final_count = encoder.get_count()
            final_angle = encoder.get_angle()
            motor_count_change = final_count - start_count
            motor_angle_change = final_angle - start_angle

            print()
            print("MOTOR TEST RESULTS:")
            print(f"  Count change: {motor_count_change:+d}")
            print(f"  Angle change: {motor_angle_change:+.2f}°")
            print()

            if motor_count_change < 0:
                print("✓ PASS: Positive PWM → counts decreased (correct)")
            else:
                print("✗ FAIL: Positive PWM → counts increased (INVERTED!)")

        print()
        print("=" * 70)
        print("Test complete!")
        print("=" * 70)

    finally:
        motor.set_pwm(0.0)
        motor.close()
        encoder.close()

if __name__ == "__main__":
    main()
