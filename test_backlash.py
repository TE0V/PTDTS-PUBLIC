#!/usr/bin/env python3
"""
Backlash Diagnostic Script

This script tests for mechanical backlash in the pan motor gear system.
Backlash is "play" or "slop" in the gears - when the motor reverses direction,
there's a small gap before the gears engage and the output shaft moves.

With a 135:1 gear ratio, even small backlash in the gearbox compounds significantly.
Typical backlash in low-cost gear systems: 1-5 degrees of output shaft rotation.

This can cause oscillation because:
- Motor moves CW to position A
- System detects target slightly left
- Motor reverses to CCW
- First 2-3° of motor rotation absorbed by backlash (no movement)
- PID sees "motor not responding" and increases output
- Backlash gap closes, full torque applied, overshoots
- Repeat indefinitely
"""

import sys
import time
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from src.utils.config_loader import load_config
from src.control.pan_controller import PanController
from src.hardware.factory import HardwareFactory
import numpy as np

def main():
    print("=" * 70)
    print("BACKLASH DIAGNOSTIC TEST")
    print("=" * 70)
    print()
    print("This script measures mechanical backlash in the pan motor gear system.")
    print()
    print("WHAT IS BACKLASH?")
    print("  Backlash is the 'play' or 'slop' in gears when reversing direction.")
    print("  Example: Motor moves CW, then reverses CCW - there's a small gap")
    print("  before the gears engage and the output shaft actually moves.")
    print()
    print("WHY DOES IT MATTER?")
    print("  Backlash can cause oscillation during tracking:")
    print("  - System commands motor to move 2° right")
    print("  - First 2° absorbed by backlash (no movement)")
    print("  - PID increases output thinking motor 'stuck'")
    print("  - Backlash gap closes, full torque applied, overshoots")
    print("  - System reverses direction, backlash causes same problem")
    print("  - Result: Continuous oscillation")
    print()
    print("TYPICAL BACKLASH:")
    print("  - High-quality gearbox: 0.5-1°")
    print("  - Standard gearbox: 1-3°")
    print("  - Low-cost gearbox: 3-5°")
    print("  - Your 135:1 gear ratio may compound this")
    print()
    print("=" * 70)
    print()

    # Load configuration
    config = load_config()
    config.simulation_mode = False  # Force real hardware

    # Create hardware
    encoder = HardwareFactory.create_encoder(config)
    motor = HardwareFactory.create_motor(config)

    # Create controller (motor, encoder, config)
    pan_controller = PanController(motor, encoder, config)

    try:
        pan_controller.start()
        time.sleep(1.0)  # Let controller stabilize

        print("STEP 1: Initial position measurement")
        print("-" * 70)

        initial_angle = pan_controller.get_current_angle()
        print(f"Starting angle: {initial_angle:.2f}°")
        print()

        # Test backlash by moving back and forth
        test_angles = [15, -15, 30, -30, 45, -45]
        backlash_measurements = []

        print("STEP 2: Backlash measurement sequence")
        print("-" * 70)
        print()
        print("Test procedure:")
        print("  1. Move to target angle")
        print("  2. Wait for motor to settle")
        print("  3. Record actual position")
        print("  4. Move back to initial position")
        print("  5. Wait for motor to settle")
        print("  6. Record actual position")
        print("  7. Calculate error (backlash)")
        print()

        for i, offset in enumerate(test_angles):
            print(f"Test {i+1}/{len(test_angles)}: Moving {offset:+.1f}° from initial...")

            target_angle = initial_angle + offset

            # Move to target
            pan_controller.set_position_target(target_angle)
            time.sleep(3.0)  # Wait for motor to settle

            # Read actual position
            actual_angle_1 = pan_controller.get_current_angle()
            print(f"  → Reached: {actual_angle_1:.2f}° (target: {target_angle:.2f}°, error: {actual_angle_1 - target_angle:+.2f}°)")

            # Move back to initial
            pan_controller.set_position_target(initial_angle)
            time.sleep(3.0)  # Wait for motor to settle

            # Read actual position
            actual_angle_2 = pan_controller.get_current_angle()
            print(f"  ← Returned: {actual_angle_2:.2f}° (target: {initial_angle:.2f}°, error: {actual_angle_2 - initial_angle:+.2f}°)")

            # Calculate backlash (difference between expected and actual return position)
            backlash = abs(actual_angle_2 - initial_angle)
            backlash_measurements.append(backlash)

            print(f"  Backlash: {backlash:.2f}°")
            print()

            time.sleep(1.0)  # Brief pause between tests

        print("=" * 70)
        print("RESULTS")
        print("=" * 70)
        print()

        avg_backlash = np.mean(backlash_measurements)
        max_backlash = np.max(backlash_measurements)
        min_backlash = np.min(backlash_measurements)
        std_backlash = np.std(backlash_measurements)

        print(f"Backlash measurements: {[f'{b:.2f}°' for b in backlash_measurements]}")
        print()
        print(f"Average backlash: {avg_backlash:.2f}°")
        print(f"Maximum backlash: {max_backlash:.2f}°")
        print(f"Minimum backlash: {min_backlash:.2f}°")
        print(f"Std deviation: {std_backlash:.2f}°")
        print()

        # Interpretation
        if avg_backlash < 1.0:
            print("✓ GOOD: Backlash is minimal (<1°)")
            print("  Backlash is unlikely to cause tracking oscillation.")
        elif avg_backlash < 3.0:
            print("⚠️  MODERATE: Backlash is noticeable (1-3°)")
            print("  This may contribute to oscillation during direction reversals.")
            print()
            print("RECOMMENDATIONS:")
            print("  1. Increase tracking deadzone to at least 2× backlash")
            print(f"     Current deadzone: {config.pan_motor.position_pid['deadzone_degrees']}°")
            print(f"     Recommended: {avg_backlash * 2:.1f}°")
            print()
            print("  2. Consider backlash compensation in software:")
            print("     - Pre-load gears when reversing direction")
            print("     - Add small 'bump' in reverse direction before actual move")
        else:
            print("✗ LARGE: Backlash is significant (>3°)")
            print("  This is likely a MAJOR contributor to oscillation!")
            print()
            print("RECOMMENDATIONS:")
            print("  1. HARDWARE: Check for:")
            print("     - Loose gears or set screws")
            print("     - Worn gears")
            print("     - Misaligned gear mesh")
            print("     - Loose motor mount")
            print()
            print("  2. SOFTWARE: Increase deadzone significantly:")
            print(f"     Current deadzone: {config.pan_motor.position_pid['deadzone_degrees']}°")
            print(f"     Recommended: {avg_backlash * 2:.1f}°")
            print()
            print("  3. Consider replacing gearbox with higher-quality unit")

        print()
        print("STEP 3: Directional asymmetry test (optional)")
        print("-" * 70)
        response = input("Test for directional differences in backlash? (y/n): ")

        if response.lower() == 'y':
            print()
            print("Testing CW direction...")

            # Move CW
            pan_controller.set_position_target(initial_angle + 30)
            time.sleep(3.0)
            cw_angle = pan_controller.get_current_angle()

            # Return
            pan_controller.set_position_target(initial_angle)
            time.sleep(3.0)
            cw_return = pan_controller.get_current_angle()
            cw_backlash = abs(cw_return - initial_angle)

            print(f"CW backlash: {cw_backlash:.2f}°")
            print()

            print("Testing CCW direction...")

            # Move CCW
            pan_controller.set_position_target(initial_angle - 30)
            time.sleep(3.0)
            ccw_angle = pan_controller.get_current_angle()

            # Return
            pan_controller.set_position_target(initial_angle)
            time.sleep(3.0)
            ccw_return = pan_controller.get_current_angle()
            ccw_backlash = abs(ccw_return - initial_angle)

            print(f"CCW backlash: {ccw_backlash:.2f}°")
            print()

            asymmetry = abs(cw_backlash - ccw_backlash)
            print(f"Directional asymmetry: {asymmetry:.2f}°")

            if asymmetry > 1.0:
                print()
                print("⚠️  Significant directional asymmetry detected!")
                print("  This suggests:")
                print("  - Gear mesh may be tighter in one direction")
                print("  - Motor mount may be loose or tilted")
                print("  - Gears may be worn unevenly")

        print()
        print("=" * 70)
        print("Test complete!")
        print("=" * 70)

    finally:
        pan_controller.stop()

if __name__ == "__main__":
    main()
