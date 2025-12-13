#!/usr/bin/env python3
"""
Acoustic Array Alignment Calibration for PTDTS
Calibrates azimuth offset between acoustic array and pan motor zero position

Usage:
    python3 calibration/calibrate_acoustic.py

This tool will:
1. Play test sound at known azimuth
2. Measure acoustic detection angle
3. Calculate offset between acoustic and motor coordinate systems
4. Update config.yaml with acoustic_offset
"""

import sys
import time
from pathlib import Path
import math

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.utils.config_loader import load_config
from src.utils.logger import setup_logging
from src.hardware.factory import HardwareFactory
import logging

logger = logging.getLogger(__name__)


def calibrate_acoustic_alignment():
    """
    Calibrate acoustic array alignment with pan motor
    """
    print("\n" + "="*70)
    print("PTDTS Acoustic Array Alignment Calibration")
    print("="*70 + "\n")

    # Load config
    config = load_config()
    setup_logging(config)

    print("This calibration aligns the acoustic detection coordinate system")
    print("with the pan motor coordinate system.\n")

    print("You will need:")
    print("  - Sound source (phone playing tone, speaker, etc.)")
    print("  - Ability to place sound at known azimuth relative to system")
    print("  - Quiet environment for best results\n")

    print("COORDINATE SYSTEMS:")
    print("-" * 70)
    print("Pan Motor:  0° = forward, increases clockwise (viewed from top)")
    print("Acoustic:   0° = front of mic array, needs alignment with motor")
    print("-" * 70)
    print()

    input("Press ENTER to continue...")

    # Initialize hardware
    config.simulation_mode = False

    print("\nInitializing hardware...")
    try:
        # Create encoder
        encoder = HardwareFactory.create_encoder(config)

        # Create motor
        motor = HardwareFactory.create_motor(config, encoder)

        # Create acoustic detector
        acoustic = HardwareFactory.create_acoustic_detector(config)

        print("Hardware initialized.\n")

    except Exception as e:
        print(f"ERROR: Failed to initialize hardware: {e}")
        print("\nMake sure you're running on Raspberry Pi with hardware connected.")
        print("ODAS must be running for acoustic detection.")
        return

    try:
        # METHOD 1: Fixed Position Test
        print("\n" + "="*70)
        print("METHOD 1: Fixed Position Alignment Test")
        print("="*70)
        print("\nThis method uses a sound source at a known position.")
        print()
        print("Procedure:")
        print("  1. Position sound source directly in front (0° motor position)")
        print("  2. System will measure acoustic detection angle")
        print("  3. Difference is the calibration offset")
        print()

        input("Position sound source at motor 0° (directly forward). Press ENTER when ready...")

        # Reset encoder to define 0°
        print("\nResetting motor position to 0°...")
        encoder.reset()
        motor_angle = 0.0

        # Start acoustic detection
        print("Starting acoustic detector...")
        acoustic.start()
        time.sleep(1.0)

        # Play sound and detect
        print("\nPlay a continuous tone/sound from the source.")
        print("Listening for 10 seconds...\n")

        detections = []
        start_time = time.time()

        while time.time() - start_time < 10.0:
            current_detections = acoustic.get_detections()
            if current_detections:
                for det in current_detections:
                    if det['energy'] >= config.acoustic.energy_threshold:
                        detections.append(det['azimuth'])
                        print(f"  Detection: azimuth {det['azimuth']:6.2f}°, energy {det['energy']:.2f}")

            time.sleep(0.1)

        acoustic.stop()

        if len(detections) == 0:
            print("\nNo acoustic detections received!")
            print("Make sure:")
            print("  - Sound source is loud enough")
            print("  - ODAS is running and configured correctly")
            print("  - USB mic array is connected")
            return

        # Calculate average detected azimuth
        avg_acoustic_azimuth = sum(detections) / len(detections)

        print(f"\nReceived {len(detections)} detections")
        print(f"Average acoustic azimuth: {avg_acoustic_azimuth:.2f}°")
        print(f"Motor azimuth:           {motor_angle:.2f}°")

        # Calculate offset
        # Offset = motor_angle - acoustic_angle
        # When motor points at 0°, acoustic reports avg_acoustic_azimuth
        # So offset = 0 - avg_acoustic_azimuth
        offset_method1 = motor_angle - avg_acoustic_azimuth

        print(f"\nCalculated offset: {offset_method1:.2f}°")

        # METHOD 2: Multiple Position Test (optional but more accurate)
        print("\n" + "="*70)
        print("METHOD 2: Multiple Position Test (Optional)")
        print("="*70)
        print("\nThis method tests multiple known positions for better accuracy.")
        print("\nPerform multi-position test? (y/n): ", end='')

        if input().lower().strip() == 'y':
            test_angles = [0, 90, 180, 270]
            offsets = []

            for test_angle in test_angles:
                print(f"\n--- Testing motor position {test_angle}° ---")

                # Rotate to test angle
                print(f"Rotating to {test_angle}°...")
                # Note: Would need position control here
                # For now, prompt user to manually position
                input(f"Manually rotate pan to {test_angle}°. Press ENTER when ready...")

                # Update motor reference
                current_motor = test_angle

                # Detect
                print("Play sound. Listening for 10 seconds...")
                acoustic.start()
                time.sleep(1.0)

                test_detections = []
                start_time = time.time()

                while time.time() - start_time < 10.0:
                    current_detections = acoustic.get_detections()
                    if current_detections:
                        for det in current_detections:
                            if det['energy'] >= config.acoustic.energy_threshold:
                                test_detections.append(det['azimuth'])
                                print(f"  {det['azimuth']:6.2f}°", end='', flush=True)
                    time.sleep(0.1)

                acoustic.stop()

                if test_detections:
                    avg_det = sum(test_detections) / len(test_detections)
                    test_offset = current_motor - avg_det
                    offsets.append(test_offset)
                    print(f"\n{len(test_detections)} detections, avg {avg_det:.2f}°, offset {test_offset:.2f}°")
                else:
                    print("\nNo detections at this angle")

            if offsets:
                offset_method2 = sum(offsets) / len(offsets)
                offset_std = (sum((o - offset_method2)**2 for o in offsets) / len(offsets))**0.5

                print(f"\nMulti-position results:")
                print(f"  Average offset: {offset_method2:.2f}°")
                print(f"  Std deviation:  {offset_std:.2f}°")

                recommended_offset = offset_method2
            else:
                recommended_offset = offset_method1
        else:
            recommended_offset = offset_method1

        # Display results
        print("\n" + "="*70)
        print("CALIBRATION RESULTS")
        print("="*70)
        print(f"\nRecommended acoustic offset: {recommended_offset:.2f}°")
        print()
        print("This offset will be applied as:")
        print("  motor_azimuth = acoustic_azimuth + offset")
        print()
        print("When ODAS reports azimuth θ, the motor will point to (θ + offset)°")

        print("\n" + "="*70)
        print("CONFIGURATION UPDATE")
        print("="*70)
        print(f"\nUpdate config/config.yaml in calibration section:")
        print(f"\n  acoustic_offset: {recommended_offset:.1f}")

        print("\n" + "="*70)

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted.")
    except Exception as e:
        logger.error(f"Calibration error: {e}", exc_info=True)
    finally:
        # Cleanup
        try:
            acoustic.stop()
            motor.stop()
        except:
            pass
        print("\nDone.\n")


if __name__ == "__main__":
    try:
        calibrate_acoustic_alignment()
    except KeyboardInterrupt:
        print("\n\nExiting...")
        sys.exit(0)
