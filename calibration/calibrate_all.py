#!/usr/bin/env python3
"""
PTDTS Master Calibration Script
Guides user through complete system calibration

Usage:
    python3 calibration/calibrate_all.py

This script will:
1. Run motor encoder calibration
2. Run camera FOV calibration (HQ and GS)
3. Run servo feedback calibration (optional)
4. Run acoustic alignment calibration
5. Save all calibration data
6. Generate config.yaml update instructions
"""

import sys
import subprocess
from pathlib import Path

# Add project root and src to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "src"))

from calibration.calibration_data import CalibrationData


def print_header(title):
    """Print formatted section header"""
    print("\n" + "=" * 70)
    print(title)
    print("=" * 70 + "\n")


def print_separator():
    """Print separator line"""
    print("-" * 70)


def run_calibration_script(script_name, description):
    """
    Run a calibration script

    Args:
        script_name: Name of the script to run
        description: Description of what's being calibrated

    Returns:
        True if successful, False otherwise
    """
    print_header(f"STEP: {description}")

    script_path = Path(__file__).parent / script_name

    if not script_path.exists():
        print(f"ERROR: Script not found: {script_path}")
        return False

    print(f"Running: {script_name}\n")
    print_separator()

    try:
        # Run the calibration script
        result = subprocess.run(
            [sys.executable, str(script_path)],
            cwd=Path(__file__).parent.parent,
            check=False
        )

        print_separator()

        if result.returncode == 0:
            print(f"\n{description} completed successfully!\n")
            return True
        else:
            print(f"\n{description} encountered errors or was skipped.\n")
            return False

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user.")
        return False
    except Exception as e:
        print(f"\nERROR running {script_name}: {e}\n")
        return False


def main():
    """Main calibration workflow"""
    print_header("PTDTS COMPLETE SYSTEM CALIBRATION")

    print("This script will guide you through calibrating all hardware subsystems.")
    print("You can skip any calibration step if needed.")
    print()
    print("Calibration steps:")
    print("  1. Motor encoder (required for pan axis)")
    print("  2. Camera FOV - HQ camera (required for detection)")
    print("  3. Camera FOV - GS camera (required for tracking)")
    print("  4. Servo feedback (optional, for closed-loop tilt)")
    print("  5. Acoustic alignment (required for sound localization)")
    print()
    print("All calibration data will be saved to calibration/calibration.json")
    print()

    input("Press ENTER to begin calibration...")

    # Initialize calibration data storage
    cal_data = CalibrationData("calibration/calibration.json")

    # Track completion status
    completed_steps = []
    skipped_steps = []

    # Step 1: Motor encoder calibration
    print_header("1. MOTOR ENCODER CALIBRATION")
    print("This calibration measures encoder counts per 360° rotation.")
    print("You will need to manually rotate the motor shaft.")
    print()

    proceed = input("Proceed with motor calibration? (y/n): ").lower().strip()
    if proceed == 'y':
        success = run_calibration_script("calibrate_motor.py", "Motor Encoder Calibration")
        if success:
            completed_steps.append("Motor encoder")
        else:
            skipped_steps.append("Motor encoder")
    else:
        print("Skipping motor calibration.\n")
        skipped_steps.append("Motor encoder")

    # Step 2: HQ Camera FOV calibration
    print_header("2. HQ CAMERA FOV CALIBRATION")
    print("This calibration measures the HQ camera's field of view.")
    print("You will need a calibration object of known size.")
    print()

    proceed = input("Proceed with HQ camera calibration? (y/n): ").lower().strip()
    if proceed == 'y':
        # Run with --camera hq argument
        print_separator()
        print("Running: calibrate_camera_fov.py --camera hq\n")
        print_separator()

        try:
            result = subprocess.run(
                [sys.executable, "calibration/calibrate_camera_fov.py", "--camera", "hq"],
                cwd=Path(__file__).parent.parent,
                check=False
            )

            print_separator()

            if result.returncode == 0:
                print("\nHQ Camera FOV calibration completed successfully!\n")
                completed_steps.append("HQ camera FOV")
            else:
                print("\nHQ Camera FOV calibration encountered errors or was skipped.\n")
                skipped_steps.append("HQ camera FOV")

        except KeyboardInterrupt:
            print("\n\nCalibration interrupted.\n")
            skipped_steps.append("HQ camera FOV")
        except Exception as e:
            print(f"\nERROR: {e}\n")
            skipped_steps.append("HQ camera FOV")
    else:
        print("Skipping HQ camera calibration.\n")
        skipped_steps.append("HQ camera FOV")

    # Step 3: GS Camera FOV calibration
    print_header("3. GS CAMERA FOV CALIBRATION")
    print("This calibration measures the GS camera's field of view.")
    print("You will need a calibration object of known size.")
    print()

    proceed = input("Proceed with GS camera calibration? (y/n): ").lower().strip()
    if proceed == 'y':
        # Run with --camera gs argument
        print_separator()
        print("Running: calibrate_camera_fov.py --camera gs\n")
        print_separator()

        try:
            result = subprocess.run(
                [sys.executable, "calibration/calibrate_camera_fov.py", "--camera", "gs"],
                cwd=Path(__file__).parent.parent,
                check=False
            )

            print_separator()

            if result.returncode == 0:
                print("\nGS Camera FOV calibration completed successfully!\n")
                completed_steps.append("GS camera FOV")
            else:
                print("\nGS Camera FOV calibration encountered errors or was skipped.\n")
                skipped_steps.append("GS camera FOV")

        except KeyboardInterrupt:
            print("\n\nCalibration interrupted.\n")
            skipped_steps.append("GS camera FOV")
        except Exception as e:
            print(f"\nERROR: {e}\n")
            skipped_steps.append("GS camera FOV")
    else:
        print("Skipping GS camera calibration.\n")
        skipped_steps.append("GS camera FOV")

    # Step 4: Servo feedback calibration (optional)
    print_header("4. SERVO FEEDBACK CALIBRATION (OPTIONAL)")
    print("This calibration is only needed if you want closed-loop tilt control.")
    print("It requires the ADS1115 ADC to be connected to servo feedback.")
    print()

    proceed = input("Proceed with servo calibration? (y/n): ").lower().strip()
    if proceed == 'y':
        success = run_calibration_script("calibrate_servo.py", "Servo Feedback Calibration")
        if success:
            completed_steps.append("Servo feedback")
        else:
            skipped_steps.append("Servo feedback")
    else:
        print("Skipping servo calibration.\n")
        skipped_steps.append("Servo feedback")

    # Step 5: Acoustic alignment calibration
    print_header("5. ACOUSTIC ALIGNMENT CALIBRATION")
    print("This calibration aligns the acoustic array with the pan motor.")
    print("You will need a sound source (phone, speaker, etc.).")
    print()

    proceed = input("Proceed with acoustic calibration? (y/n): ").lower().strip()
    if proceed == 'y':
        success = run_calibration_script("calibrate_acoustic.py", "Acoustic Alignment Calibration")
        if success:
            completed_steps.append("Acoustic alignment")
        else:
            skipped_steps.append("Acoustic alignment")
    else:
        print("Skipping acoustic calibration.\n")
        skipped_steps.append("Acoustic alignment")

    # Summary
    print_header("CALIBRATION COMPLETE")

    print(f"Completed steps ({len(completed_steps)}):")
    if completed_steps:
        for step in completed_steps:
            print(f"  - {step}")
    else:
        print("  (none)")
    print()

    print(f"Skipped steps ({len(skipped_steps)}):")
    if skipped_steps:
        for step in skipped_steps:
            print(f"  - {step}")
    else:
        print("  (none)")
    print()

    # Display calibration data summary
    if Path("calibration/calibration.json").exists():
        print_separator()
        print()
        cal_data = CalibrationData("calibration/calibration.json")
        print(cal_data.get_summary())
        print()
        cal_data.print_config_updates()
    else:
        print("No calibration data file found.")
        print("Run individual calibration scripts to generate data.")

    print("\n" + "=" * 70)
    print("\nCalibration workflow complete!")
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user.")
        print("You can resume calibration at any time by running this script again.\n")
        sys.exit(0)
    except Exception as e:
        print(f"\n\nERROR: {e}\n")
        sys.exit(1)
