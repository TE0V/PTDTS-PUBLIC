#!/usr/bin/env python3
"""
Camera FOV Calibration Tool for PTDTS
Measures field of view for both HQ and GS cameras

Usage:
    python3 calibration/calibrate_camera_fov.py [--camera hq|gs]

This tool will:
1. Display live camera feed with overlay
2. Guide you to measure known-width object at known distance
3. Calculate horizontal and vertical FOV
4. Update config.yaml with accurate FOV values
"""

import sys
import cv2
import numpy as np
import argparse
import math
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.utils.config_loader import load_config
from src.utils.logger import setup_logging
from src.hardware.factory import HardwareFactory
import logging

logger = logging.getLogger(__name__)


def calculate_fov(object_width_mm, object_distance_mm, pixels_span, image_width_pixels):
    """
    Calculate field of view using known object dimensions

    Args:
        object_width_mm: Physical width of calibration object (mm)
        object_distance_mm: Distance from camera to object (mm)
        pixels_span: Number of pixels the object spans in image
        image_width_pixels: Total image width in pixels

    Returns:
        FOV in degrees
    """
    # Calculate sensor width that sees the object
    # object_width / sensor_width = pixels_span / image_width_pixels
    sensor_width_mm = (object_width_mm * image_width_pixels) / pixels_span

    # Calculate FOV using arctan
    # FOV = 2 * arctan(sensor_width / (2 * focal_length))
    # But focal_length = object_distance * sensor_width / object_width (at focus)
    # Simplified: FOV = 2 * arctan(object_width / (2 * object_distance))
    # Using pixels: FOV = 2 * arctan((image_width / pixels_span) * object_width / (2 * distance))

    fov_radians = 2 * math.atan(sensor_width_mm / (2 * object_distance_mm))
    fov_degrees = math.degrees(fov_radians)

    return fov_degrees


def calibrate_camera_fov(camera_name='hq'):
    """
    Calibrate camera field of view

    Args:
        camera_name: 'hq' or 'gs'
    """
    print("\n" + "="*70)
    print(f"PTDTS Camera FOV Calibration - {camera_name.upper()} Camera")
    print("="*70 + "\n")

    # Load config
    config = load_config()
    setup_logging(config)

    # Get camera config
    if camera_name == 'hq':
        cam_config = config.cameras.hq
    elif camera_name == 'gs':
        cam_config = config.cameras.gs
    else:
        print(f"ERROR: Unknown camera '{camera_name}'. Use 'hq' or 'gs'.")
        return

    print(f"Camera: {camera_name.upper()}")
    print(f"Resolution: {cam_config.resolution[0]}x{cam_config.resolution[1]}")
    print(f"Current FOV (config): {cam_config.fov_h}° H x {cam_config.fov_v}° V\n")

    # Initialize camera
    print("Initializing camera...")
    try:
        camera_manager = HardwareFactory.create_cameras(config)
        camera_manager.start_cameras()

        if camera_name == 'hq':
            camera = camera_manager.hq_camera
        else:
            camera = camera_manager.gs_camera

        print("Camera initialized.\n")
    except Exception as e:
        print(f"ERROR: Failed to initialize camera: {e}")
        print("\nRunning in simulation mode for demonstration.")
        print("For actual calibration, run on Raspberry Pi with cameras.\n")
        return

    print("CALIBRATION PROCEDURE:")
    print("-" * 70)
    print("You will need:")
    print("  - A calibration object of known width (e.g., ruler, paper, poster)")
    print("  - Tape measure to measure distance from camera")
    print()
    print("Steps:")
    print("  1. Place calibration object at known distance from camera")
    print("  2. Align object horizontally for horizontal FOV measurement")
    print("  3. Mark object edges in the camera view")
    print("  4. Repeat vertically for vertical FOV")
    print("-" * 70)
    print("\nPress any key in the camera window to continue...")

    try:
        # Capture and display frame
        frame, _ = camera_manager.capture_frame() if camera_name == 'hq' else (camera.capture_array(), camera_name)

        height, width = frame.shape[:2]

        # Horizontal FOV measurement
        print("\n" + "="*70)
        print("HORIZONTAL FOV MEASUREMENT")
        print("="*70)

        object_width = float(input("\nEnter calibration object width (mm): "))
        distance = float(input("Enter distance from camera to object (mm): "))

        # Display frame with crosshairs for measurement
        display_frame = frame.copy()
        cv2.line(display_frame, (width//2, 0), (width//2, height), (0, 255, 0), 1)
        cv2.line(display_frame, (0, height//2), (width, height//2), (0, 255, 0), 1)

        # Add instructions overlay
        cv2.putText(display_frame, "Click LEFT edge of calibration object",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(display_frame, f"Object width: {object_width}mm at {distance}mm",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Mouse callback for marking points
        points = []

        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                points.append((x, y))
                cv2.circle(display_frame, (x, y), 5, (0, 0, 255), -1)
                cv2.imshow('Camera Calibration', display_frame)

        cv2.imshow('Camera Calibration', display_frame)
        cv2.setMouseCallback('Camera Calibration', mouse_callback)

        print("\nClick the LEFT edge of the calibration object in the camera view...")
        while len(points) < 1:
            cv2.waitKey(10)

        # Update display for right edge
        display_frame = frame.copy()
        cv2.circle(display_frame, points[0], 5, (0, 0, 255), -1)
        cv2.putText(display_frame, "Click RIGHT edge of calibration object",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow('Camera Calibration', display_frame)

        print("Click the RIGHT edge of the calibration object...")
        while len(points) < 2:
            cv2.waitKey(10)

        # Calculate horizontal FOV
        pixels_h = abs(points[1][0] - points[0][0])
        fov_h = calculate_fov(object_width, distance, pixels_h, width)

        print(f"\nHorizontal FOV: {fov_h:.2f}°")
        print(f"  Object spans {pixels_h} pixels ({pixels_h/width*100:.1f}% of frame width)")

        # Vertical FOV measurement
        print("\n" + "="*70)
        print("VERTICAL FOV MEASUREMENT")
        print("="*70)
        print("\nRotate calibration object 90° (now vertical orientation)")

        object_height = float(input("Enter calibration object height (mm, or same as width): ") or object_width)

        print("\nPress any key when ready...")
        cv2.waitKey(0)

        # Capture new frame
        frame, _ = camera_manager.capture_frame() if camera_name == 'hq' else (camera.capture_array(), camera_name)

        points_v = []
        display_frame = frame.copy()
        cv2.putText(display_frame, "Click TOP edge of calibration object",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        def mouse_callback_v(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                points_v.append((x, y))
                cv2.circle(display_frame, (x, y), 5, (0, 0, 255), -1)
                cv2.imshow('Camera Calibration', display_frame)

        cv2.imshow('Camera Calibration', display_frame)
        cv2.setMouseCallback('Camera Calibration', mouse_callback_v)

        print("Click the TOP edge of the calibration object...")
        while len(points_v) < 1:
            cv2.waitKey(10)

        # Update for bottom edge
        display_frame = frame.copy()
        cv2.circle(display_frame, points_v[0], 5, (0, 0, 255), -1)
        cv2.putText(display_frame, "Click BOTTOM edge of calibration object",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow('Camera Calibration', display_frame)

        print("Click the BOTTOM edge of the calibration object...")
        while len(points_v) < 2:
            cv2.waitKey(10)

        # Calculate vertical FOV
        pixels_v = abs(points_v[1][1] - points_v[0][1])
        fov_v = calculate_fov(object_height, distance, pixels_v, height)

        print(f"\nVertical FOV: {fov_v:.2f}°")
        print(f"  Object spans {pixels_v} pixels ({pixels_v/height*100:.1f}% of frame height)")

        # Display results
        print("\n" + "="*70)
        print("CALIBRATION RESULTS")
        print("="*70)
        print(f"\nCamera: {camera_name.upper()}")
        print(f"Resolution: {width}x{height}")
        print(f"\nMeasured FOV:")
        print(f"  Horizontal: {fov_h:.2f}°")
        print(f"  Vertical:   {fov_v:.2f}°")
        print(f"  Aspect ratio: {fov_h/fov_v:.3f}")

        print("\n" + "="*70)
        print("CONFIGURATION UPDATE")
        print("="*70)
        print(f"\nUpdate config/config.yaml in cameras.{camera_name} section:")
        print(f"\n  fov_h: {fov_h:.1f}")
        print(f"  fov_v: {fov_v:.1f}")

        cv2.destroyAllWindows()

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted.")
        cv2.destroyAllWindows()
    except Exception as e:
        logger.error(f"Calibration error: {e}", exc_info=True)
        cv2.destroyAllWindows()
    finally:
        camera_manager.close()
        print("\nDone.\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calibrate camera FOV")
    parser.add_argument('--camera', choices=['hq', 'gs'], default='hq',
                       help='Camera to calibrate (default: hq)')
    args = parser.parse_args()

    try:
        calibrate_camera_fov(args.camera)
    except KeyboardInterrupt:
        print("\n\nExiting...")
        sys.exit(0)
