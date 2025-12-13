#!/usr/bin/env python3
"""
GS Camera Offset Calibration Script

This script helps calibrate the offset between HQ and GS cameras.
The cameras may not have exactly the same optical axis, so when switching
from HQ to GS, the target may appear off-center even if it was centered on HQ.

This script measures the offset and updates the configuration.
"""

import sys
import time
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from src.utils.config_loader import load_config
from src.hardware.camera import DualCameraManager
from src.detection.visual_detector import VisualDetector
import numpy as np

def main():
    print("=" * 70)
    print("GS CAMERA OFFSET CALIBRATION")
    print("=" * 70)
    print()
    print("This script measures the optical offset between HQ and GS cameras.")
    print()
    print("SETUP INSTRUCTIONS:")
    print("1. Place a drone or test target in view")
    print("2. Ensure target is stationary")
    print("3. Make sure both cameras can see the target")
    print()
    print("=" * 70)
    print()

    # Load configuration
    config = load_config()
    print(f"Current GS camera offset: {config.calibration.camera_offsets['gs']}")
    print()

    input("Press Enter when ready to start calibration...")
    print()

    # Initialize cameras and detector
    camera_manager = DualCameraManager(config)
    camera_manager.start()
    visual_detector = VisualDetector(config)

    try:
        print("STEP 1: Detecting target on HQ camera")
        print("-" * 70)

        # Switch to HQ camera
        camera_manager.switch_to_hq()
        time.sleep(0.5)  # Let camera settle

        # Capture and detect
        hq_detections = []
        for i in range(5):
            frame, camera_name = camera_manager.capture_frame()
            detections = visual_detector.detect(frame)

            if detections:
                best = detections[0]
                hq_detections.append(best.center[0])
                print(f"  Frame {i+1}: Detected at x={best.center[0]:.1f} (confidence: {best.confidence:.2f})")
            else:
                print(f"  Frame {i+1}: No detection")

            time.sleep(0.1)

        if not hq_detections:
            print()
            print("✗ ERROR: No detections on HQ camera!")
            print("  Make sure target is visible and detection confidence is adequate.")
            return

        hq_center_x = np.mean(hq_detections)
        hq_frame_width = frame.shape[1]
        hq_frame_center = hq_frame_width / 2
        hq_pixel_error = hq_center_x - hq_frame_center

        print()
        print(f"HQ Camera Results:")
        print(f"  Frame size: {hq_frame_width}x{frame.shape[0]}")
        print(f"  Target center: {hq_center_x:.1f} px")
        print(f"  Frame center: {hq_frame_center:.1f} px")
        print(f"  Pixel offset: {hq_pixel_error:+.1f} px")

        # Convert to angle
        hq_fov = config.cameras.hq.fov_h
        hq_angle_error = hq_pixel_error * (hq_fov / hq_frame_width)
        print(f"  Angle offset: {hq_angle_error:+.2f}° (FOV: {hq_fov}°)")
        print()

        print("STEP 2: Detecting target on GS camera")
        print("-" * 70)

        # Switch to GS camera
        camera_manager.switch_to_gs()
        time.sleep(0.5)  # Let camera settle

        # Capture and detect
        gs_detections = []
        for i in range(5):
            frame, camera_name = camera_manager.capture_frame()
            detections = visual_detector.detect(frame)

            if detections:
                best = detections[0]
                gs_detections.append(best.center[0])
                print(f"  Frame {i+1}: Detected at x={best.center[0]:.1f} (confidence: {best.confidence:.2f})")
            else:
                print(f"  Frame {i+1}: No detection")

            time.sleep(0.1)

        if not gs_detections:
            print()
            print("✗ ERROR: No detections on GS camera!")
            print("  Make sure target is still visible on GS camera.")
            print("  GS camera has narrower FOV (45°) than HQ (60°).")
            return

        gs_center_x = np.mean(gs_detections)
        gs_frame_width = frame.shape[1]
        gs_frame_center = gs_frame_width / 2
        gs_pixel_error = gs_center_x - gs_frame_center

        print()
        print(f"GS Camera Results:")
        print(f"  Frame size: {gs_frame_width}x{frame.shape[0]}")
        print(f"  Target center: {gs_center_x:.1f} px")
        print(f"  Frame center: {gs_frame_center:.1f} px")
        print(f"  Pixel offset: {gs_pixel_error:+.1f} px")

        # Convert to angle
        gs_fov = config.cameras.gs.fov_h
        gs_angle_error = gs_pixel_error * (gs_fov / gs_frame_width)
        print(f"  Angle offset: {gs_angle_error:+.2f}° (FOV: {gs_fov}°)")
        print()

        print("STEP 3: Calculate camera offset")
        print("-" * 70)

        # The offset is the difference in where the target appears on each camera
        # If HQ sees target at +5° and GS sees it at +8°, offset is +3°
        pan_offset = gs_angle_error - hq_angle_error

        print(f"Calculated pan offset: {pan_offset:+.2f}°")
        print()
        print("This offset should be added to config/config.yaml:")
        print()
        print("calibration:")
        print("  camera_offsets:")
        print(f"    hq: [0.0, 0.0]")
        print(f"    gs: [{pan_offset:.2f}, 0.0]  # <-- UPDATE THIS")
        print()

        if abs(pan_offset) < 1.0:
            print("✓ GOOD: Offset is small (<1°), cameras are well-aligned")
        elif abs(pan_offset) < 3.0:
            print("⚠️  MODERATE: Offset is noticeable, calibration recommended")
        else:
            print("✗ LARGE: Offset is significant (>3°), calibration strongly recommended")

        print()
        response = input("Update config.yaml automatically? (y/n): ")

        if response.lower() == 'y':
            # Update config
            config.calibration.camera_offsets['gs'] = [round(pan_offset, 2), 0.0]

            # Save config
            from src.utils.config_loader import save_config
            config_path = "config/config.yaml"
            save_config(config, config_path)

            print()
            print("✓ Configuration updated!")
            print(f"  GS camera offset set to: {config.calibration.camera_offsets['gs']}")
        else:
            print()
            print("Configuration NOT updated. Please update manually if needed.")

        print()
        print("=" * 70)
        print("Calibration complete!")
        print("=" * 70)

    finally:
        camera_manager.stop()

if __name__ == "__main__":
    main()
