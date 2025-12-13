#!/bin/bash
# PTDTS Calibration Launcher
# Usage: ./run_calibration.sh [calibration_name]
#
# Examples:
#   ./run_calibration.sh all          # Run all calibrations
#   ./run_calibration.sh motor        # Run motor calibration
#   ./run_calibration.sh camera_hq    # Run HQ camera calibration
#   ./run_calibration.sh camera_gs    # Run GS camera calibration
#   ./run_calibration.sh servo        # Run servo calibration
#   ./run_calibration.sh acoustic     # Run acoustic calibration

set -e  # Exit on error

# Change to PTDTS directory
cd "$(dirname "$0")"

# Check argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 [calibration_name]"
    echo ""
    echo "Available calibrations:"
    echo "  all         - Run all calibrations (interactive)"
    echo "  motor       - Motor encoder calibration"
    echo "  camera_hq   - HQ camera FOV calibration"
    echo "  camera_gs   - GS camera FOV calibration"
    echo "  servo       - Servo feedback calibration"
    echo "  acoustic    - Acoustic alignment calibration"
    echo ""
    exit 1
fi

CALIBRATION=$1

case $CALIBRATION in
    all)
        echo "Running master calibration script..."
        python3 calibration/calibrate_all.py
        ;;
    motor)
        echo "Running motor encoder calibration..."
        python3 calibration/calibrate_motor.py
        ;;
    camera_hq)
        echo "Running HQ camera FOV calibration..."
        python3 calibration/calibrate_camera_fov.py --camera hq
        ;;
    camera_gs)
        echo "Running GS camera FOV calibration..."
        python3 calibration/calibrate_camera_fov.py --camera gs
        ;;
    servo)
        echo "Running servo feedback calibration..."
        python3 calibration/calibrate_servo.py
        ;;
    acoustic)
        echo "Running acoustic alignment calibration..."
        python3 calibration/calibrate_acoustic.py
        ;;
    *)
        echo "Error: Unknown calibration '$CALIBRATION'"
        echo "Run '$0' without arguments to see available calibrations"
        exit 1
        ;;
esac

echo ""
echo "Calibration complete!"
