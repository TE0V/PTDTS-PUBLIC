#!/usr/bin/env python3
"""
Test script for ReSpeaker audio array and ODAS integration
"""
import sys
import subprocess
import time
import socket
import json
import logging

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)


def check_audio_devices():
    """Check for connected audio devices"""
    logger.info("Checking for audio devices...")
    try:
        result = subprocess.run(['arecord', '-l'], capture_output=True, text=True)
        if result.returncode == 0:
            logger.info("Audio devices found:")
            print(result.stdout)

            # Look for ReSpeaker (case-insensitive)
            stdout_lower = result.stdout.lower()
            if 'respeaker' in stdout_lower or 'arrayuac10' in stdout_lower:
                logger.info("✓ ReSpeaker 4-mic array detected!")
                return True
            else:
                logger.warning("⚠ ReSpeaker not found. Please connect the USB microphone array.")
                return False
        else:
            logger.error("No audio devices found.")
            logger.error(result.stderr)
            return False
    except FileNotFoundError:
        logger.error("arecord command not found. Please install alsa-utils.")
        return False


def check_odas():
    """Check if ODAS is installed"""
    logger.info("Checking ODAS installation...")
    try:
        result = subprocess.run(['which', 'odaslive'], capture_output=True, text=True)
        if result.returncode == 0:
            logger.info(f"✓ ODAS installed at: {result.stdout.strip()}")
            return True
        else:
            logger.error("✗ ODAS not found. Please install ODAS first.")
            return False
    except Exception as e:
        logger.error(f"Error checking ODAS: {e}")
        return False


def test_odas_listener():
    """Test connecting to ODAS server"""
    logger.info("Testing ODAS listener connection...")
    logger.info("Note: ODAS server must be running for this test to work.")
    logger.info("To start ODAS manually, run:")
    logger.info("  odaslive -c /opt/odas/config/ptdts_respeaker.cfg")

    # Try to connect to ODAS port
    port = 9000
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        sock.connect(('localhost', port))
        logger.info(f"✓ Connected to ODAS on port {port}")

        # Try to receive data
        logger.info("Listening for detections (5 seconds)...")
        sock.settimeout(5)
        data = sock.recv(4096)
        if data:
            logger.info(f"✓ Received data from ODAS: {len(data)} bytes")
            try:
                decoded = data.decode('utf-8')
                logger.info(f"Sample data: {decoded[:200]}")
            except:
                pass
        sock.close()
        return True

    except socket.timeout:
        logger.warning("⚠ Connection timeout. Is ODAS server running?")
        return False
    except ConnectionRefusedError:
        logger.warning("⚠ Connection refused. ODAS server is not running.")
        logger.info("Start ODAS first, then run this test again.")
        return False
    except Exception as e:
        logger.error(f"✗ Error: {e}")
        return False


def test_python_integration():
    """Test the Python acoustic detector integration"""
    logger.info("Testing Python integration...")

    try:
        # Add src directory to path
        import os
        ptdts_root = os.path.dirname(os.path.abspath(__file__))
        sys.path.insert(0, os.path.join(ptdts_root, 'src'))

        from hardware.acoustic import ODASAcousticDetector
        from utils.config_loader import load_config

        # Load config
        config_path = os.path.join(ptdts_root, 'config', 'config.yaml')
        config = load_config(config_path)

        # Create detector
        logger.info("Creating ODASAcousticDetector...")
        detector = ODASAcousticDetector(
            mic_radius_mm=config.acoustic.mic_radius_mm,
            mic_count=config.acoustic.mic_count,
            odas_port=config.acoustic.odas_port,
            odas_config_path=config.acoustic.odas_config_path,
            energy_threshold=config.acoustic.energy_threshold,
            frequency_min=config.acoustic.frequency_min,
            frequency_max=config.acoustic.frequency_max
        )

        logger.info("✓ ODASAcousticDetector created successfully")
        logger.info("Starting detector...")

        detector.start()

        if detector.is_running():
            logger.info("✓ Detector is running")

            # Listen for detections
            logger.info("Listening for acoustic detections (10 seconds)...")
            logger.info("Make some noise near the microphone array!")

            for i in range(10):
                time.sleep(1)
                detections = detector.get_detections()
                if detections:
                    logger.info(f"✓ Detections received: {len(detections)}")
                    for det in detections[:3]:  # Show first 3
                        logger.info(f"  Azimuth: {det['azimuth']:.1f}°, "
                                  f"Elevation: {det['elevation']:.1f}°, "
                                  f"Energy: {det['energy']:.2f}")
                else:
                    logger.info(f"  [{i+1}/10] No detections yet...")

            detector.stop()
            logger.info("✓ Detector stopped")
            return True
        else:
            logger.error("✗ Detector failed to start")
            return False

    except ImportError as e:
        logger.error(f"✗ Import error: {e}")
        logger.info("Make sure all dependencies are installed: pip install -r requirements.txt")
        return False
    except Exception as e:
        logger.error(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests"""
    print("=" * 60)
    print("PTDTS Audio Array Test Suite")
    print("=" * 60)
    print()

    results = {}

    # Test 1: Audio devices
    print("\n--- Test 1: Audio Device Detection ---")
    results['audio_devices'] = check_audio_devices()

    # Test 2: ODAS installation
    print("\n--- Test 2: ODAS Installation ---")
    results['odas'] = check_odas()

    # Test 3: ODAS listener (optional, requires running server)
    print("\n--- Test 3: ODAS Connection ---")
    results['odas_connection'] = test_odas_listener()

    # Test 4: Python integration (optional)
    print("\n--- Test 4: Python Integration ---")
    results['python_integration'] = test_python_integration()

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary:")
    print("=" * 60)
    for test, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{test:20s}: {status}")

    print("\nNext steps:")
    if not results['audio_devices']:
        print("1. Connect the ReSpeaker 4-mic array via USB")
    if results['audio_devices'] and results['odas']:
        print("1. Audio array is ready!")
        print("2. Run: python main.py")
        print("   The system will auto-start ODAS and begin listening for drones")

    print()


if __name__ == '__main__':
    main()
