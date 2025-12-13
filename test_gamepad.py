#!/usr/bin/env python3
"""
Test script for gamepad controller
"""

import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

from src.utils.config_loader import load_config
from src.control.gamepad_controller import GamepadController, GAMEPAD_AVAILABLE

def test_gamepad():
    """Test gamepad controller initialization"""
    print("=" * 60)
    print("PTDTS Gamepad Controller Test")
    print("=" * 60)

    # Check if gamepad library is available
    print(f"\nGamepad library available: {GAMEPAD_AVAILABLE}")

    if not GAMEPAD_AVAILABLE:
        print("ERROR: inputs library not installed")
        print("Install with: pip install inputs")
        return False

    # Load config
    print("\nLoading configuration...")
    try:
        config = load_config("config/config.yaml")
        print("✓ Configuration loaded")
    except Exception as e:
        print(f"✗ Failed to load configuration: {e}")
        return False

    # Check gamepad config
    if hasattr(config, 'gamepad'):
        print(f"✓ Gamepad enabled: {config.gamepad.enabled}")
        print(f"  - Stick deadzone: {config.gamepad.stick_deadzone}")
        print(f"  - Pan sensitivity: {config.gamepad.pan_sensitivity} deg/s")
        print(f"  - Tilt sensitivity: {config.gamepad.tilt_sensitivity} deg/s")
    else:
        print("✗ Gamepad configuration not found")
        return False

    # Try to create gamepad controller
    print("\nInitializing gamepad controller...")
    try:
        gamepad = GamepadController(config)
        print("✓ Gamepad controller created")
    except Exception as e:
        print(f"✗ Failed to create gamepad controller: {e}")
        return False

    # Test callback registration
    print("\nTesting callback registration...")

    test_calls = []

    def test_callback():
        test_calls.append(True)

    try:
        gamepad.register_callback('toggle_mode', test_callback)
        gamepad.register_callback('emergency_stop', test_callback)
        print(f"✓ Registered {len(gamepad.callbacks)} callbacks")
    except Exception as e:
        print(f"✗ Failed to register callbacks: {e}")
        return False

    # Test state dict
    print("\nGetting gamepad state...")
    try:
        state = gamepad.get_state_dict()
        print(f"✓ Gamepad state retrieved")
        print(f"  - Connected: {state['connected']}")
        print(f"  - Pan velocity: {state['commands']['pan_velocity']} deg/s")
        print(f"  - Tilt angle: {state['commands']['tilt_angle']}°")
    except Exception as e:
        print(f"✗ Failed to get state: {e}")
        return False

    print("\n" + "=" * 60)
    print("Gamepad controller test completed successfully!")
    print("=" * 60)

    # Note about physical gamepad
    if not gamepad.is_connected():
        print("\nNOTE: No physical gamepad detected.")
        print("Connect an Xbox controller to test input functionality.")
        print("The system will automatically detect it when connected.")

    return True

if __name__ == "__main__":
    success = test_gamepad()
    sys.exit(0 if success else 1)
