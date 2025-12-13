#!/usr/bin/env python3
"""
Test script for state persistence functionality
Verifies that state is saved and restored correctly
"""

import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

from src.state.state_manager import StateManager
from src.utils.config_loader import load_config
from src.hardware.encoder import SimulatedEncoder
from src.control.pan_controller import PanController, PanMode
from src.hardware.motor import SimulatedMotor


def test_state_manager():
    """Test basic StateManager functionality"""
    print("\n=== Testing StateManager ===")

    # Create state manager with test file
    state_file = "state/test_runtime_state.json"
    sm = StateManager(state_file=state_file)

    # Test encoder state
    print("\n1. Testing encoder state...")
    sm.set_encoder_state(count=1000, angle=42.5, velocity=10.0)
    encoder_state = sm.get_encoder_state()
    assert encoder_state is not None
    assert encoder_state['count'] == 1000
    assert encoder_state['angle'] == 42.5
    assert encoder_state['velocity'] == 10.0
    print("   ✓ Encoder state saved and retrieved")

    # Test pan controller state
    print("\n2. Testing pan controller state...")
    sm.set_pan_controller_state(
        mode="position",
        target_position=90.0,
        target_velocity=None,
        pid_integral=0.5,
        pid_last_error=1.2
    )
    pan_state = sm.get_pan_controller_state()
    assert pan_state is not None
    assert pan_state['mode'] == "position"
    assert pan_state['target_position'] == 90.0
    print("   ✓ Pan controller state saved and retrieved")

    # Test tilt controller state
    print("\n3. Testing tilt controller state...")
    sm.set_tilt_controller_state(
        current_angle=45.0,
        target_angle=45.0,
        pid_integral=0.0,
        pid_last_error=0.0
    )
    tilt_state = sm.get_tilt_controller_state()
    assert tilt_state is not None
    assert tilt_state['current_angle'] == 45.0
    print("   ✓ Tilt controller state saved and retrieved")

    # Test state machine state
    print("\n4. Testing state machine state...")
    sm.set_state_machine_state(
        current_state="tracking",
        current_target={'center_x': 320, 'center_y': 240, 'confidence': 0.95},
        pending_acoustic=None,
        tracking_lost_count=0
    )
    sm_state = sm.get_state_machine_state()
    assert sm_state is not None
    assert sm_state['current_state'] == "tracking"
    assert sm_state['current_target']['confidence'] == 0.95
    print("   ✓ State machine state saved and retrieved")

    # Save to disk
    print("\n5. Testing disk persistence...")
    sm.save()
    print("   ✓ State saved to disk")

    # Load from disk
    sm2 = StateManager(state_file=state_file)
    encoder_state2 = sm2.get_encoder_state()
    assert encoder_state2['count'] == 1000
    print("   ✓ State loaded from disk successfully")

    print("\n=== StateManager tests passed ===\n")


def test_controller_integration():
    """Test controller save/load integration"""
    print("\n=== Testing Controller Integration ===")

    # Load config
    config = load_config("config/config.yaml")

    # Create simulated hardware
    encoder = SimulatedEncoder(counts_per_360=-8556)
    motor = SimulatedMotor(encoder=encoder)

    # Create pan controller
    pan_controller = PanController(
        motor=motor,
        encoder=encoder,
        config=config
    )

    # Set some state
    print("\n1. Setting controller state...")
    pan_controller.set_position_target(180.0)
    encoder.set_count(4278)  # Set encoder to a specific position
    print(f"   Position target: 180.0°")
    print(f"   Encoder count: {encoder.get_count()}")
    print(f"   Encoder angle: {encoder.get_angle():.2f}°")

    # Save state
    print("\n2. Saving controller state...")
    state_manager = StateManager(state_file="state/test_controller_state.json")
    pan_controller.save_state(state_manager)
    state_manager.save()
    print("   ✓ State saved")

    # Create new controller and encoder
    print("\n3. Creating new controller instance...")
    encoder2 = SimulatedEncoder(counts_per_360=-8556)
    motor2 = SimulatedMotor(encoder=encoder2)
    pan_controller2 = PanController(
        motor=motor2,
        encoder=encoder2,
        config=config
    )

    print(f"   Initial encoder count: {encoder2.get_count()}")
    print(f"   Initial encoder angle: {encoder2.get_angle():.2f}°")

    # Load state
    print("\n4. Loading saved state...")
    state_manager2 = StateManager(state_file="state/test_controller_state.json")
    pan_controller2.load_state(state_manager2)

    print(f"   Restored encoder count: {encoder2.get_count()}")
    print(f"   Restored encoder angle: {encoder2.get_angle():.2f}°")
    print(f"   Restored mode: {pan_controller2.get_mode().value}")

    # Verify state was restored
    assert encoder2.get_count() == 4278, f"Expected count 4278, got {encoder2.get_count()}"
    assert pan_controller2.get_mode() == PanMode.POSITION, f"Expected POSITION mode, got {pan_controller2.get_mode()}"

    print("   ✓ State restored successfully")

    print("\n=== Controller integration tests passed ===\n")


def main():
    """Run all tests"""
    print("\n" + "=" * 60)
    print("State Persistence Test Suite")
    print("=" * 60)

    try:
        test_state_manager()
        test_controller_integration()

        print("\n" + "=" * 60)
        print("✓ All tests passed!")
        print("=" * 60 + "\n")

    except AssertionError as e:
        print(f"\n✗ Test failed: {e}\n")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}\n")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
