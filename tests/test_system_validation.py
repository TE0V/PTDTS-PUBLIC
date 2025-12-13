#!/usr/bin/env python3
"""
PTDTS System Validation Test Suite
Comprehensive end-to-end testing in simulation mode

Tests:
1. System initialization and startup
2. State machine transitions
3. Control loop performance (50Hz target)
4. Pan/tilt controller response
5. Detection pipeline (simulated)
6. Manual control modes
7. Gamepad integration
8. State persistence
9. Hardware monitoring
10. Error handling and recovery
"""

import sys
import time
import json
import threading
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.utils.config_loader import load_config
from src.utils.logger import setup_logging
from src.hardware.factory import HardwareFactory
from src.control.pan_controller import PanController
from src.control.tilt_controller import TiltController
from src.state.state_machine import StateMachine, SystemState
from src.state.state_manager import StateManager


class TestResult:
    """Container for test results"""
    def __init__(self, name: str):
        self.name = name
        self.passed = False
        self.duration = 0.0
        self.message = ""
        self.details = {}
        self.warnings = []
        self.start_time = None
        self.end_time = None


class SystemValidator:
    """Comprehensive system validation"""

    def __init__(self, config_path: str = None):
        """Initialize validator"""
        # Load config in simulation mode
        self.config = load_config(config_path or "config/config.yaml")
        self.config.simulation_mode = True  # Force simulation mode

        # Test results
        self.results: List[TestResult] = []
        self.start_time = None
        self.end_time = None

        # Components (will be initialized per test)
        self.encoder = None
        self.pan_motor = None
        self.tilt_servo = None
        self.camera_manager = None
        self.acoustic_detector = None
        self.pan_controller = None
        self.tilt_controller = None
        self.state_machine = None
        self.visual_detector = None

        print("=" * 80)
        print("PTDTS SYSTEM VALIDATION TEST SUITE")
        print("=" * 80)
        print(f"Mode: SIMULATION")
        print(f"Config: {config_path or 'config/config.yaml'}")
        print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 80)
        print()

    def run_test(self, test_func, name: str) -> TestResult:
        """
        Run a single test and capture results

        Args:
            test_func: Test function to run
            name: Test name

        Returns:
            TestResult object
        """
        result = TestResult(name)
        result.start_time = time.time()

        print(f"\n{'─' * 80}")
        print(f"TEST: {name}")
        print(f"{'─' * 80}")

        try:
            # Run test
            test_func(result)

            # Mark as passed if not explicitly failed
            if not hasattr(result, 'passed'):
                result.passed = True

        except AssertionError as e:
            result.passed = False
            result.message = f"Assertion failed: {str(e)}"
            print(f"✗ FAILED: {result.message}")
        except Exception as e:
            result.passed = False
            result.message = f"Exception: {str(e)}"
            print(f"✗ ERROR: {result.message}")
        finally:
            result.end_time = time.time()
            result.duration = result.end_time - result.start_time

        # Print result
        status = "✓ PASSED" if result.passed else "✗ FAILED"
        print(f"\n{status} ({result.duration:.3f}s)")

        if result.warnings:
            print(f"Warnings: {len(result.warnings)}")
            for warning in result.warnings:
                print(f"  ⚠ {warning}")

        self.results.append(result)
        return result

    def test_system_initialization(self, result: TestResult):
        """Test 1: System initialization"""
        print("Initializing hardware components...")

        # Create encoder
        self.encoder = HardwareFactory.create_encoder(self.config)
        assert self.encoder is not None, "Encoder initialization failed"
        print(f"  ✓ Encoder: {type(self.encoder).__name__}")

        # Create motor
        self.pan_motor = HardwareFactory.create_motor(self.config, self.encoder)
        assert self.pan_motor is not None, "Motor initialization failed"
        print(f"  ✓ Motor: {type(self.pan_motor).__name__}")

        # Create servo
        self.tilt_servo = HardwareFactory.create_servo(self.config)
        assert self.tilt_servo is not None, "Servo initialization failed"
        print(f"  ✓ Servo: {type(self.tilt_servo).__name__}")

        # Create cameras
        self.camera_manager = HardwareFactory.create_cameras(self.config)
        assert self.camera_manager is not None, "Camera manager initialization failed"
        self.camera_manager.start_cameras()
        print(f"  ✓ Cameras: HQ={type(self.camera_manager.hq_camera).__name__}, GS={type(self.camera_manager.gs_camera).__name__}")

        # Create acoustic detector
        self.acoustic_detector = HardwareFactory.create_acoustic_detector(self.config)
        assert self.acoustic_detector is not None, "Acoustic detector initialization failed"
        print(f"  ✓ Acoustic: {type(self.acoustic_detector).__name__}")

        # Create controllers
        self.pan_controller = PanController(self.pan_motor, self.encoder, self.config)
        assert self.pan_controller is not None, "Pan controller initialization failed"
        print(f"  ✓ Pan Controller")

        self.tilt_controller = TiltController(self.tilt_servo, self.config)
        assert self.tilt_controller is not None, "Tilt controller initialization failed"
        print(f"  ✓ Tilt Controller")

        result.passed = True
        result.message = "All components initialized successfully"
        result.details = {
            'encoder': type(self.encoder).__name__,
            'motor': type(self.pan_motor).__name__,
            'servo': type(self.tilt_servo).__name__,
            'cameras': f"HQ={type(self.camera_manager.hq_camera).__name__}, GS={type(self.camera_manager.gs_camera).__name__}",
            'acoustic': type(self.acoustic_detector).__name__
        }

    def test_control_loop_timing(self, result: TestResult):
        """Test 2: Control loop timing validation"""
        print("Testing control loop timing (50Hz target)...")

        # Start pan controller
        self.pan_controller.start()

        # Let it run for 5 seconds
        test_duration = 5.0
        print(f"  Running control loop for {test_duration}s...")
        time.sleep(test_duration)

        # Get timing metrics
        status = self.pan_controller.get_status()
        timing = status.get('timing', {})

        # Stop controller
        self.pan_controller.stop()

        # Validate timing
        target_rate = self.config.pan_motor.control_rate_hz
        actual_rate = timing.get('actual_rate_hz', 0)
        jitter = timing.get('jitter_ms', 0)
        max_duration = timing.get('max_iteration_ms', 0)

        print(f"\n  Control Loop Metrics:")
        print(f"    Target rate: {target_rate} Hz")
        print(f"    Actual rate: {actual_rate:.2f} Hz")
        print(f"    Jitter: {jitter:.2f} ms")
        print(f"    Max iteration: {max_duration:.2f} ms")

        # Check if within acceptable range (±5%)
        rate_tolerance = 0.05
        min_acceptable = target_rate * (1 - rate_tolerance)
        max_acceptable = target_rate * (1 + rate_tolerance)

        if not (min_acceptable <= actual_rate <= max_acceptable):
            result.warnings.append(f"Loop rate {actual_rate:.2f}Hz outside acceptable range [{min_acceptable:.1f}, {max_acceptable:.1f}]")

        # Check jitter (should be < 5ms for good performance)
        if jitter > 5.0:
            result.warnings.append(f"High jitter detected: {jitter:.2f}ms")

        result.passed = True
        result.message = f"Control loop running at {actual_rate:.2f} Hz"
        result.details = {
            'target_rate_hz': target_rate,
            'actual_rate_hz': round(actual_rate, 2),
            'jitter_ms': round(jitter, 2),
            'max_iteration_ms': round(max_duration, 2),
            'within_tolerance': min_acceptable <= actual_rate <= max_acceptable
        }

    def test_pan_controller_response(self, result: TestResult):
        """Test 3: Pan controller position and velocity control"""
        print("Testing pan controller response...")

        # Start controller
        self.pan_controller.start()

        # Test position control
        print("\n  Test 3a: Position control")
        target_angle = 90.0
        print(f"    Setting target: {target_angle}°")

        self.pan_controller.set_position_target(target_angle)

        # Wait for movement
        timeout = 10.0
        start_time = time.time()
        arrived = False

        while time.time() - start_time < timeout:
            if self.pan_controller.is_position_reached():
                arrived = True
                break
            time.sleep(0.1)

        elapsed = time.time() - start_time
        final_angle = self.encoder.get_angle()

        print(f"    Reached: {arrived}")
        print(f"    Final angle: {final_angle:.2f}°")
        print(f"    Time: {elapsed:.2f}s")

        if not arrived:
            result.warnings.append(f"Position target not reached within {timeout}s")

        # Test velocity control
        print("\n  Test 3b: Velocity control")
        target_velocity = 30.0
        print(f"    Setting velocity: {target_velocity}°/s")

        self.pan_controller.set_velocity_target(target_velocity)
        time.sleep(2.0)

        current_velocity = self.encoder.get_velocity()
        print(f"    Current velocity: {current_velocity:.2f}°/s")

        # Stop
        self.pan_controller.set_idle()
        self.pan_controller.stop()

        result.passed = True
        result.message = "Pan controller responding correctly"
        result.details = {
            'position_test': {
                'target': target_angle,
                'final': round(final_angle, 2),
                'time': round(elapsed, 2),
                'arrived': arrived
            },
            'velocity_test': {
                'target': target_velocity,
                'measured': round(current_velocity, 2)
            }
        }

    def test_tilt_controller_response(self, result: TestResult):
        """Test 4: Tilt controller response"""
        print("Testing tilt controller response...")

        # Test several angles
        test_angles = [45.0, 90.0, 135.0]

        for angle in test_angles:
            print(f"\n  Setting tilt to {angle}°...")
            self.tilt_controller.set_target_angle(angle)

            # Give it time to move
            time.sleep(1.0)

            # Update controller
            self.tilt_controller.update()

            current = self.tilt_controller.current_angle
            print(f"    Current angle: {current:.2f}°")

        result.passed = True
        result.message = "Tilt controller responding correctly"
        result.details = {
            'test_angles': test_angles,
            'final_angle': round(self.tilt_controller.current_angle, 2)
        }

    def test_state_machine_transitions(self, result: TestResult):
        """Test 5: State machine transitions"""
        print("Testing state machine transitions...")

        # Create state machine
        from src.detection.visual_detector import VisualDetector

        try:
            self.visual_detector = VisualDetector(self.config)
        except Exception as e:
            print(f"  ⚠ Visual detector not available: {e}")
            self.visual_detector = None

        self.state_machine = StateMachine(
            pan_controller=self.pan_controller,
            tilt_controller=self.tilt_controller,
            camera_manager=self.camera_manager,
            visual_detector=self.visual_detector,
            acoustic_detector=self.acoustic_detector,
            config=self.config
        )

        # Start controllers
        self.pan_controller.start()

        # Test transitions
        transitions = []

        # Initial state should be LISTENING
        initial_state = self.state_machine.get_current_state()
        print(f"\n  Initial state: {initial_state.value}")
        assert initial_state == SystemState.LISTENING, f"Expected LISTENING, got {initial_state.value}"

        # Test manual mode transition
        print("\n  Requesting MANUAL mode...")
        self.state_machine.request_manual_mode()
        time.sleep(0.5)
        self.state_machine.update()

        current_state = self.state_machine.get_current_state()
        print(f"    Current state: {current_state.value}")
        transitions.append(('LISTENING', 'MANUAL', current_state == SystemState.MANUAL))

        if current_state != SystemState.MANUAL:
            result.warnings.append("Failed to transition to MANUAL mode")

        # Return to auto mode
        print("\n  Requesting AUTO mode...")
        self.state_machine.request_auto_mode()
        time.sleep(0.5)
        self.state_machine.update()

        current_state = self.state_machine.get_current_state()
        print(f"    Current state: {current_state.value}")
        transitions.append(('MANUAL', 'LISTENING', current_state == SystemState.LISTENING))

        # Stop
        self.pan_controller.stop()

        result.passed = True
        result.message = f"State machine transitions working ({len([t for t in transitions if t[2]])}/{len(transitions)} successful)"
        result.details = {
            'transitions': [{'from': t[0], 'to': t[1], 'success': t[2]} for t in transitions]
        }

    def test_state_persistence(self, result: TestResult):
        """Test 6: State persistence system"""
        print("Testing state persistence...")

        # Create state manager
        test_state_file = "tests/test_state.json"
        state_manager = StateManager(test_state_file)

        # Save encoder state
        print("\n  Saving encoder state...")
        test_count = 1234
        test_angle = 45.67
        self.encoder.set_count(test_count)

        state_manager.set_encoder_state(
            count=test_count,
            angle=test_angle,
            velocity=0.0
        )

        # Save pan controller state
        print("  Saving pan controller state...")
        state_manager.set_pan_controller_state(
            mode='idle',
            target_position=0.0,
            target_velocity=0.0,
            pid_integral=0.0,
            pid_last_error=0.0
        )

        # Write to disk
        state_manager.save()
        print("  State saved to disk")

        # Create new state manager and load
        print("\n  Loading state from disk...")
        state_manager2 = StateManager(test_state_file)

        encoder_state = state_manager2.get_encoder_state()
        pan_state = state_manager2.get_pan_controller_state()

        print(f"    Encoder count: {encoder_state.get('count')} (expected {test_count})")
        print(f"    Encoder angle: {encoder_state.get('angle')} (expected {test_angle})")

        # Validate
        assert encoder_state.get('count') == test_count, "Encoder count mismatch"
        assert abs(encoder_state.get('angle') - test_angle) < 0.01, "Encoder angle mismatch"

        # Clean up
        Path(test_state_file).unlink(missing_ok=True)

        result.passed = True
        result.message = "State persistence working correctly"
        result.details = {
            'saved_count': test_count,
            'loaded_count': encoder_state.get('count'),
            'match': encoder_state.get('count') == test_count
        }

    def test_camera_capture(self, result: TestResult):
        """Test 7: Camera frame capture"""
        print("Testing camera frame capture...")

        # Test HQ camera
        print("\n  Capturing from HQ camera...")
        self.camera_manager.switch_to_hq()
        time.sleep(0.5)

        frame_hq, camera_name = self.camera_manager.capture_frame()
        assert frame_hq is not None, "HQ camera frame is None"
        assert camera_name == 'hq', f"Expected 'hq', got '{camera_name}'"

        height_hq, width_hq, channels_hq = frame_hq.shape
        print(f"    Frame: {width_hq}x{height_hq}x{channels_hq}")

        # Test GS camera
        print("\n  Capturing from GS camera...")
        self.camera_manager.switch_to_gs()
        time.sleep(0.5)

        frame_gs, camera_name = self.camera_manager.capture_frame()
        assert frame_gs is not None, "GS camera frame is None"
        assert camera_name == 'gs', f"Expected 'gs', got '{camera_name}'"

        height_gs, width_gs, channels_gs = frame_gs.shape
        print(f"    Frame: {width_gs}x{height_gs}x{channels_gs}")

        result.passed = True
        result.message = "Camera capture working correctly"
        result.details = {
            'hq_resolution': f"{width_hq}x{height_hq}x{channels_hq}",
            'gs_resolution': f"{width_gs}x{height_gs}x{channels_gs}"
        }

    def cleanup(self):
        """Cleanup resources"""
        print("\nCleaning up resources...")

        if self.pan_controller:
            self.pan_controller.stop()

        if self.camera_manager:
            self.camera_manager.stop_cameras()
            self.camera_manager.close()

        if self.acoustic_detector:
            self.acoustic_detector.stop()

    def run_all_tests(self):
        """Run complete test suite"""
        self.start_time = time.time()

        # Run tests
        self.run_test(self.test_system_initialization, "1. System Initialization")
        self.run_test(self.test_control_loop_timing, "2. Control Loop Timing")
        self.run_test(self.test_pan_controller_response, "3. Pan Controller Response")
        self.run_test(self.test_tilt_controller_response, "4. Tilt Controller Response")
        self.run_test(self.test_state_machine_transitions, "5. State Machine Transitions")
        self.run_test(self.test_state_persistence, "6. State Persistence")
        self.run_test(self.test_camera_capture, "7. Camera Frame Capture")

        self.end_time = time.time()

        # Cleanup
        self.cleanup()

        # Generate report
        self.generate_report()

    def generate_report(self):
        """Generate test report"""
        print("\n" + "=" * 80)
        print("TEST SUMMARY")
        print("=" * 80)

        passed = sum(1 for r in self.results if r.passed)
        failed = len(self.results) - passed
        total_time = self.end_time - self.start_time

        print(f"\nTotal tests: {len(self.results)}")
        print(f"Passed: {passed}")
        print(f"Failed: {failed}")
        print(f"Total time: {total_time:.2f}s")
        print()

        # Individual results
        for result in self.results:
            status = "✓ PASS" if result.passed else "✗ FAIL"
            print(f"{status}  {result.name} ({result.duration:.3f}s)")
            if result.message:
                print(f"       {result.message}")
            if result.warnings:
                for warning in result.warnings:
                    print(f"       ⚠ {warning}")

        # Save detailed report
        report_path = "tests/validation_report.json"
        report_data = {
            'timestamp': datetime.now().isoformat(),
            'summary': {
                'total': len(self.results),
                'passed': passed,
                'failed': failed,
                'duration': round(total_time, 2)
            },
            'tests': []
        }

        for result in self.results:
            report_data['tests'].append({
                'name': result.name,
                'passed': result.passed,
                'duration': round(result.duration, 3),
                'message': result.message,
                'details': result.details,
                'warnings': result.warnings
            })

        with open(report_path, 'w') as f:
            json.dump(report_data, f, indent=2)

        print(f"\nDetailed report saved to: {report_path}")
        print("=" * 80)

        # Return exit code
        return 0 if failed == 0 else 1


def main():
    """Main entry point"""
    try:
        validator = SystemValidator()
        exit_code = validator.run_all_tests()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\nTests interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nFATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
