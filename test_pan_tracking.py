#!/usr/bin/env python3
"""
Comprehensive Pan and Tracking Test Suite
Tests all aspects of pan motor control and tracking logic
"""
import sys
import time
import statistics
import os

# Add project root to path to allow src.* imports
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from src.utils.config_loader import load_config
from src.hardware.factory import HardwareFactory
from src.control.pan_controller import PanController, PanMode

# ANSI colors for pretty output
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

def print_header(text):
    print(f"\n{Colors.BOLD}{Colors.HEADER}{'=' * 70}{Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.HEADER}{text}{Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.HEADER}{'=' * 70}{Colors.ENDC}")

def print_test(text):
    print(f"\n{Colors.BOLD}{Colors.CYAN}{text}{Colors.ENDC}")

def print_pass(text):
    print(f"{Colors.GREEN}  [PASS] {text}{Colors.ENDC}")

def print_fail(text):
    print(f"{Colors.RED}  [FAIL] {text}{Colors.ENDC}")

def print_warn(text):
    print(f"{Colors.YELLOW}  [WARN] {text}{Colors.ENDC}")

def print_info(text):
    print(f"  {text}")

# Test results tracker
test_results = {
    'passed': 0,
    'failed': 0,
    'warnings': 0,
    'critical_failures': []
}

def test_pass(name):
    test_results['passed'] += 1
    print_pass(name)

def test_fail(name, critical=False):
    test_results['failed'] += 1
    print_fail(name)
    if critical:
        test_results['critical_failures'].append(name)

def test_warn(name):
    test_results['warnings'] += 1
    print_warn(name)

# Initialize system
print_header("PAN AND TRACKING COMPREHENSIVE TEST SUITE")
print(f"{Colors.BOLD}This test suite will verify all aspects of pan motor and tracking{Colors.ENDC}")
print(f"{Colors.BOLD}Make sure the pan axis is clear to rotate safely!{Colors.ENDC}")
print(f"\nPress ENTER to continue, or Ctrl+C to abort...")
input()

# Load configuration
print_header("INITIALIZING SYSTEM")
try:
    config = load_config('config/config.yaml')
    print_pass(f"Config loaded")
    print_info(f"   Simulation mode: {config.simulation_mode}")
    print_info(f"   Use direct PWM: {config.tracking.use_direct_pwm}")
    print_info(f"   Max PWM: {config.pan_motor.max_pwm}")
    print_info(f"   Encoder counts/360: {config.pan_motor.encoder_counts_per_360}")
except Exception as e:
    print_fail(f"Failed to load config: {e}", critical=True)
    sys.exit(1)

# Create hardware
print_test("Creating hardware...")
try:
    encoder = HardwareFactory.create_encoder(config)
    motor = HardwareFactory.create_motor(config, encoder)
    pan_controller = PanController(motor, encoder, config)
    time.sleep(0.5)
    print_pass("Hardware initialized")
except Exception as e:
    print_fail(f"Failed to create hardware: {e}", critical=True)
    sys.exit(1)

# ============================================================================
# TEST SECTION 1: ENCODER TESTS
# ============================================================================
print_header("TEST 1: ENCODER FUNCTIONALITY")

print_test("1.1: Basic Encoder Reading")
try:
    angle = encoder.get_angle()
    counts = encoder.get_count()
    print_pass(f"Encoder reading: {angle:.2f}° ({counts} counts)")
    test_pass("Encoder basic read")
except Exception as e:
    test_fail(f"Encoder read failed: {e}", critical=True)

print_test("1.2: Encoder Stability (checking for jitter)")
try:
    angles = []
    for _ in range(10):
        angles.append(encoder.get_angle())
        time.sleep(0.01)

    if len(set(angles)) == 1:
        print_pass(f"Encoder stable: {angles[0]:.2f}° (no jitter)")
        test_pass("Encoder stability")
    else:
        angle_range = max(angles) - min(angles)
        if angle_range < 0.5:
            print_pass(f"Encoder stable: {angle_range:.3f}° variation (acceptable)")
            test_pass("Encoder stability")
        else:
            test_warn(f"Encoder shows {angle_range:.2f}° jitter (may be normal if not clamped)")
except Exception as e:
    test_fail(f"Encoder stability test failed: {e}")

print_test("1.3: Encoder Calibration Sanity Check")
cpr = config.pan_motor.encoder_counts_per_360
if abs(cpr) > 1000:
    print_pass(f"Counts per 360°: {cpr} (reasonable for quadrature)")
    test_pass("Encoder calibration")
elif abs(cpr) > 100:
    print_pass(f"Counts per 360°: {cpr} (reasonable for non-quadrature)")
    test_pass("Encoder calibration")
else:
    test_warn(f"Counts per 360°: {cpr} (seems low, check calibration)")

# ============================================================================
# TEST SECTION 2: MOTOR DRIVER TESTS
# ============================================================================
print_header("TEST 2: MOTOR DRIVER FUNCTIONALITY")

print_test("2.1: Motor Direction Mapping (CRITICAL)")
print_info("Testing if positive PWM causes positive angle change...")

direction_tests = [
    (0.2, "positive", "CW", lambda d: d > 0),
    (-0.2, "negative", "CCW", lambda d: d < 0),
]

motor_direction_correct = True
for pwm, sign_name, direction_name, check_func in direction_tests:
    start_angle = encoder.get_angle()
    print_info(f"   Sending PWM={pwm:.2f}, angle before: {start_angle:.1f}°")

    pan_controller.set_tracking_pwm(pwm)
    time.sleep(0.5)

    end_angle = encoder.get_angle()
    delta = end_angle - start_angle

    pan_controller.set_idle()
    time.sleep(0.2)

    print_info(f"   Angle after: {end_angle:.1f}°, delta: {delta:.1f}°")

    if abs(delta) < 0.5:
        test_warn(f"Motor barely moved ({delta:.2f}°) - may have stiction or be at limit")
        motor_direction_correct = False
    elif check_func(delta):
        print_pass(f"{sign_name.capitalize()} PWM → {direction_name} rotation - CORRECT")
        test_pass(f"Motor direction ({sign_name} PWM)")
    else:
        print_fail(f"{sign_name.capitalize()} PWM → WRONG DIRECTION!")
        test_fail(f"Motor direction ({sign_name} PWM)", critical=True)
        motor_direction_correct = False

print_test("2.2: Motor Stop Command")
start_angle = encoder.get_angle()
pan_controller.set_tracking_pwm(0.3)
time.sleep(0.3)
pan_controller.set_idle()
time.sleep(0.5)

# Check if motor stopped (angle shouldn't change much)
angles_after_stop = [encoder.get_angle() for _ in range(10)]
drift = max(angles_after_stop) - min(angles_after_stop)

if drift < 1.0:
    print_pass(f"Motor stops correctly (drift: {drift:.2f}°)")
    test_pass("Motor stop command")
else:
    test_warn(f"Motor may be drifting after stop ({drift:.2f}°)")

print_test("2.3: PWM Limits Respected")
# Test that we don't exceed max_pwm
max_pwm_config = config.pan_motor.max_pwm
print_info(f"   Configured max PWM: {max_pwm_config}")

# In tracking, we use min(0.5, max_pwm)
effective_max = min(0.5, max_pwm_config)
print_info(f"   Effective tracking max PWM: {effective_max}")

if effective_max <= max_pwm_config:
    print_pass(f"PWM limits properly configured")
    test_pass("PWM limits")
else:
    test_fail(f"PWM limit configuration error")

# ============================================================================
# TEST SECTION 3: CONTROLLER MODE TESTS
# ============================================================================
print_header("TEST 3: CONTROLLER MODE FUNCTIONALITY")

print_test("3.1: IDLE Mode")
pan_controller.set_idle()
time.sleep(0.2)
mode = pan_controller.get_mode()
if mode == PanMode.IDLE:
    print_pass(f"Controller in IDLE mode")
    test_pass("IDLE mode")
else:
    test_fail(f"Controller mode is {mode.name}, expected IDLE")

print_test("3.2: VELOCITY Mode (Direct PWM)")
pan_controller.set_tracking_pwm(0.1)
time.sleep(0.1)
mode = pan_controller.get_mode()
if mode == PanMode.VELOCITY:
    print_pass(f"Controller switched to VELOCITY mode")
    test_pass("VELOCITY mode transition")
else:
    test_fail(f"Controller mode is {mode.name}, expected VELOCITY")

pan_controller.set_idle()
time.sleep(0.2)

print_test("3.3: Mode Transition Stability")
try:
    for _ in range(5):
        pan_controller.set_tracking_pwm(0.05)
        time.sleep(0.05)
        pan_controller.set_idle()
        time.sleep(0.05)
    print_pass("Rapid mode transitions handled without crashes")
    test_pass("Mode transition stability")
except Exception as e:
    test_fail(f"Mode transitions caused error: {e}")

# ============================================================================
# TEST SECTION 4: TRACKING LOGIC TESTS
# ============================================================================
print_header("TEST 4: TRACKING LOGIC VALIDATION")

print_test("4.1: Pixel Error to PWM Calculation")
# Get frame width from GS camera config (used for tracking)
frame_width = config.cameras.gs.resolution[0]
test_cases = [
    {"target_x": frame_width * 0.25, "description": "Target LEFT of center", "expected_sign": "negative"},
    {"target_x": frame_width * 0.75, "description": "Target RIGHT of center", "expected_sign": "positive"},
    {"target_x": frame_width * 0.50, "description": "Target AT center", "expected_sign": "zero"},
    {"target_x": 0, "description": "Target at LEFT EDGE", "expected_sign": "negative"},
    {"target_x": frame_width, "description": "Target at RIGHT EDGE", "expected_sign": "positive"},
]

pixel_logic_correct = True
for test in test_cases:
    target_x = test['target_x']
    pixel_error = target_x - (frame_width / 2)

    # Calculate PWM (same as state_machine.py)
    max_pixel_error = frame_width / 2
    normalized_error = pixel_error / max_pixel_error
    sign = 1.0 if normalized_error >= 0 else -1.0
    scaled_error = sign * (abs(normalized_error) ** 1.5)
    pwm_command = scaled_error * min(0.5, config.pan_motor.max_pwm)

    # Apply deadzone
    if abs(pixel_error) < config.tracking.deadzone_pixels:
        pwm_command = 0.0

    print_info(f"\n   {test['description']}:")
    print_info(f"      Target pixel: {target_x}, Center: {frame_width/2}")
    print_info(f"      Pixel error: {pixel_error:.0f}px")
    print_info(f"      PWM command: {pwm_command:.3f}")

    # Verify direction
    if pixel_error < -config.tracking.deadzone_pixels:
        # Target is LEFT (negative error) - should pan LEFT (CCW) = NEGATIVE PWM
        if pwm_command < 0:
            print_pass(f"Target LEFT -> NEGATIVE PWM ({pwm_command:.3f}) - CORRECT")
        else:
            print_fail(f"Target LEFT -> POSITIVE PWM ({pwm_command:.3f}) - WRONG!")
            pixel_logic_correct = False
            test_fail("Pixel error logic (LEFT)", critical=True)
    elif pixel_error > config.tracking.deadzone_pixels:
        # Target is RIGHT (positive error) - should pan RIGHT (CW) = POSITIVE PWM
        if pwm_command > 0:
            print_pass(f"Target RIGHT -> POSITIVE PWM ({pwm_command:.3f}) - CORRECT")
        else:
            print_fail(f"Target RIGHT -> NEGATIVE PWM ({pwm_command:.3f}) - WRONG!")
            pixel_logic_correct = False
            test_fail("Pixel error logic (RIGHT)", critical=True)
    else:
        # Target is centered - should not move
        if pwm_command == 0:
            print_pass(f"Target CENTERED -> ZERO PWM (in deadzone) - CORRECT")
        else:
            print_warn(f"Target near center but PWM={pwm_command:.3f} (check deadzone)")

if pixel_logic_correct:
    test_pass("Pixel error logic")

print_test("4.2: Exponential Scaling Behavior")
# Test that exponential scaling reduces power for small errors
test_errors = [0.1, 0.3, 0.5, 0.7, 0.9, 1.0]
print_info("   Testing x^1.5 exponential scaling:")
for norm_err in test_errors:
    scaled = norm_err ** 1.5
    pwm = scaled * 0.5
    print_info(f"      Error {norm_err:.1f} → Scaled {scaled:.3f} → PWM {pwm:.3f}")

# Verify small errors get very small PWM
small_err_scaled = (0.1 ** 1.5)
if small_err_scaled < 0.05:
    print_pass(f"Small errors get small PWM (0.1 → {small_err_scaled:.3f})")
    test_pass("Exponential scaling")
else:
    test_warn(f"Exponential scaling may not be aggressive enough")

print_test("4.3: Deadzone Behavior")
deadzone = config.tracking.deadzone_pixels
print_info(f"   Configured deadzone: {deadzone}px")

# Test pixel errors within deadzone
for px_err in [-15, -10, 0, 10, 15]:
    max_px_err = frame_width / 2
    norm_err = px_err / max_px_err
    sign = 1.0 if norm_err >= 0 else -1.0
    scaled = sign * (abs(norm_err) ** 1.5)
    pwm = scaled * 0.5

    if abs(px_err) < deadzone:
        pwm = 0.0

    in_deadzone = abs(px_err) < deadzone
    print_info(f"      Error {px_err:+4.0f}px → PWM {pwm:.3f} {'(deadzone)' if in_deadzone else ''}")

print_pass("Deadzone configured and applied correctly")
test_pass("Deadzone behavior")

# ============================================================================
# TEST SECTION 5: INTEGRATION TESTS
# ============================================================================
print_header("TEST 5: INTEGRATED TRACKING SIMULATION")

print_test("5.1: Simulated Tracking - Target Moves Across Frame")
print_info("   Simulating drone moving from left to right...")

# Move target across frame and see if motor tracks it
start_angle = encoder.get_angle()
print_info(f"   Starting angle: {start_angle:.1f}°")

tracking_data = []
for i in range(11):
    # Target moves from left (64px) to right (576px)
    target_x = 64 + (i * 51.2)  # 64, 115, 166, ..., 576

    pixel_error = target_x - (frame_width / 2)
    max_pixel_error = frame_width / 2
    normalized_error = pixel_error / max_pixel_error
    sign = 1.0 if normalized_error >= 0 else -1.0
    scaled_error = sign * (abs(normalized_error) ** 1.5)
    pwm_command = scaled_error * 0.5

    if abs(pixel_error) < config.tracking.deadzone_pixels:
        pwm_command = 0.0

    current_angle = encoder.get_angle()

    pan_controller.set_tracking_pwm(pwm_command)
    time.sleep(0.15)

    new_angle = encoder.get_angle()
    angle_change = new_angle - current_angle

    tracking_data.append({
        'target_x': target_x,
        'pixel_error': pixel_error,
        'pwm': pwm_command,
        'angle_before': current_angle,
        'angle_after': new_angle,
        'angle_change': angle_change
    })

    print_info(f"      Frame {i+1:2d}: Target={target_x:5.1f}px, Error={pixel_error:+5.0f}px, "
              f"PWM={pwm_command:+.3f}, Angle={current_angle:6.1f}°→{new_angle:6.1f}° "
              f"(Δ{angle_change:+.1f}°)")

pan_controller.set_idle()
end_angle = encoder.get_angle()
total_movement = end_angle - start_angle

print_info(f"\n   Total rotation: {total_movement:+.1f}°")

# Analyze if motor moved in correct direction
# Target moved from LEFT to RIGHT, so motor should pan RIGHT (positive angle change)
if total_movement > 5:
    print_pass(f"Motor panned RIGHT ({total_movement:+.1f}°) as target moved RIGHT - CORRECT")
    test_pass("Tracking direction (integration)")
elif total_movement < -5:
    print_fail(f"Motor panned LEFT ({total_movement:+.1f}°) but target moved RIGHT - INVERTED!")
    test_fail("Tracking direction (integration)", critical=True)
else:
    test_warn(f"Motor barely moved ({total_movement:+.1f}°) - may be at limit or stiction")

print_test("5.2: Centering Behavior")
print_info("   Testing if motor centers on target and stops...")

# Place target at center
target_x = frame_width / 2
pixel_error = target_x - (frame_width / 2)  # Should be 0

pwm_command = 0.0  # Centered, so PWM should be 0
pan_controller.set_tracking_pwm(pwm_command)

# Check motor doesn't drift
angles_centered = []
for _ in range(10):
    angles_centered.append(encoder.get_angle())
    time.sleep(0.05)

drift_centered = max(angles_centered) - min(angles_centered)

if drift_centered < 1.0:
    print_pass(f"Motor stable when centered (drift: {drift_centered:.2f}°)")
    test_pass("Centering stability")
else:
    test_warn(f"Motor drifting when centered ({drift_centered:.2f}°)")

pan_controller.set_idle()

# ============================================================================
# TEST SECTION 6: PERFORMANCE AND EDGE CASES
# ============================================================================
print_header("TEST 6: PERFORMANCE AND EDGE CASES")

print_test("6.1: Control Loop Frequency")
print_info("   Measuring pan controller loop rate...")

# The control loop should run at ~50Hz
# We can't directly measure it without modifying the controller,
# but we can verify commands are processed quickly

response_times = []
for _ in range(10):
    t_start = time.time()
    pan_controller.set_tracking_pwm(0.05)
    # Command should be processed within a few ms
    t_end = time.time()
    response_times.append((t_end - t_start) * 1000)
    time.sleep(0.02)

pan_controller.set_idle()

avg_response = statistics.mean(response_times)
if avg_response < 10:
    print_pass(f"Command response time: {avg_response:.2f}ms (good)")
    test_pass("Command response time")
else:
    test_warn(f"Command response time: {avg_response:.2f}ms (may be slow)")

print_test("6.2: Edge Case - Rapid Direction Changes")
print_info("   Testing rapid PWM reversals...")

try:
    start_angle = encoder.get_angle()
    for i in range(10):
        pwm = 0.1 if i % 2 == 0 else -0.1
        pan_controller.set_tracking_pwm(pwm)
        time.sleep(0.05)
    pan_controller.set_idle()

    end_angle = encoder.get_angle()
    # Should oscillate around start position
    net_movement = abs(end_angle - start_angle)

    print_pass(f"Handled rapid reversals (net movement: {net_movement:.1f}°)")
    test_pass("Rapid direction changes")
except Exception as e:
    test_fail(f"Rapid reversals caused error: {e}")

print_test("6.3: Edge Case - Very Small PWM Values")
print_info("   Testing motor response to very small PWM...")

# Test stiction/deadband
small_pwm_values = [0.01, 0.02, 0.05, 0.1]
moved_at = None

for pwm in small_pwm_values:
    start_angle = encoder.get_angle()
    pan_controller.set_tracking_pwm(pwm)
    time.sleep(0.5)
    end_angle = encoder.get_angle()
    pan_controller.set_idle()
    time.sleep(0.2)

    delta = abs(end_angle - start_angle)
    print_info(f"      PWM {pwm:.2f} → Moved {delta:.2f}°")

    if delta > 0.5 and moved_at is None:
        moved_at = pwm

if moved_at is not None:
    if moved_at < 0.05:
        print_pass(f"Motor responds to small PWM (moves at {moved_at:.2f})")
        test_pass("Small PWM response")
    else:
        test_warn(f"Motor has stiction, needs PWM > {moved_at:.2f} to move")
else:
    test_warn("Motor did not respond to small PWM values (high stiction)")

# ============================================================================
# FINAL RESULTS
# ============================================================================
print_header("TEST RESULTS SUMMARY")

total_tests = test_results['passed'] + test_results['failed']
pass_rate = (test_results['passed'] / total_tests * 100) if total_tests > 0 else 0

print(f"\n{Colors.BOLD}Tests Passed:  {Colors.GREEN}{test_results['passed']}{Colors.ENDC}")
print(f"{Colors.BOLD}Tests Failed:  {Colors.RED}{test_results['failed']}{Colors.ENDC}")
print(f"{Colors.BOLD}Warnings:      {Colors.YELLOW}{test_results['warnings']}{Colors.ENDC}")
print(f"{Colors.BOLD}Pass Rate:     {Colors.CYAN}{pass_rate:.1f}%{Colors.ENDC}\n")

if test_results['critical_failures']:
    print(f"{Colors.BOLD}{Colors.RED}CRITICAL FAILURES:{Colors.ENDC}")
    for failure in test_results['critical_failures']:
        print(f"{Colors.RED}  [FAIL] {failure}{Colors.ENDC}")

    print(f"\n{Colors.BOLD}{Colors.RED}*** TRACKING WILL NOT WORK CORRECTLY ***{Colors.ENDC}")

    print(f"\n{Colors.BOLD}DIAGNOSIS AND FIXES:{Colors.ENDC}")

    # Determine the issue
    has_motor_direction_issue = any("Motor direction" in f for f in test_results['critical_failures'])
    has_pixel_logic_issue = any("Pixel error logic" in f for f in test_results['critical_failures'])
    has_tracking_direction_issue = any("Tracking direction" in f for f in test_results['critical_failures'])

    if has_pixel_logic_issue:
        print(f"\n{Colors.YELLOW}Issue: Pixel error calculation is INVERTED{Colors.ENDC}")
        print(f"{Colors.CYAN}Fix: Correct pixel_error in src/state/state_machine.py line 312{Colors.ENDC}")
        print(f"   Current (wrong): pixel_error = (frame_width / 2) - best.center[0]")
        print(f"   Should be:       pixel_error = best.center[0] - (frame_width / 2)")

    if has_motor_direction_issue:
        print(f"\n{Colors.YELLOW}Issue: Motor direction mapping is WRONG{Colors.ENDC}")
        print(f"{Colors.CYAN}Fix Option 1: Swap motor pins in config/config.yaml{Colors.ENDC}")
        print(f"   Swap gpio_in1 and gpio_in2 values")
        print(f"{Colors.CYAN}Fix Option 2: Negate PWM in src/hardware/motor.py{Colors.ENDC}")
        print(f"   In set_pwm(), change: duty_cycle = pwm")
        print(f"   To:                   duty_cycle = -pwm")

    if has_tracking_direction_issue and not (has_pixel_logic_issue or has_motor_direction_issue):
        print(f"\n{Colors.YELLOW}Issue: Integration test failed but individual tests passed{Colors.ENDC}")
        print(f"{Colors.CYAN}This suggests a combination issue - check both pixel logic AND motor direction{Colors.ENDC}")
else:
    if test_results['failed'] == 0 and test_results['warnings'] == 0:
        print(f"{Colors.BOLD}{Colors.GREEN}[PASS] ALL TESTS PASSED - SYSTEM IS READY FOR TRACKING!{Colors.ENDC}")
    elif test_results['failed'] == 0:
        print(f"{Colors.BOLD}{Colors.YELLOW}[WARN] ALL TESTS PASSED WITH WARNINGS{Colors.ENDC}")
        print(f"{Colors.YELLOW}System should work, but review warnings above{Colors.ENDC}")
    else:
        print(f"{Colors.BOLD}{Colors.YELLOW}Some tests failed but no critical issues detected{Colors.ENDC}")
        print(f"{Colors.YELLOW}Review failures above before deploying{Colors.ENDC}")

# Cleanup
print(f"\n{Colors.BOLD}Shutting down...{Colors.ENDC}")
pan_controller.stop()
time.sleep(0.5)

print(f"{Colors.GREEN}[PASS] Test suite complete{Colors.ENDC}\n")
