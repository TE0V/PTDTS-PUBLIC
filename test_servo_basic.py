#!/usr/bin/env python3
"""
ULTRA-BASIC SERVO TEST
Just sends center pulse (1500us) to servo to trigger beep

This is the absolute simplest test - if this doesn't work, there's a wiring issue.
"""

import time
import sys

print("="*70)
print("ULTRA-BASIC SERVO TEST - GPIO 18 HARDWARE PWM")
print("="*70)
print()
print("This test sends a simple 1500µs pulse to the servo.")
print("The servo SHOULD beep immediately when it receives this signal.")
print()
print("WIRING CHECK:")
print("  Servo SIGNAL wire → Pin 12 (GPIO 18) ← MUST BE THIS PIN!")
print("  Servo GROUND wire → Any GND pin")
print("  Servo POWER wire  → External 6V supply (NOT Pi)")
print()
print("=" * 70)
print()

response = input("Have you confirmed servo signal is on Pin 12 (GPIO 18)? (y/n): ")
if response.lower() != 'y':
    print("\nPlease rewire servo signal to Pin 12 (GPIO 18) first!")
    print("Pin 12 is in the SECOND column, 6th pin from top (next to Pin 11)")
    sys.exit(1)

print()
print("Initializing GPIO 18 with hardware PWM...")

try:
    from gpiozero import Servo
    from gpiozero.pins.lgpio import LGPIOFactory

    factory = LGPIOFactory()
    print("✓ LGPIOFactory initialized")

    # Create servo on GPIO 18 (hardware PWM pin)
    servo = Servo(
        18,  # GPIO 18 = Pin 12 = HARDWARE PWM
        min_pulse_width=0.0005,   # 500µs
        max_pulse_width=0.0025,   # 2500µs
        frame_width=1.0/333,      # 333Hz
        pin_factory=factory
    )
    print("✓ Servo object created on GPIO 18")

    # TEST 1: Static center position (some servos beep immediately)
    print()
    print("TEST 1: Sending static center pulse (1500µs)...")
    servo.value = 0  # 0 = center in gpiozero
    print("*** LISTEN FOR BEEP NOW ***")

    for i in range(3, 0, -1):
        print(f"  {i}...", end='', flush=True)
        time.sleep(1)
    print()

    beep_check = input("Did servo beep? (y/n): ")
    if beep_check.lower() == 'y':
        print("✓ Servo responded to static pulse!")
        servo.close()
        print()
        print("SUCCESS! Servo is working. You can now run test_tilt_servo.py")
        sys.exit(0)

    # TEST 2: Movement sweep (some servos only beep when they move)
    print()
    print("TEST 2: Trying movement sequence...")
    print("Moving servo through positions: 0° → 45° → 90° → 45° → 0°")
    print("*** LISTEN FOR BEEP DURING MOVEMENT ***")
    print()

    positions = [-1, -0.5, 0, 0.5, 1, 0.5, 0, -0.5, -1, 0]  # Full sweep
    position_labels = ["0°", "22.5°", "45°", "67.5°", "90°", "67.5°", "45°", "22.5°", "0°", "45°"]

    for pos, label in zip(positions, position_labels):
        print(f"  → {label}...", flush=True)
        servo.value = pos
        time.sleep(0.5)  # Slower movement so you can hear beep

    print()
    print("Movement sequence complete.")
    print()
    print("=" * 70)
    print("TEST COMPLETE")
    print("=" * 70)
    print()

    response = input("Did the servo beep? (y/n): ")

    if response.lower() == 'y':
        print()
        print("✓ SUCCESS! Servo is receiving PWM signal correctly.")
        print("  GPIO 18 hardware PWM is working.")
        print("  You can now run: sudo python3 test_tilt_servo.py")
    else:
        print()
        print("✗ PROBLEM: Servo did not beep.")
        print()
        print("TROUBLESHOOTING:")
        print("  1. VERIFY WIRING:")
        print("     - Servo signal wire is physically connected to Pin 12?")
        print("     - Not Pin 22 (old location)?")
        print("     - Pin 12 is 2nd column, 6th from top")
        print()
        print("  2. CHECK POWER:")
        print("     - Is servo powered with external 6V supply?")
        print("     - Is ground shared between Pi and servo power?")
        print()
        print("  3. TEST WITH ORIGINAL CONTROLLER:")
        print("     - Does servo beep/work with its own controller?")
        print("     - If yes → wiring issue with Pi")
        print("     - If no → servo or power issue")
        print()
        print("  4. VERIFY PIN 12:")
        print("     - Looking at Pi with USB ports at bottom")
        print("     - Pin 12 is on RIGHT side, 6th from top")
        print("     - Next to Pin 11 (left side)")

    servo.close()

except Exception as e:
    print(f"✗ ERROR: {e}")
    import traceback
    traceback.print_exc()
    print()
    print("If you see a permissions error, run with sudo:")
    print("  sudo python3 test_servo_basic.py")

print()
