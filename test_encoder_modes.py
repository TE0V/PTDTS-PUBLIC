#!/usr/bin/env python3
"""
LS7366R Encoder Mode Testing
Tests different quadrature modes to find the best working configuration
"""

import spidev
import time
import sys

SPI_BUS = 0
SPI_DEVICE = 0

# Commands
CMD_CLEAR_CNTR = 0x20
CMD_WRITE = 0x80
CMD_READ = 0x60
REG_MDR0 = 0x08
REG_CNTR = 0x20

def test_mode(spi, mode_name, mdr0_value, test_duration=10):
    """
    Test a specific encoder mode

    Args:
        spi: SPI device
        mode_name: Human-readable name
        mdr0_value: MDR0 register value to test
        test_duration: How long to test in seconds

    Returns:
        (success, max_count, notes)
    """
    print(f"\n{'='*60}")
    print(f"Testing: {mode_name}")
    print(f"MDR0: 0x{mdr0_value:02X} (0b{mdr0_value:08b})")
    print(f"{'='*60}")

    try:
        # Clear and configure
        spi.xfer2([CMD_CLEAR_CNTR])
        time.sleep(0.1)

        spi.xfer2([CMD_WRITE | REG_MDR0, mdr0_value])
        time.sleep(0.1)

        # Test for freezing - read multiple times
        print("Checking for freeze (reading 5 times)...")
        for i in range(5):
            data = spi.xfer2([CMD_READ | REG_CNTR, 0, 0, 0, 0])
            count = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]
            if count & 0x80000000:
                count -= 0x100000000
            print(f"  Read {i+1}: {count}")
            time.sleep(0.2)

        print(f"\n✓ Mode does not freeze! Now testing counting...")
        print(f"Manually rotate the motor shaft and watch for counts.")
        print(f"Testing for {test_duration} seconds...\n")

        start_time = time.time()
        last_count = 0
        max_count = 0
        count_changes = 0

        while time.time() - start_time < test_duration:
            data = spi.xfer2([CMD_READ | REG_CNTR, 0, 0, 0, 0])
            count = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]
            if count & 0x80000000:
                count -= 0x100000000

            if count != last_count:
                change = count - last_count
                count_changes += 1
                print(f"  Count: {count:6d}  (change: {change:+4d})")
                last_count = count
                max_count = max(max_count, abs(count))

            time.sleep(0.1)

        if count_changes == 0:
            print("\n⚠ WARNING: No counts detected during manual rotation!")
            return (False, 0, "No counts detected")
        else:
            print(f"\n✓ SUCCESS: Detected {count_changes} count changes")
            print(f"  Maximum count reached: {max_count}")
            return (True, max_count, f"{count_changes} changes detected")

    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return (False, 0, str(e))

def main():
    print("="*60)
    print("LS7366R ENCODER MODE TESTING")
    print("="*60)
    print("\nThis script will test different encoder modes to find")
    print("which ones work reliably on your hardware.")
    print("\nBe ready to manually rotate the motor shaft during tests.")
    print("\nPress ENTER to begin...")
    input()

    # Initialize SPI
    try:
        spi = spidev.SpiDev()
        spi.open(SPI_BUS, SPI_DEVICE)
        spi.max_speed_hz = 50000  # Use proven 50kHz speed
        spi.mode = 0
        print(f"✓ SPI {SPI_BUS}.{SPI_DEVICE} opened at 50 kHz\n")
    except Exception as e:
        print(f"✗ Failed to open SPI: {e}")
        return

    # Test modes in order of increasing complexity
    modes = [
        ("Non-Quadrature (current)", 0b00000000),
        ("x1 Quadrature", 0b00010000),
        ("x2 Quadrature", 0b00100000),
        ("x4 Quadrature", 0b00110000),
    ]

    results = []

    for mode_name, mdr0 in modes:
        success, max_count, notes = test_mode(spi, mode_name, mdr0, test_duration=10)
        results.append((mode_name, success, max_count, notes))

        if not success and "freeze" in notes.lower():
            print(f"\n⚠ Skipping remaining tests - mode froze the encoder")
            break

        print(f"\nPress ENTER to continue to next mode...")
        input()

    # Summary
    print("\n" + "="*60)
    print("TEST RESULTS SUMMARY")
    print("="*60)
    print(f"\n{'Mode':<25} {'Status':<10} {'Max Count':<12} {'Notes'}")
    print("-"*60)

    for mode_name, success, max_count, notes in results:
        status = "✓ WORKS" if success else "✗ FAILED"
        print(f"{mode_name:<25} {status:<10} {max_count:<12} {notes}")

    # Recommendation
    print("\n" + "="*60)
    print("RECOMMENDATION")
    print("="*60)

    working_modes = [m for m in results if m[1]]
    if working_modes:
        # Find mode with highest max_count (best resolution)
        best_mode = max(working_modes, key=lambda x: x[2])
        mode_to_mdr0 = dict(modes)

        print(f"\nBest working mode: {best_mode[0]}")
        print(f"  MDR0 value: 0x{mode_to_mdr0[best_mode[0]]:02X}")
        print(f"  Max counts observed: {best_mode[2]}")
        print(f"\nUpdate config/config.yaml:")

        # Determine encoder_mode value for config
        if "Non-Quadrature" in best_mode[0]:
            config_mode = 0
        elif "x1" in best_mode[0]:
            config_mode = 1
        elif "x2" in best_mode[0]:
            config_mode = 2
        elif "x4" in best_mode[0]:
            config_mode = 4

        print(f"  encoder_mode: {config_mode}")

        # Estimate counts per 360
        print(f"\nAfter calibration, expect approximately:")
        theoretical_counts = {
            0: 8640,   # 64 * 135
            1: 17280,  # 64 * 2 * 135
            2: 17280,  # 64 * 2 * 135 (x2 same as x1)
            4: 34560   # 64 * 4 * 135
        }
        print(f"  encoder_counts_per_360: ~{theoretical_counts.get(config_mode, 'unknown')}")
        print(f"  (Run calibrate_motor.py to get exact value)")
    else:
        print("\n⚠ No modes worked! Check hardware connections.")

    spi.close()
    print()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(0)
