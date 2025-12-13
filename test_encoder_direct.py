#!/usr/bin/env python3
"""
Direct LS7366R encoder test - minimal dependencies
Tests SPI communication and encoder counting
"""

import time
import sys

try:
    import spidev
except ImportError:
    print("ERROR: spidev not installed. Run: sudo pip3 install spidev")
    sys.exit(1)

# LS7366R Configuration
SPI_BUS = 0
SPI_DEVICE = 0

# Commands
CMD_CLEAR_CNTR = 0x20
CMD_CLEAR_STR = 0x30
CMD_READ = 0x40
CMD_WRITE = 0x80

# Registers
REG_MDR0 = 0x08
REG_MDR1 = 0x10
REG_CNTR = 0x20
REG_STR = 0x30

def test_encoder():
    """Test LS7366R encoder communication and counting"""

    print("="*60)
    print("LS7366R Encoder Direct Test")
    print("="*60)

    # Initialize SPI
    try:
        spi = spidev.SpiDev()
        spi.open(SPI_BUS, SPI_DEVICE)
        spi.max_speed_hz = 50000  # 50 kHz (matches working configuration)
        spi.mode = 0
        print(f"✓ SPI {SPI_BUS}.{SPI_DEVICE} opened successfully at 50 kHz")
    except Exception as e:
        print(f"✗ Failed to open SPI: {e}")
        print("\nTroubleshooting:")
        print("  1. Check if SPI is enabled: ls /dev/spidev0.0")
        print("  2. Enable SPI via: sudo raspi-config")
        print("  3. Verify wiring to CS0 (GPIO 8)")
        return

    try:
        # Configure LS7366R - matches working troubleshooting code
        print("\nConfiguring LS7366R...")

        # Clear counter first
        spi.xfer2([CMD_CLEAR_CNTR])
        time.sleep(0.1)  # Critical delay

        # Set to non-quadrature mode (simplest, most reliable)
        mdr0 = 0b00000000  # Non-quadrature mode
        spi.xfer2([CMD_WRITE | REG_MDR0, mdr0])
        print(f"  MDR0 = 0x{mdr0:02X} (non-quadrature mode)")
        time.sleep(0.1)  # Critical delay

        # Read initial count - single transaction method
        data = spi.xfer2([CMD_READ | REG_CNTR, 0, 0, 0, 0])
        initial_count = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]
        if initial_count & 0x80000000:
            initial_count -= 0x100000000

        print(f"\n✓ Initial count: {initial_count}")
        print(f"  (Raw bytes: {data[1]:02X} {data[2]:02X} {data[3]:02X} {data[4]:02X})")

        # Read status register
        spi.xfer2([CMD_READ | REG_STR])
        status = spi.readbytes(1)[0]
        print(f"  Status register: 0x{status:02X}")

        # Continuous monitoring
        print("\n" + "="*60)
        print("Monitoring encoder counts (manually rotate motor shaft)")
        print("Press Ctrl+C to stop")
        print("="*60)
        print(f"\n{'Time (s)':>10} | {'Count':>10} | {'Change':>10} | {'Rate (c/s)':>12}")
        print("-" * 50)

        last_count = initial_count
        last_time = time.time()
        start_time = last_time

        while True:
            # Read counter - single transaction method
            data = spi.xfer2([CMD_READ | REG_CNTR, 0, 0, 0, 0])
            count = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]
            if count & 0x80000000:
                count -= 0x100000000

            current_time = time.time()
            elapsed = current_time - start_time
            dt = current_time - last_time

            change = count - last_count
            rate = change / dt if dt > 0 else 0

            # Only print if there's a change or every 2 seconds
            if change != 0 or int(elapsed) % 2 == 0:
                print(f"{elapsed:>10.1f} | {count:>10} | {change:>+10} | {rate:>+12.1f}")

            last_count = count
            last_time = current_time

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user")
    except Exception as e:
        print(f"\n✗ Error during testing: {e}")
        import traceback
        traceback.print_exc()
    finally:
        spi.close()
        print("\nSPI closed")

    # Troubleshooting guide
    print("\n" + "="*60)
    print("TROUBLESHOOTING GUIDE")
    print("="*60)
    print("\nIf you see NO COUNTS when rotating:")
    print("  1. Check encoder A/B channel wiring to LS7366R")
    print("  2. Verify encoder has power (3.3V or 5V depending on encoder)")
    print("  3. Test encoder directly with multimeter/oscilloscope")
    print("  4. Check LS7366R power supply (5V)")
    print("  5. Verify common ground between Pi, encoder, and LS7366R")
    print("  6. Try swapping A/B channels (will reverse count direction)")
    print("\nIf you see RANDOM/NOISY counts:")
    print("  1. Add pull-up resistors on A/B lines (4.7kΩ to VCC)")
    print("  2. Add 100nF bypass capacitor near LS7366R VDD pin")
    print("  3. Keep encoder wires short and twisted together")
    print("  4. Check for proper grounding")
    print("\nIf counts go WRONG DIRECTION:")
    print("  1. Swap encoder A and B channels (hardware fix)")
    print("  2. Direction is handled by negation in encoder.py (software)")
    print()

if __name__ == "__main__":
    test_encoder()
