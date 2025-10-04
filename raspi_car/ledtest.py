import smbus
import time

# Use I2C bus 1 on Raspberry Pi
bus = smbus.SMBus(1)

print("Scanning I2C bus for devices...\n")

devices_found = []

for address in range(0x03, 0x78):
    try:
        bus.write_quick(address)
        print(f"Found device at 0x{address:02X}")
        devices_found.append(address)
    except OSError:
        pass  # No device at this address

if not devices_found:
    print("No I2C devices found.")
else:
    print("\nScan complete.")
    print("Devices found at:")
    for addr in devices_found:
        print(f" - 0x{addr:02X}")
