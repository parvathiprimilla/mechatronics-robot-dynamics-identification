from machine import I2C, Pin

print("=== 3pi+ 2040 IMU Detection Test ===\n")

# Initialize I2C bus (standard pins for RP2040)
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

# Scan for I2C devices
devices = i2c.scan()

print(f"Found {len(devices)} I2C device(s):")
for device in devices:
    print(f"  Address: 0x{device:02X}")

if len(devices) > 0:
    IMU_ADDR = devices[0]
    try:
        # Read WHO_AM_I register (0x0F) to identify chip
        who_am_i = i2c.readfrom_mem(IMU_ADDR, 0x0F, 1)[0]
        print(f"\nWHO_AM_I register: 0x{who_am_i:02X}")
        
        if who_am_i == 0x6C:
            print("LSM6DSO IMU confirmed!")
        else:
            print(f"⚠️  Unknown IMU chip ID: 0x{who_am_i:02X}")
            
    except Exception as e:
        print(f"❌ Error reading IMU: {e}")
else:
    print("\n⚠️ No IMU detected on I2C bus")
    print("This may require Pololu library initialization")

print("\nTest complete!")
