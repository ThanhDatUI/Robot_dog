import pyb
from pyb import I2C

def test_mpu():
    # 1. Setup I2C (Mode 1 on PyBoard uses X9=SCL, X10=SDA typically)
    # Check your PyBoard pinout if unsure, but I2C(1) is standard.
    i2c = I2C(1, I2C.MASTER)
    mpu_addr = 0x68
    
    print("-" * 30)
    print("Scanning I2C bus...")
    devices = i2c.scan()
    found_addresses = [hex(x) for x in devices]
    print("Devices found: " + str(found_addresses))
    
    if mpu_addr not in devices:
        print("ERROR: MPU6050 not found at address " + hex(mpu_addr) + "!")
        print("Please check wiring:")
        print("  - VCC -> 3.3V (or 5V if module has regulator)")
        print("  - GND -> GND")
        print("  - SCL -> Pin X9")
        print("  - SDA -> Pin X10")
        return

    print("MPU6050 found at " + hex(mpu_addr))

    # 2. Wake up MPU6050
    # By default MPU6050 is in sleep mode. Write 0 to Power Management 1 register (0x6B)
    try:
        i2c.mem_write(0x00, mpu_addr, 0x6B)
        print("Wake up command sent.")
    except Exception as e:
        print("Failed to wake up MPU: " + str(e))
        return

    # 3. Read Data Loop
    print("Reading data (Press Ctrl+C to stop)...")
    print("{:>8} {:>8} {:>8} | {:>8} {:>8} {:>8}".format("Accel X", "Accel Y", "Accel Z", "Gyro X", "Gyro Y", "Gyro Z"))
    
    try:
        while True:
            # Read 14 bytes starting from register 0x3B (ACCEL_XOUT_H)
            # Structure: Ax_H, Ax_L, Ay_H, Ay_L, Az_H, Az_L, Temp_H, Temp_L, Gx_H, Gx_L, ...
            data = i2c.mem_read(14, mpu_addr, 0x3B)
            
            # Helper to convert 2 bytes to signed 16-bit integer
            def bytes_to_int(msb, lsb):
                val = (msb << 8) | lsb
                if val >= 0x8000:
                    val -= 0x10000
                return val

            ax = bytes_to_int(data[0], data[1])
            ay = bytes_to_int(data[2], data[3])
            az = bytes_to_int(data[4], data[5])
            # temp = bytes_to_int(data[6], data[7]) # Skipping temp
            gx = bytes_to_int(data[8], data[9])
            gy = bytes_to_int(data[10], data[11])
            gz = bytes_to_int(data[12], data[13])

            print("{:8d} {:8d} {:8d} | {:8d} {:8d} {:8d}".format(ax, ay, az, gx, gy, gz))
            
            pyb.delay(100) # 10Hz sample rate for display
            
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print("\nError reading data: " + str(e))

if __name__ == "__main__":
    test_mpu()
