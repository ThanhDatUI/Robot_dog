"""
Test script for PCA9685 servo driver on PyBoard
Wiring:
  PyBoard X9 (SCL) -> PCA9685 SCL
  PyBoard X10 (SDA) -> PCA9685 SDA
  PyBoard GND -> PCA9685 GND
  PyBoard 3V3 -> PCA9685 VCC
  
  External 5-6V -> PCA9685 V+
  External GND -> PCA9685 GND
  
  Servo -> PCA9685 Channel 0
"""

import utime
from pyb import I2C, Pin
from pca9685 import PCA9685


def main():
    print("=== PCA9685 Servo Driver Test ===")
    
    # Initialize I2C
    print("Initializing I2C on bus 1 (X9=SCL, X10=SDA)...")
    i2c = I2C(1, I2C.MASTER)
    
    # Scan for devices
    print("Scanning I2C bus...")
    devices = i2c.scan()
    print("Found devices at addresses:", [hex(addr) for addr in devices])
    
    # Check if PCA9685 is present
    PWM_ADDR = 0x40
    if PWM_ADDR not in devices:
        print("ERROR: PCA9685 not found at address 0x40!")
        print("Check your wiring and power supply.")
        return
    
    print("PCA9685 found at address 0x40")
    
    # Initialize PCA9685
    print("\nInitializing PCA9685...")
    pca = PCA9685(i2c, PWM_ADDR)
    
    # Set frequency to 50Hz (standard for servos)
    print("Setting PWM frequency to 50 Hz...")
    pca.freq(50)
    actual_freq = pca.freq()
    print("Actual frequency: {} Hz".format(actual_freq))
    
    # Helper function to convert angle to 12-bit duty cycle
    def angle_to_duty(angle_deg):
        # Standard servo: 500us = 0 deg, 2500us = 180 deg
        # Period = 1/50Hz = 20ms = 20000us
        # Duty cycle range: 0-4095 (12-bit)
        us_min = 500
        us_max = 2500
        period_us = 20000
        
        pulse_us = us_min + (angle_deg / 180.0) * (us_max - us_min)
        duty = int((pulse_us / period_us) * 4095)
        return duty
    
    # Test servo on channel 0
    print("\n=== Testing Servo on Channel 0 ===")
    channel = 0
    
    # Sweep test
    angles = [0, 45, 90, 135, 180, 135, 90, 45, 0]
    
    for angle in angles:
        duty = angle_to_duty(angle)
        print("Moving to {} degrees (duty={})...".format(angle, duty))
        pca.duty(channel, duty)
        utime.sleep_ms(500)  # Wait for servo to move
    
    print("\n=== Test Complete ===")
    print("If servo moved smoothly through all angles, PCA9685 is working correctly!")
    
    # Center the servo
    print("\nCentering servo at 90 degrees...")
    pca.duty(channel, angle_to_duty(90))


if __name__ == "__main__":
    main()
