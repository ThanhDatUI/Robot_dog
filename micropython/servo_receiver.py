"""
PyBoard Servo Controller - Receiver Side
Receives servo commands via USB serial and controls servos via PCA9685

Protocol: "CH<channel>:<angle>\n"
Example: "CH1:115\n" sets channel 1 to 115 degrees
"""

import utime
from pyb import USB_VCP, I2C
from pca9685 import PCA9685

# Initialize PCA9685
print("Initializing PCA9685...")
i2c = I2C(1, I2C.MASTER)
pca = PCA9685(i2c, 0x40)
pca.freq(50)
print("PCA9685 ready at 50 Hz")

# USB serial for PC communication
usb = USB_VCP()

# Valid servo channels
VALID_CHANNELS = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]

def angle_to_duty(angle_deg):
    """Convert angle to duty cycle"""
    us_min = 500
    us_max = 2500
    period_us = 20000
    pulse_us = us_min + (angle_deg / 180.0) * (us_max - us_min)
    duty = int((pulse_us / period_us) * 4095)
    return duty

def set_servo(channel, angle):
    """Set servo to angle"""
    if channel not in VALID_CHANNELS:
        return False
    if angle < 0 or angle > 180:
        return False
    
    duty = angle_to_duty(angle)
    pca.duty(channel, duty)
    return True

# Main loop
print("Waiting for commands from PC...")
print("Format: CH<channel>:<angle>")

buffer = ""
while True:
    if usb.any():
        # Read incoming data
        data = usb.read(usb.any()).decode('utf-8', 'ignore')
        buffer += data
        
        # Process complete lines
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            line = line.strip()
            
            if not line:
                continue
            
            # Parse command: CH<channel>:<angle>
            if line.startswith('CH') and ':' in line:
                try:
                    parts = line[2:].split(':')
                    channel = int(parts[0])
                    angle = int(parts[1])
                    
                    if set_servo(channel, angle):
                        response = "OK CH{}: {}deg\n".format(channel, angle)
                        usb.write(response)
                        print(response.strip())  # Also print to console
                    else:
                        usb.write("ERROR Invalid channel/angle\n")
                except Exception as e:
                    usb.write("ERROR Parse failed: {}\n".format(str(e)))
            else:
                usb.write("ERROR Invalid format\n")
    
    utime.sleep_ms(10)
