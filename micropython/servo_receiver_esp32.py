"""
ESP32-S3 Servo Receiver - danh cho Calibration GUI
Protocol: "CH<channel>:<angle>\n"
Example:  "CH1:90\n" -> set channel 1 to 90 degrees
"""

import utime
import sys
import select
from machine import I2C, Pin
from pca9685 import PCA9685

# Init PCA9685
print("Initializing PCA9685...")
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
pca = PCA9685(i2c, 0x40)
pca.freq(50)
print("PCA9685 ready")

VALID_CHANNELS = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]

def angle_to_duty(angle):
    pulse = 500 + (angle / 180.0) * 2000
    return int((pulse / 20000) * 4095)

def set_servo(ch, angle):
    if ch not in VALID_CHANNELS or not (0 <= angle <= 180):
        return False
    pca.duty(ch, angle_to_duty(angle))
    return True

def usb_write(msg):
    sys.stdout.write(msg)

def usb_any():
    r, _, _ = select.select([sys.stdin], [], [], 0)
    return bool(r)

def usb_read_char():
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        return sys.stdin.read(1)
    return ''

print("Waiting for commands... Format: CH<ch>:<angle>")
usb_write("READY\n")

buffer = ""
while True:
    if usb_any():
        ch = usb_read_char()
        if ch:
            buffer += ch

    while '\n' in buffer:
        line, buffer = buffer.split('\n', 1)
        line = line.strip()
        if not line:
            continue

        if line.startswith('CH') and ':' in line:
            try:
                parts = line[2:].split(':')
                channel = int(parts[0])
                angle = int(parts[1])
                if set_servo(channel, angle):
                    usb_write("OK CH{}: {}deg\n".format(channel, angle))
                    print("CH{} -> {}deg".format(channel, angle))
                else:
                    usb_write("ERROR Invalid channel/angle\n")
            except Exception as e:
                usb_write("ERROR {}\n".format(str(e)))
        else:
            usb_write("ERROR Invalid format\n")

    utime.sleep_ms(5)
