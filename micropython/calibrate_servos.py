"""
Interactive Servo Calibration Tool for SpotMicro
-------------------------------------------------
This tool helps you find the correct center angles for all 12 servos.

GOAL: Adjust each servo so the leg segment is PERFECTLY VERTICAL (straight down)

Servo Mapping:
  Front Right: Shoulder=0, Leg=1, Foot=2
  Front Left:  Shoulder=4, Leg=5, Foot=6
  Rear Right:  Shoulder=8, Leg=9, Foot=10
  Rear Left:   Shoulder=12, Leg=13, Foot=14

Usage:
  1. Run this script: mpremote run calibrate_servos.py
  2. In REPL, use commands:
     - set_servo(channel, angle)  # Set servo to specific angle
     - adjust(channel, delta)     # Adjust by ±degrees
     - show_all()                 # Display all current angles
     - export_config()            # Print code for main.py
  
  3. For each servo:
     - Start with default angle (see below)
     - Use adjust(channel, ±5) for coarse adjustment
     - Use adjust(channel, ±1) for fine tuning
     - Visually check leg is vertical
     - Record angle when perfect
"""

import utime
from pyb import I2C, Pin
from pca9685 import PCA9685

# Default starting angles (from original main.py)
DEFAULT_ANGLES = {
    0: 82,   # Front Right Shoulder
    1: 111,  # Front Right Leg
    2: 30,   # Front Right Foot
    4: 97,   # Front Left Shoulder
    5: 95,   # Front Left Leg
    6: 142,  # Front Left Foot
    8: 73,   # Rear Right Shoulder
    9: 84,   # Rear Right Leg
    10: 34,  # Rear Right Foot
    12: 94,  # Rear Left Shoulder
    13: 90,  # Rear Left Leg
    14: 140, # Rear Left Foot
}

# Current angles (will be modified during calibration)
current_angles = DEFAULT_ANGLES.copy()

# Global PCA9685 instance
pca = None


def init_pca():
    """Initialize PCA9685"""
    global pca
    print("Initializing PCA9685...")
    i2c = I2C(1, I2C.MASTER)
    pca = PCA9685(i2c, 0x40)
    pca.freq(50)
    print("PCA9685 ready at 50 Hz")


def angle_to_duty(angle_deg):
    """Convert servo angle (0-180) to 12-bit duty cycle"""
    us_min = 500
    us_max = 2500
    period_us = 20000
    
    pulse_us = us_min + (angle_deg / 180.0) * (us_max - us_min)
    duty = int((pulse_us / period_us) * 4095)
    return duty


def set_servo(channel, angle):
    """Set servo to specific angle and update tracking"""
    if channel not in DEFAULT_ANGLES:
        print("ERROR: Invalid channel {}. Valid: {}".format(channel, list(DEFAULT_ANGLES.keys())))
        return
    
    if angle < 0 or angle > 180:
        print("ERROR: Angle must be 0-180 degrees")
        return
    
    duty = angle_to_duty(angle)
    pca.duty(channel, duty)
    current_angles[channel] = angle
    print("Channel {}: {}° (duty={})".format(channel, angle, duty))


def adjust(channel, delta):
    """Adjust servo by delta degrees"""
    if channel not in current_angles:
        print("ERROR: Invalid channel {}".format(channel))
        return
    
    new_angle = current_angles[channel] + delta
    set_servo(channel, new_angle)


def show_all():
    """Display all current servo angles"""
    print("\n=== Current Servo Angles ===")
    print("Front Right:")
    print("  Shoulder (0): {}°".format(current_angles[0]))
    print("  Leg      (1): {}°".format(current_angles[1]))
    print("  Foot     (2): {}°".format(current_angles[2]))
    print("\nFront Left:")
    print("  Shoulder (4): {}°".format(current_angles[4]))
    print("  Leg      (5): {}°".format(current_angles[5]))
    print("  Foot     (6): {}°".format(current_angles[6]))
    print("\nRear Right:")
    print("  Shoulder (8): {}°".format(current_angles[8]))
    print("  Leg      (9): {}°".format(current_angles[9]))
    print("  Foot    (10): {}°".format(current_angles[10]))
    print("\nRear Left:")
    print("  Shoulder (12): {}°".format(current_angles[12]))
    print("  Leg      (13): {}°".format(current_angles[13]))
    print("  Foot     (14): {}°".format(current_angles[14]))
    print("=" * 30 + "\n")


def reset_all():
    """Reset all servos to default angles"""
    print("Resetting all servos to defaults...")
    for channel, angle in DEFAULT_ANGLES.items():
        set_servo(channel, angle)
    print("Reset complete!")


def center_all():
    """Move all servos to 90° (mechanical center)"""
    print("Moving all servos to 90° (mechanical center)...")
    for channel in DEFAULT_ANGLES.keys():
        set_servo(channel, 90)
    print("All servos at 90°")


def export_config():
    """Export calibrated angles as Python code"""
    print("\n" + "=" * 50)
    print("Copy and paste this into main.py (lines 161-175):")
    print("=" * 50)
    print()
    print("    front_right_shoulder = Servo(0, 0, {}, 180, False)".format(current_angles[0]))
    print("    front_right_leg = Servo(1, 0, {}, 180, False)".format(current_angles[1]))
    print("    front_right_foot = Servo(2, 0, {}, 180, False)".format(current_angles[2]))
    print()
    print("    front_left_shoulder = Servo(4, 0, {}, 180, True)".format(current_angles[4]))
    print("    front_left_leg = Servo(5, 0, {}, 180, True)".format(current_angles[5]))
    print("    front_left_foot = Servo(6, 0, {}, 180, True)".format(current_angles[6]))
    print()
    print("    rear_right_shoulder = Servo(8, 0, {}, 180, False)".format(current_angles[8]))
    print("    rear_right_leg = Servo(9, 0, {}, 180, False)".format(current_angles[9]))
    print("    rear_right_foot = Servo(10, 0, {}, 180, False)".format(current_angles[10]))
    print()
    print("    rear_left_shoulder = Servo(12, 0, {}, 180, True)".format(current_angles[12]))
    print("    rear_left_leg = Servo(13, 0, {}, 180, True)".format(current_angles[13]))
    print("    rear_left_foot = Servo(14, 0, {}, 180, True)".format(current_angles[14]))
    print()
    print("=" * 50 + "\n")


def main():
    print("\n" + "=" * 50)
    print("SpotMicro Servo Calibration Tool")
    print("=" * 50)
    
    init_pca()
    
    print("\nAvailable commands:")
    print("  set_servo(channel, angle)  - Set servo to angle")
    print("  adjust(channel, delta)     - Adjust servo by ± degrees")
    print("  show_all()                 - Show all current angles")
    print("  reset_all()                - Reset to defaults")
    print("  center_all()               - Move all to 90°")
    print("  export_config()            - Print code for main.py")
    print("\nStarting with default angles...")
    reset_all()
    
    print("\n✓ Ready! Use commands above in REPL.")
    print("Example: adjust(1, 5)  # Increase channel 1 by 5°\n")


if __name__ == "__main__":
    main()
