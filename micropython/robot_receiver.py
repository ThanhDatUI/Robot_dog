"""
SpotMicro Robot Controller - PyBoard Receiver
Receives control commands from PC and controls robot via state machine

Commands:
  STAND - Stand up
  SIT - Sit down
  PITCH:<angle> - Set pitch angle
  ROLL:<angle> - Set roll angle
  THROTTLE:<value> - Set height
"""

import utime
from pyb import USB_VCP, I2C, Pin
from pca9685 import PCA9685
from profiler import Profiler
from state import StateMachine

# Initialize hardware
print("Initializing hardware...")
i2c = I2C(1, I2C.MASTER)

# PCA9685
pca = PCA9685(i2c, 0x40)
pca.freq(50)
print("PCA9685 ready")


# Servos (using calibrated angles from main.py)
class Servo:
    us_min_cmd = 500.0
    us_max_cmd = 2500.0
    deg_min_cmd = 0.0
    deg_max_cmd = 180.0
    us_per_180_degs = us_max_cmd - us_min_cmd
    us_per_deg = us_per_180_degs / 180.0
    freq_hz = 50.0
    period_us = (1.0 / freq_hz) * 1e6
    pca9685 = pca

    def __init__(self, pwm_channel, min_angle_deg, center_angle_deg, max_angle_deg, is_inverted):
        self.pwm_channel = pwm_channel
        self.min_angle_deg = min_angle_deg
        self.center_angle_deg = center_angle_deg
        self.max_angle_deg = max_angle_deg
        self.is_inverted = is_inverted

    @staticmethod
    def get_12_bit_duty_cycle_for_us(position_us):
        if position_us < Servo.us_min_cmd or position_us > Servo.us_max_cmd:
            raise ValueError
        duty_cycle_12_bit = int((float(position_us) / float(Servo.period_us)) * (2 ** 12))
        return duty_cycle_12_bit

    @staticmethod
    def get_12_bit_duty_cycle_for_angle(angle_deg):
        if angle_deg < Servo.deg_min_cmd or angle_deg > Servo.deg_max_cmd:
            raise ValueError
        target_us = Servo.us_min_cmd + (float(angle_deg) * Servo.us_per_deg)
        return Servo.get_12_bit_duty_cycle_for_us(target_us)

    def command_deg(self, angle_deg):
        angle_deg_with_inversion_relative_to_middle = angle_deg
        if self.is_inverted:
            angle_deg_with_inversion_relative_to_middle = -angle_deg
        angle_deg_raw = self.center_angle_deg + angle_deg_with_inversion_relative_to_middle
        if (angle_deg_raw < self.min_angle_deg) or (angle_deg_raw > self.max_angle_deg):
            raise ValueError
        self.pca9685.duty(self.pwm_channel, Servo.get_12_bit_duty_cycle_for_angle(angle_deg_raw))

# Initialize servos with calibrated angles
front_right_shoulder = Servo(0, 0, 100, 180, False)
front_right_leg = Servo(1, 0, 100, 180, False)
front_right_foot = Servo(2, 0, 30, 180, False)

front_left_shoulder = Servo(4, 0, 80, 180, True)
front_left_leg = Servo(5, 0, 80, 180, True)
front_left_foot = Servo(6, 0, 145, 180, True)

rear_right_shoulder = Servo(8, 0, 90, 180, False)
rear_right_leg = Servo(9, 0, 90, 180, False)
rear_right_foot = Servo(10, 0, 30, 180, False)

rear_left_shoulder = Servo(12, 0, 80, 180, True)
rear_left_leg = Servo(13, 0, 85, 180, True)
rear_left_foot = Servo(14, 0, 145, 180, True)

# Profiler and state machine
profiler = Profiler()
state_machine = StateMachine()

# USB serial
usb = USB_VCP()

# Servo command shortcuts
servo_commands = [
    front_right_shoulder.command_deg, front_left_shoulder.command_deg,
    front_right_leg.command_deg, front_left_leg.command_deg,
    front_right_foot.command_deg, front_left_foot.command_deg,
    rear_right_shoulder.command_deg, rear_left_shoulder.command_deg,
    rear_right_leg.command_deg, rear_left_leg.command_deg,
    rear_right_foot.command_deg, rear_left_foot.command_deg,
]

print("Ready for PC control commands")
usb.write("READY\n")

# Crawl state: 0=stop, 1=left, 2=right
crawl_mode = 0
crawl_rc_channels = None

# Main control loop
buffer = ""
last_loop_us = utime.ticks_us()
loop_period_us = 19650  # ~50Hz

while True:
    # Handle incoming commands
    if usb.any():
        data = usb.read(usb.any()).decode('utf-8', 'ignore')
        buffer += data
        
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            cmd = line.strip()
            
            if not cmd:
                continue
            
            try:
                if cmd == "STAND":
                    # STAND: rc[6]=1580 (arm), rc[7]=1000 (stand mode)
                    # rc[0]=rc[1]=980 (no tilt)
                    rc_channels = [980, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK STAND\n")
                    print("Standing up...")
                    
                elif cmd == "SIT":
                    # SIT: rc[6]=1580 (arm), rc[7]=180 (<500 = sit)
                    rc_channels = [980, 980, 980, 980, 980, 980, 1580, 180] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK SIT\n")
                    print("Sitting down...")
                
                elif cmd == "LEAN_LEFT":
                    # Use rc[1] for left/right lean
                    rc_channels = [980, 1380, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK LEAN_LEFT\n")
                    print("Leaning left...")
                
                elif cmd == "LEAN_CENTER":
                    # Center position - all at 980
                    rc_channels = [980, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK LEAN_CENTER\n")
                    print("Centering...")
                
                elif cmd == "LEAN_RIGHT":
                    # Use rc[1] for left/right lean
                    rc_channels = [980, 580, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK LEAN_RIGHT\n")
                    print("Leaning right...")
                
                elif cmd == "WALK_FORWARD":
                    # rc[0] > 980 = tilt body forward
                    rc_channels = [1380, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK WALK_FORWARD\n")
                    print("Moving forward...")
                
                elif cmd == "WALK_CENTER":
                    # Center position
                    rc_channels = [980, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK WALK_CENTER\n")
                    print("Centering...")
                
                elif cmd == "WALK_BACKWARD":
                    # rc[0] < 980 = tilt body backward
                    rc_channels = [580, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK WALK_BACKWARD\n")
                    print("Moving backward...")
                
                # CRAWL mode - actual walking (rc[7] > 1500)
                elif cmd == "CRAWL_LEFT":
                    # Set crawl mode to left - will continue in main loop
                    crawl_mode = 1
                    crawl_rc_channels = [1380, 980, 980, 980, 980, 980, 1580, 1580] + [980] * 8
                    usb.write("OK CRAWL_LEFT\n")
                    print("Crawling left...")
                
                elif cmd == "CRAWL_STOP":
                    # Stop crawling
                    crawl_mode = 0
                    crawl_rc_channels = None
                    rc_channels = [980, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK CRAWL_STOP\n")
                    print("Stopping crawl...")
                
                elif cmd == "CRAWL_RIGHT":
                    # Set crawl mode to right - will continue in main loop
                    crawl_mode = 2
                    crawl_rc_channels = [580, 980, 980, 980, 980, 980, 1580, 1580] + [980] * 8
                    usb.write("OK CRAWL_RIGHT\n")
                    print("Crawling right...")
                    
                else:
                    usb.write("ERROR Unknown command\n")
                    
            except Exception as e:
                usb.write("ERROR {}\n".format(str(e)))
    
    # Continue crawling if in crawl mode
    if crawl_mode != 0 and crawl_rc_channels is not None:
        state_machine.update(profiler, crawl_rc_channels, last_loop_us)
    
    # Update profiler and servo positions
    profiler.tick()
    pos_cmds = profiler.get_position_commands()
    
    # Command servos
    for i, cmd in enumerate(pos_cmds):
        if cmd is not None:
            try:
                servo_commands[i](cmd)
            except:
                pass
    
    # Wait for next loop
    while utime.ticks_diff(utime.ticks_us(), last_loop_us) < loop_period_us:
        pass
    last_loop_us = utime.ticks_us()
