import utime
from pyb import I2C, Pin, ExtInt
from pca9685 import PCA9685
from profiler import Profiler
from state import StateMachine

# ============== RC PWM READER ==============
receiver_value = [1500, 1500, 1500, 1500, 1500]  # [CH1, CH2, CH3, CH5, CH6]
last_time = [0, 0, 0, 0, 0]
last_state = [0, 0, 0, 0, 0]
pins = []

def make_callback(index, pin):
    def callback(line):
        global last_time, last_state, receiver_value
        current_time = utime.ticks_us()
        pin_state = pin.value()
        
        if pin_state == 1 and last_state[index] == 0:
            last_time[index] = current_time
            last_state[index] = 1
        elif pin_state == 0 and last_state[index] == 1:
            pulse_width = utime.ticks_diff(current_time, last_time[index])
            if 800 < pulse_width < 2200:
                receiver_value[index] = pulse_width
            last_state[index] = 0
    return callback

def setup_rc():
    global pins
    pin_names = ['X1', 'X2', 'X3', 'X4', 'X5']
    for i, pin_name in enumerate(pin_names):
        pin = Pin(pin_name, Pin.IN, Pin.PULL_UP)
        pins.append(pin)
        callback = make_callback(i, pin)
        ExtInt(pin, ExtInt.IRQ_RISING_FALLING, Pin.PULL_UP, callback)
    print("RC PWM Reader ready")

print("Initializing hardware...")
i2c = I2C(1, I2C.MASTER)

pca = PCA9685(i2c, 0x40)
pca.freq(50)
print("PCA9685 ready")

# Servo class
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
front_right_foot = Servo(2, 0, 40, 180, False)

front_left_shoulder = Servo(4, 0, 80, 180, True)
front_left_leg = Servo(5, 0, 70, 180, True)
front_left_foot = Servo(6, 0, 150, 180, True)

rear_right_shoulder = Servo(8, 0, 90, 180, False)
rear_right_leg = Servo(9, 0, 100, 180, False)
rear_right_foot = Servo(10, 0, 30, 180, False)

rear_left_shoulder = Servo(12, 0, 80, 180, True)
rear_left_leg = Servo(13, 0, 80, 180, True)
rear_left_foot = Servo(14, 0, 150, 180, True)

# Profiler and state machine
profiler = Profiler()
state_machine = StateMachine()

# Servo command shortcuts
servo_commands = [
    front_right_shoulder.command_deg, front_left_shoulder.command_deg,
    front_right_leg.command_deg, front_left_leg.command_deg,
    front_right_foot.command_deg, front_left_foot.command_deg,
    rear_right_shoulder.command_deg, rear_left_shoulder.command_deg,
    rear_right_leg.command_deg, rear_left_leg.command_deg,
    rear_right_foot.command_deg, rear_left_foot.command_deg,
]

# Setup RC
setup_rc()

print("=" * 40)
print("RC Robot Controller Ready!")
print("Use RC remote to control robot")
print("=" * 40)

# ============== MAIN CONTROL LOOP ==============
last_loop_us = utime.ticks_us()
loop_period_us = 19650  # ~50Hz
last_print_time = 0

while True:
    # Read RC channels (SWAPPED: CH1=Pitch, CH2=Lean)
    ch1 = receiver_value[0]  # Pitch Fwd/Back: 967-2016, center 1515 (fwd=low, back=high)
    ch2 = receiver_value[1]  # Lean Left/Right: 944-1979, center 1531 (left=low, right=high)
    ch3 = receiver_value[2]  # Walk L/R: 943-1885
    ch5 = receiver_value[3]  # Stand/Sit: stand 1000, sit 2000
    ch6 = receiver_value[4]  # Mode: walk 854, reset 1906
    
    # Convert RC values to state machine format
    # State machine expects: rc[0]=roll, rc[1]=pitch, rc[2]=throttle, rc[6]=arm, rc[7]=mode
    
    # Determine arm state (CH5 < 1200 = armed/stand)
    if ch5 < 1200:
        arm_value = 1580  # Armed
        mode_value = 1000  # Stand mode (UP state)
    elif ch5 > 1800:
        arm_value = 1580  # Armed
        mode_value = 180   # Sit mode (DOWN state)
    else:
        arm_value = 180   # Disarmed
        mode_value = 980
    
    # Determine walk mode (CH6 < 1200 = walk enabled)
    if ch6 < 1200:
        # Walk mode - CH3 controls walking
        mode_value = 1580  # CRAWL mode (rc[7] > 1500)
        # Map CH3 to roll_value for crawl direction
        # CH3 ~1885 = walk left (high), CH3 ~943 = walk right (low)
        # y_command = ((rc[0] - 980) / 800) * -40
        if ch3 > 1600:
            roll_value = 1500  # Walk left
        elif ch3 < 1200:
            roll_value = 500   # Walk right
        else:
            roll_value = 980   # Stop
        pitch_value = 980   # No pitch in walk mode
    else:
        # Normal mode - CH2 controls lean (roll), CH1 controls pitch
        # No crawl mode
        if ch5 < 1200:
            mode_value = 1000  # Stand UP mode
        
        # Map CH2 (944-1979) to roll: left=1380, center=980, right=580
        if ch2 < 1200:
            roll_value = 1380  # Lean left
        elif ch2 > 1800:
            roll_value = 580   # Lean right
        else:
            roll_value = 980   # Center
        
        # Map CH1 (967-2016) to pitch: forward=1380 (low), center=980, backward=580 (high)
        if ch1 < 1200:
            pitch_value = 1380  # Pitch forward (stick forward = low value)
        elif ch1 > 1800:
            pitch_value = 580   # Pitch backward (stick back = high value)
        else:
            pitch_value = 980   # Center
    
    # Create RC channels array for state machine
    rc_channels = [roll_value, pitch_value, 980, 980, 980, 980, arm_value, mode_value] + [980] * 8
    
    # Update state machine
    state_machine.update(profiler, rc_channels, last_loop_us)
    
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
    
    # Debug print every 500ms
    current_time = utime.ticks_ms()
    if utime.ticks_diff(current_time, last_print_time) > 500:
        print("CH1:{:4d} CH2:{:4d} CH3:{:4d} CH5:{:4d} CH6:{:4d} | arm:{} mode:{}".format(
            ch1, ch2, ch3, ch5, ch6, arm_value, mode_value))
        last_print_time = current_time
    
    # Wait for next loop
    while utime.ticks_diff(utime.ticks_us(), last_loop_us) < loop_period_us:
        pass
    last_loop_us = utime.ticks_us()
 