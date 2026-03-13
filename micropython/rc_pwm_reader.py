import pyb
from pyb import Pin, ExtInt
import utime

# Global variables to store PWM pulse width (in microseconds)
receiver_value = [1500, 1500]  # [Channel1, Channel3]

# State tracking for edge detection
last_time_ch1 = 0
last_time_ch3 = 0
last_state_ch1 = 0
last_state_ch3 = 0

def ch1_callback(line):
    global last_time_ch1, last_state_ch1, receiver_value
    current_time = utime.ticks_us()
    pin_state = ch1_pin.value()
    
    if pin_state == 1 and last_state_ch1 == 0:  # Rising edge
        last_time_ch1 = current_time
        last_state_ch1 = 1
    elif pin_state == 0 and last_state_ch1 == 1:  # Falling edge
        pulse_width = utime.ticks_diff(current_time, last_time_ch1)
        if 800 < pulse_width < 2200:  # Valid RC PWM range
            receiver_value[0] = pulse_width
        last_state_ch1 = 0

def ch3_callback(line):
    global last_time_ch3, last_state_ch3, receiver_value
    current_time = utime.ticks_us()
    pin_state = ch3_pin.value()
    
    if pin_state == 1 and last_state_ch3 == 0:  # Rising edge
        last_time_ch3 = current_time
        last_state_ch3 = 1
    elif pin_state == 0 and last_state_ch3 == 1:  # Falling edge
        pulse_width = utime.ticks_diff(current_time, last_time_ch3)
        if 800 < pulse_width < 2200:  # Valid RC PWM range
            receiver_value[1] = pulse_width  # Fixed: was [2], should be [1]
        last_state_ch3 = 0

def setup():
    global ch1_pin, ch3_pin, ch1_int, ch3_int
    
    # Setup pins with pull-up (same as Arduino INPUT_PULLUP)
    ch1_pin = Pin('X1', Pin.IN, Pin.PULL_UP)
    ch3_pin = Pin('X2', Pin.IN, Pin.PULL_UP)
    
    # Attach interrupts on both edges (RISING and FALLING)
    ch1_int = ExtInt(ch1_pin, ExtInt.IRQ_RISING_FALLING, Pin.PULL_UP, ch1_callback)
    ch3_int = ExtInt(ch3_pin, ExtInt.IRQ_RISING_FALLING, Pin.PULL_UP, ch3_callback)
    
    print("RC PWM Reader initialized")
    print("Channel 1: X1")
    print("Channel 3: X2")

def get_channel(channel_num):
    """Get PWM value for a specific channel (1 or 3)"""
    if channel_num == 1:
        return receiver_value[0]
    elif channel_num == 3:
        return receiver_value[1]
    else:
        return 1500

def test_loop():
    """Test function to print values continuously"""
    print("Reading RC values (Ctrl+C to stop)...")
    try:
        while True:
            ch1 = receiver_value[0]
            ch3 = receiver_value[1]
            print("CH1: {:4d} | CH3: {:4d}".format(ch1, ch3))
            pyb.delay(100)  # Print every 100ms
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    setup()
    test_loop()
