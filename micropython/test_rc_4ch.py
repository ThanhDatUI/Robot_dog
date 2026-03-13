"""
Test RC PWM Reader - 5 Channels
Reads PWM signals from RC receiver on 5 channels

Wiring:
  CH1 (Left/Right) -> X1
  CH2 (unknown)    -> X2
  CH3 (Fwd/Back)   -> X3
  CH5 (unknown)    -> X4
  CH6 (unknown)    -> X5
"""

import pyb
from pyb import Pin, ExtInt
import utime

# Global variables to store PWM pulse width (in microseconds)
receiver_value = [1500, 1500, 1500, 1500, 1500]  # [CH1, CH2, CH3, CH5, CH6]

# State tracking for edge detection
last_time = [0, 0, 0, 0, 0]
last_state = [0, 0, 0, 0, 0]

# Pin objects
pins = []

def make_callback(index, pin):
    """Create callback function for each channel"""
    def callback(line):
        global last_time, last_state, receiver_value
        current_time = utime.ticks_us()
        pin_state = pin.value()
        
        if pin_state == 1 and last_state[index] == 0:  # Rising edge
            last_time[index] = current_time
            last_state[index] = 1
        elif pin_state == 0 and last_state[index] == 1:  # Falling edge
            pulse_width = utime.ticks_diff(current_time, last_time[index])
            if 800 < pulse_width < 2200:  # Valid RC PWM range
                receiver_value[index] = pulse_width
            last_state[index] = 0
    return callback

def setup():
    global pins
    
    # Setup pins: X1, X2, X3, X4, X5
    pin_names = ['X1', 'X2', 'X3', 'X4', 'X5']
    channel_names = ['CH1', 'CH2', 'CH3', 'CH5', 'CH6']
    
    print("RC PWM Reader - 4 Channels")
    print("=" * 40)
    
    for i, pin_name in enumerate(pin_names):
        pin = Pin(pin_name, Pin.IN, Pin.PULL_UP)
        pins.append(pin)
        callback = make_callback(i, pin)
        ExtInt(pin, ExtInt.IRQ_RISING_FALLING, Pin.PULL_UP, callback)
        print("{}: {} -> Pin {}".format(channel_names[i], pin_name, pin_name))
    
    print("=" * 40)
    print("Ready!\n")

def test_loop():
    """Test function to print values continuously"""
    print("Reading RC values (Ctrl+C to stop)...")
    print("")
    try:
        while True:
            ch1 = receiver_value[0]  # Left/Right
            ch2 = receiver_value[1]  # ?
            ch3 = receiver_value[2]  # Fwd/Back
            ch5 = receiver_value[3]  # ?
            ch6 = receiver_value[4]  # ?
            
            print("CH1:{:4d} CH2:{:4d} CH3:{:4d} CH5:{:4d} CH6:{:4d}".format(
                ch1, ch2, ch3, ch5, ch6))
            pyb.delay(100)  # Print every 100ms
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    setup()
    test_loop()
