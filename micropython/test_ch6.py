import utime
from machine import I2C, Pin
from pca9685 import PCA9685

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
pca = PCA9685(i2c, 0x40)
pca.freq(50)

def duty(a):
    return int(((500 + a / 180 * 2000) / 20000) * 4095)

# Doi kenh o day: 6 = FL Foot
CH = 10
print("Sweeping CH{} (Ctrl+C to stop)".format(CH))

while True:
    for a in range(0, 91, 5):
        pca.duty(CH, duty(a))
        utime.sleep_ms(50)
    for a in range(90, -1, -5):
        pca.duty(CH, duty(a))
        utime.sleep_ms(50)
