"""
Test PCA9685 Servo Driver trên ESP32-S3
========================================

Wiring ESP32-S3 → PCA9685:
  GPIO 8  → SDA
  GPIO 9  → SCL
  3.3V    → VCC
  GND     → GND

Wiring PCA9685 → Servo:
  V+      → 5-6V nguồn ngoài (KHÔNG dùng 3.3V từ ESP32!)
  GND     → GND chung
  Channel → Servo signal

CÁCH DÙNG:
  1. Copy file này lên ESP32-S3 (dùng Thonny hoặc mpremote)
  2. Chạy: exec(open('test_pca9685_esp32.py').read())
  3. Hoặc import test_pca9685_esp32
"""

import utime
from machine import I2C, Pin
from pca9685 import PCA9685

I2C_SDA_PIN = 8       # GPIO SDA
I2C_SCL_PIN = 9       # GPIO SCL
I2C_FREQ    = 400000  # 400kHz
PCA_ADDR    = 0x40    # Địa chỉ I2C mặc định của PCA9685
PWM_FREQ_HZ = 50      # Tần số PWM cho servo (50Hz)

SERVO_US_MIN = 500    # 0 độ
SERVO_US_MAX = 2500   # 180 độ

def angle_to_duty(angle_deg):
    """Convert góc (0-180) sang 12-bit duty cycle cho PCA9685."""
    angle_deg = max(0, min(180, angle_deg))  # clamp
    period_us = 1_000_000 / PWM_FREQ_HZ      # 20000 us
    pulse_us = SERVO_US_MIN + (angle_deg / 180.0) * (SERVO_US_MAX - SERVO_US_MIN)
    duty = int((pulse_us / period_us) * 4095)
    return duty


def move_servo(pca, channel, angle_deg, delay_ms=500):
    """Di chuyển servo đến góc chỉ định và in log."""
    duty = angle_to_duty(angle_deg)
    print("  CH{:02d} → {:3d}° (duty={})".format(channel, angle_deg, duty))
    pca.duty(channel, duty)
    utime.sleep_ms(delay_ms)


# ============================================================
# I2C scan
# ============================================================
def scan_i2c(i2c):
    print("\n[1] Scanning I2C bus...")
    devices = i2c.scan()
    if devices:
        print("    Found:", [hex(d) for d in devices])
    else:
        print("    No devices found! Check wiring.")
    return devices


# ============================================================
# Test A: Sweep một kênh servo
# ============================================================
def test_sweep(pca, channel=0):
    print("\n[TEST A] Sweep servo - CH{:02d}".format(channel))
    print("  Servo sẽ quét từ 0° → 90° → 180° → 90° → 0°")
    for angle in [0, 45, 90, 135, 180, 135, 90, 45, 0]:
        move_servo(pca, channel, angle, delay_ms=600)
    print("  ✓ Sweep done")


# ============================================================
# Test B: Test từng góc cụ thể (nhập từ bàn phím)
# ============================================================
def test_manual(pca):
    print("\n[TEST B] Manual angle control")
    print("  Nhập 'CH,ANGLE' (vd: 0,90) hoặc 'q' để thoát")
    while True:
        try:
            line = input("  > ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line.lower() == 'q':
            break
        try:
            parts = line.split(',')
            ch = int(parts[0].strip())
            ang = int(parts[1].strip())
            if not (0 <= ch <= 15):
                print("  ERROR: channel phải từ 0-15")
                continue
            if not (0 <= ang <= 180):
                print("  ERROR: góc phải từ 0-180")
                continue
            move_servo(pca, ch, ang, delay_ms=300)
        except Exception as e:
            print("  ERROR:", e)
            print("  Format: CH,ANGLE  vd: 0,90")


# ============================================================
# Test C: Test tất cả 12 kênh servo cùng lúc
# ============================================================
def test_all_channels(pca):
    """Test 12 kênh servo (mapping đúng với robot) cùng lúc."""
    channels = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    labels = [
        "FR_shoulder", "FR_leg", "FR_foot",
        "FL_shoulder", "FL_leg", "FL_foot",
        "RR_shoulder", "RR_leg", "RR_foot",
        "RL_shoulder", "RL_leg", "RL_foot",
    ]
    print("\n[TEST C] All 12 servo channels")
    for angle in [90, 0, 180, 90]:
        print("  → All servos to {}°".format(angle))
        for ch, label in zip(channels, labels):
            print("    CH{:02d} ({})".format(ch, label))
            pca.duty(ch, angle_to_duty(angle))
        utime.sleep_ms(1000)
    print("  ✓ All channels done")


# ============================================================
# MAIN
# ============================================================
def main():
    print("=" * 50)
    print("  PCA9685 Servo Test - ESP32-S3")
    print("  SDA=GPIO{}, SCL=GPIO{}".format(I2C_SDA_PIN, I2C_SCL_PIN))
    print("=" * 50)

    # Init I2C
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)

    # Scan
    devices = scan_i2c(i2c)
    if PCA_ADDR not in devices:
        print("\n❌ PCA9685 KHÔNG tìm thấy tại 0x{:02X}!".format(PCA_ADDR))
        print("   Kiểm tra lại:")
        print("   - Dây nối SDA/SCL")
        print("   - Nguồn VCC 3.3V cho PCA9685")
        print("   - Địa chỉ I2C (A0-A5 jumper trên board)")
        return
    print("    ✓ PCA9685 found at 0x{:02X}".format(PCA_ADDR))

    # Init PCA9685
    print("\n[2] Initializing PCA9685...")
    pca = PCA9685(i2c, PCA_ADDR)
    pca.freq(PWM_FREQ_HZ)
    print("    ✓ Frequency set to {}Hz".format(PWM_FREQ_HZ))

    # Chọn test
    print("\n[3] Chọn test:")
    print("    A - Sweep 1 servo (CH0)")
    print("    B - Manual control (nhập tay góc)")
    print("    C - Test tất cả 12 kênh")
    print("    X - Chạy hết A + C rồi thoát")

    try:
        choice = input("\nChọn (A/B/C/X): ").strip().upper()
    except (EOFError, KeyboardInterrupt):
        choice = 'X'

    if choice == 'A':
        test_sweep(pca, channel=0)
    elif choice == 'B':
        test_manual(pca)
    elif choice == 'C':
        test_all_channels(pca)
    else:  # X hoặc mặc định
        test_sweep(pca, channel=0)
        test_all_channels(pca)

    # Center tất cả về 90° khi kết thúc
    print("\n[4] Centering all servos to 90°...")
    for ch in [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]:
        pca.duty(ch, angle_to_duty(90))
    utime.sleep_ms(500)
    print("    ✓ Done!")
    print("\n✅ Test hoàn tất!")


main()
