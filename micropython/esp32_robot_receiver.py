"""
SpotMicro Robot Controller - ESP32-S3 Receiver (WiFi + USB)

Commands: STAND, SIT, LEAN_LEFT/CENTER/RIGHT,
          WALK_FORWARD/CENTER/BACKWARD, CRAWL_LEFT/STOP/RIGHT
Calib:    RAW:ch:angle, CALIB:ch:center, SAVE_HOME, GET_CENTERS, PING
WiFi:     TCP port 8888 (same command format as USB serial)
"""

import utime
import sys
import select
import network
import socket
from machine import I2C, Pin
from pca9685 import PCA9685
from profiler import Profiler
from state import StateMachine

# ============================================================
# WiFi config
# ============================================================
WIFI_SSID = "Tuan Kiet"
WIFI_PASS = "Kiet0708"
TCP_PORT  = 8888

# ============================================================
# USB Serial (sys.stdin/stdout - USB CDC REPL on ESP32-S3)
# ============================================================
class UsbSerial:
    def write(self, data):
        if isinstance(data, bytes):
            data = data.decode('utf-8', 'ignore')
        sys.stdout.write(data)

    def any(self):
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return bool(r)

    def read(self, n=64):
        result = b''
        for _ in range(n):
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if not r:
                break
            ch = sys.stdin.read(1)
            if ch:
                result += ch.encode()
        return result

# ============================================================
# WiFi + TCP Server
# ============================================================
def connect_wifi(ssid, password, timeout_ms=30000):
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    if not sta.isconnected():
        print("Connecting to WiFi:", ssid)
        sta.connect(ssid, password)
        t0 = utime.ticks_ms()
        while not sta.isconnected():
            if utime.ticks_diff(utime.ticks_ms(), t0) > timeout_ms:
                print("WiFi timeout!")
                return None
            utime.sleep_ms(200)
    ip = sta.ifconfig()[0]
    print("WIFI IP:", ip)
    return ip

def make_tcp_server(port):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('', port))
    srv.listen(1)
    srv.setblocking(False)
    print("TCP server listening on port", port)
    return srv

# ============================================================
# Hardware init
# ============================================================
print("Initializing hardware...")

I2C_SDA = 8
I2C_SCL = 9
i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)

devices = i2c.scan()
print("I2C devices:", [hex(d) for d in devices])
if 0x40 not in devices:
    print("WARNING: PCA9685 not found! Check wiring.")

pca = PCA9685(i2c, 0x40)
pca.freq(50)
print("PCA9685 ready")

# ============================================================
# Servo class
# ============================================================
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
        self.last_angle = center_angle_deg  # track last commanded raw angle

    @staticmethod
    def get_12_bit_duty_cycle_for_us(position_us):
        if position_us < Servo.us_min_cmd or position_us > Servo.us_max_cmd:
            raise ValueError
        return int((float(position_us) / float(Servo.period_us)) * (2 ** 12))

    @staticmethod
    def get_12_bit_duty_cycle_for_angle(angle_deg):
        if angle_deg < Servo.deg_min_cmd or angle_deg > Servo.deg_max_cmd:
            raise ValueError
        target_us = Servo.us_min_cmd + (float(angle_deg) * Servo.us_per_deg)
        return Servo.get_12_bit_duty_cycle_for_us(target_us)

    def command_deg(self, angle_deg):
        offset = -angle_deg if self.is_inverted else angle_deg
        angle_raw = self.center_angle_deg + offset
        if angle_raw < self.min_angle_deg or angle_raw > self.max_angle_deg:
            raise ValueError
        self.last_angle = round(angle_raw, 1)
        self.pca9685.duty(self.pwm_channel, Servo.get_12_bit_duty_cycle_for_angle(angle_raw))

# ============================================================
# Servo initialization (calibrated center angles)
# NOTE: Foot centers: non-inv ~20-30, inv ~135-145 (IK constraint)
# ============================================================
front_right_shoulder = Servo(0,  0, 120, 180, False)
front_right_leg      = Servo(1,  0, 100, 180, False)
front_right_foot     = Servo(2,  0,  30, 180, False)

front_left_shoulder  = Servo(4,  0, 100, 180, True)
front_left_leg       = Servo(5,  0, 100, 180, True)
front_left_foot      = Servo(6,  0, 132, 180, True)

rear_right_shoulder  = Servo(8,  0, 115, 180, False)
rear_right_leg       = Servo(9,  0, 105, 180, False)
rear_right_foot      = Servo(10, 0,  20, 180, False)

rear_left_shoulder   = Servo(12, 0, 110, 180, True)
rear_left_leg        = Servo(13, 0,  65, 180, True)
rear_left_foot       = Servo(14, 0, 145, 180, True)

# Profiler + state machine
profiler = Profiler()
state_machine = StateMachine()

# USB serial instance
usb = UsbSerial()

# Servo command list (order matches profiler.get_position_commands())
servo_commands = [
    front_right_shoulder.command_deg, front_left_shoulder.command_deg,
    front_right_leg.command_deg,      front_left_leg.command_deg,
    front_right_foot.command_deg,     front_left_foot.command_deg,
    rear_right_shoulder.command_deg,  rear_left_shoulder.command_deg,
    rear_right_leg.command_deg,       rear_left_leg.command_deg,
    rear_right_foot.command_deg,      rear_left_foot.command_deg,
]

# Dict for runtime servo access by channel
servo_by_channel = {
     0: front_right_shoulder,  1: front_right_leg,   2: front_right_foot,
     4: front_left_shoulder,   5: front_left_leg,    6: front_left_foot,
     8: rear_right_shoulder,   9: rear_right_leg,   10: rear_right_foot,
    12: rear_left_shoulder,   13: rear_left_leg,    14: rear_left_foot,
}

def angle_to_duty_raw(angle):
    angle = max(0, min(180, angle))
    pulse = 500 + (angle / 180.0) * 2000
    return int((pulse / 20000) * 4095)

# ============================================================
# Ready
# ============================================================
print("Ready for PC control commands")
usb.write("READY\n")

# WiFi + TCP server
wifi_ip = connect_wifi(WIFI_SSID, WIFI_PASS)
tcp_server = None
tcp_client = None
tcp_buf    = ""
if wifi_ip:
    tcp_server = make_tcp_server(TCP_PORT)
    usb.write("WIFI_READY {}:{}\n".format(wifi_ip, TCP_PORT))
else:
    usb.write("WIFI_FAIL\n")

# ============================================================
# Crawl / state tracking
# ============================================================
crawl_mode        = 0
crawl_rc_channels = None
current_rc_channels = None
calib_mode        = False

# ============================================================
# Command handler (shared by USB and WiFi)
# ============================================================
def handle_line(cmd, reply_fn):
    global current_rc_channels, crawl_mode, crawl_rc_channels, calib_mode
    if not cmd:
        return
    try:
        if cmd == "STAND":
            current_rc_channels = [980,980,980,980,980,980,1580,1000]+[980]*8
            crawl_mode = 0; calib_mode = False
            reply_fn("OK STAND\n")

        elif cmd == "SIT":
            current_rc_channels = [980,980,980,980,980,980,1580,180]+[980]*8
            crawl_mode = 0; calib_mode = False
            reply_fn("OK SIT\n")

        elif cmd == "LEAN_LEFT":
            current_rc_channels = [980,1380,980,980,980,980,1580,1000]+[980]*8
            crawl_mode = 0; reply_fn("OK LEAN_LEFT\n")

        elif cmd == "LEAN_CENTER":
            current_rc_channels = [980,980,980,980,980,980,1580,1000]+[980]*8
            crawl_mode = 0; reply_fn("OK LEAN_CENTER\n")

        elif cmd == "LEAN_RIGHT":
            current_rc_channels = [980,580,980,980,980,980,1580,1000]+[980]*8
            crawl_mode = 0; reply_fn("OK LEAN_RIGHT\n")

        elif cmd == "WALK_FORWARD":
            current_rc_channels = [1380,980,980,980,980,980,1580,1000]+[980]*8
            crawl_mode = 0; reply_fn("OK WALK_FORWARD\n")

        elif cmd == "WALK_CENTER":
            current_rc_channels = [980,980,980,980,980,980,1580,1000]+[980]*8
            crawl_mode = 0; reply_fn("OK WALK_CENTER\n")

        elif cmd == "WALK_BACKWARD":
            current_rc_channels = [580,980,980,980,980,980,1580,1000]+[980]*8
            crawl_mode = 0; reply_fn("OK WALK_BACKWARD\n")

        elif cmd == "CRAWL_LEFT":
            crawl_mode = 1
            crawl_rc_channels = [1380,980,980,980,980,980,1580,1580]+[980]*8
            current_rc_channels = crawl_rc_channels
            reply_fn("OK CRAWL_LEFT\n")

        elif cmd == "CRAWL_STOP":
            crawl_mode = 0; crawl_rc_channels = None
            current_rc_channels = [980,980,980,980,980,980,1580,1000]+[980]*8
            reply_fn("OK CRAWL_STOP\n")

        elif cmd == "CRAWL_RIGHT":
            crawl_mode = 2
            crawl_rc_channels = [580,980,980,980,980,980,1580,1580]+[980]*8
            current_rc_channels = crawl_rc_channels
            reply_fn("OK CRAWL_RIGHT\n")

        elif cmd.startswith("RAW:"):
            parts = cmd[4:].split(':')
            ch = int(parts[0]); angle = int(parts[1])
            calib_mode = True
            pca.duty(ch, angle_to_duty_raw(angle))
            reply_fn("OK RAW {}:{}\n".format(ch, angle))

        elif cmd.startswith("CALIB:"):
            parts = cmd[6:].split(':')
            ch = int(parts[0]); center = int(parts[1])
            if ch in servo_by_channel:
                servo_by_channel[ch].center_angle_deg = center
                reply_fn("OK CALIB {}:{}\n".format(ch, center))
            else:
                reply_fn("ERROR Invalid channel\n")

        elif cmd == "SAVE_HOME":
            reply_fn("OK SAVE_HOME\n")

        elif cmd == "GET_CENTERS":
            vals = ",".join("{}:{}".format(ch, servo_by_channel[ch].center_angle_deg)
                           for ch in sorted(servo_by_channel))
            reply_fn("CENTERS {}\n".format(vals))

        elif cmd == "PING":
            reply_fn("READY\n")

        else:
            reply_fn("ERROR Unknown command\n")

    except Exception as e:
        reply_fn("ERROR {}\n".format(str(e)))

# ============================================================
# Main loop
# ============================================================
usb_buf        = ""
last_loop_us   = utime.ticks_us()
loop_period_us = 19650  # ~50 Hz
last_angles_ms = utime.ticks_ms()

while True:
    # ---- Read USB serial ----
    if usb.any():
        try:
            data = usb.read(64)
            if data:
                usb_buf += data.decode('utf-8', 'ignore')
        except:
            pass
    while '\n' in usb_buf:
        line, usb_buf = usb_buf.split('\n', 1)
        handle_line(line.strip(), usb.write)

    # ---- Accept new TCP client ----
    if tcp_server and tcp_client is None:
        try:
            tcp_client, _ = tcp_server.accept()
            tcp_client.setblocking(False)
            tcp_buf = ""
            usb.write("TCP CLIENT CONNECTED\n")
            tcp_client.send(b"READY\n")
        except OSError:
            pass

    # ---- Read TCP client ----
    if tcp_client:
        try:
            chunk = tcp_client.recv(256)
            if chunk:
                tcp_buf += chunk.decode('utf-8', 'ignore')
                while '\n' in tcp_buf:
                    line, tcp_buf = tcp_buf.split('\n', 1)
                    def tcp_reply(msg, c=tcp_client):
                        try: c.send(msg.encode())
                        except: pass
                    handle_line(line.strip(), tcp_reply)
            else:
                tcp_client.close(); tcp_client = None
                usb.write("TCP CLIENT DISCONNECTED\n")
        except OSError:
            pass

    # ---- State machine + servos (pause when in calib_mode) ----
    if not calib_mode:
        if current_rc_channels is not None:
            state_machine.update(profiler, current_rc_channels, last_loop_us)
        profiler.tick()
        pos_cmds = profiler.get_position_commands()
        for i, c in enumerate(pos_cmds):
            if c is not None:
                try:
                    servo_commands[i](c)
                except:
                    pass

    # ---- Broadcast servo angles every ~500ms ----
    if utime.ticks_diff(utime.ticks_ms(), last_angles_ms) >= 500:
        last_angles_ms = utime.ticks_ms()
        vals = ",".join("{}:{}".format(ch, int(servo_by_channel[ch].last_angle))
                        for ch in sorted(servo_by_channel))
        msg = "ANGLES {}\n".format(vals)
        usb.write(msg)
        if tcp_client:
            try: tcp_client.send(msg.encode())
            except: pass

    # ---- 50 Hz timing ----
    while utime.ticks_diff(utime.ticks_us(), last_loop_us) < loop_period_us:
        pass
    last_loop_us = utime.ticks_us()
