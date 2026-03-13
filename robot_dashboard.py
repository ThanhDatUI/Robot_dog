"""
SpotMicro Robot Dashboard
=========================
Giao dien tich hop: dieu khien robot + calib servo

Usage:
  python robot_dashboard.py

Requires:
  pip install pyserial
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import socket
import threading
import time
import re
import os

SERVO_DEFAULTS = {
    0:  ("FR Shoulder", 120, False),
    1:  ("FR Leg",      100, False),
    2:  ("FR Foot",      30, False),
    4:  ("FL Shoulder", 100, True),
    5:  ("FL Leg",      105, True),
    6:  ("FL Foot",     132, True),
    8:  ("RR Shoulder", 115, False),
    9:  ("RR Leg",      100, False),
    10: ("RR Foot",      20, False),
    12: ("RL Shoulder", 110, True),
    13: ("RL Leg",       65, True),
    14: ("RL Foot",     145, True),
}
SERVO_CHANNELS = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]

FOOT_CHANNELS = {2, 6, 10, 14}

ESP32_FILE = os.path.join(os.path.dirname(__file__),
                          "micropython", "esp32_robot_receiver.py")

# ─── Color palette ────────────────────────────────────────────────────────────
BG         = "#1a1a2e"
BG2        = "#16213e"
BG3        = "#0f3460"
ACCENT     = "#e94560"
ACCENT2    = "#4CAF50"
ACCENT3    = "#2196F3"
TEXT       = "#eaeaea"
TEXT_DIM   = "#888"
BTN_H      = {"activebackground": "#333", "relief": "flat", "bd": 0, "cursor": "hand2"}


class RobotDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("SpotMicro Dashboard")
        self.root.geometry("1100x720")
        self.root.configure(bg=BG)
        self.root.resizable(True, True)

        self.serial_conn = None   # serial.Serial object
        self.wifi_conn   = None   # socket object
        self.running     = False
        self.robot_ready = False

        # Calib state
        self.calib_centers = {ch: SERVO_DEFAULTS[ch][1] for ch in SERVO_CHANNELS}
        self.entry_vars    = {}   # StringVar for each channel entry

        self._build_ui()
        self._set_controls_state(False)

    # ─── UI ──────────────────────────────────────────────────────────────────
    def _build_ui(self):
        # ── Header ──
        hdr = tk.Frame(self.root, bg=BG3, height=50)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="🤖  SpotMicro Dashboard",
                 font=("Segoe UI", 16, "bold"), bg=BG3, fg=TEXT).pack(side=tk.LEFT, padx=16, pady=8)

        # ── Connection bar ──
        conn = tk.Frame(self.root, bg=BG2, pady=6)
        conn.pack(fill=tk.X)

        # Mode toggle
        self.conn_mode = tk.StringVar(value="serial")
        tk.Radiobutton(conn, text="Serial", variable=self.conn_mode, value="serial",
                       bg=BG2, fg=TEXT, selectcolor=BG3, activebackground=BG2,
                       font=("Segoe UI", 9), command=self._on_mode_change).pack(side=tk.LEFT, padx=(10,2))
        tk.Radiobutton(conn, text="WiFi", variable=self.conn_mode, value="wifi",
                       bg=BG2, fg=TEXT, selectcolor=BG3, activebackground=BG2,
                       font=("Segoe UI", 9), command=self._on_mode_change).pack(side=tk.LEFT, padx=(0,8))

        # Serial widgets
        self.serial_frame = tk.Frame(conn, bg=BG2)
        self.serial_frame.pack(side=tk.LEFT)
        tk.Label(self.serial_frame, text="COM:", bg=BG2, fg=TEXT, font=("Segoe UI", 10)).pack(side=tk.LEFT, padx=(0,4))
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(self.serial_frame, textvariable=self.port_var, width=9,
                                     state="readonly", font=("Segoe UI", 10))
        self.port_cb.pack(side=tk.LEFT, padx=2)
        self._refresh_ports()
        tk.Button(self.serial_frame, text="⟳", font=("Segoe UI", 10), bg=BG3, fg=TEXT,
                  command=self._refresh_ports, **BTN_H).pack(side=tk.LEFT, padx=2)

        # WiFi widgets
        self.wifi_frame = tk.Frame(conn, bg=BG2)
        tk.Label(self.wifi_frame, text="IP:", bg=BG2, fg=TEXT, font=("Segoe UI", 10)).pack(side=tk.LEFT, padx=(0,4))
        self.wifi_ip_var   = tk.StringVar(value="192.168.1.")
        self.wifi_port_var = tk.StringVar(value="8888")
        tk.Entry(self.wifi_frame, textvariable=self.wifi_ip_var, width=14,
                 bg="#0d0d1a", fg=TEXT, font=("Consolas", 10), insertbackground=TEXT,
                 relief="flat").pack(side=tk.LEFT, padx=2)
        tk.Label(self.wifi_frame, text="Port:", bg=BG2, fg=TEXT, font=("Segoe UI", 10)).pack(side=tk.LEFT, padx=(6,2))
        tk.Entry(self.wifi_frame, textvariable=self.wifi_port_var, width=6,
                 bg="#0d0d1a", fg=TEXT, font=("Consolas", 10), insertbackground=TEXT,
                 relief="flat").pack(side=tk.LEFT, padx=2)

        # Connect button + status
        self.conn_btn = tk.Button(conn, text="Connect", font=("Segoe UI", 10, "bold"),
                                  bg=ACCENT, fg="white", padx=12,
                                  command=self._toggle_connection, **BTN_H)
        self.conn_btn.pack(side=tk.LEFT, padx=8)
        self.status_lbl = tk.Label(conn, text="● Disconnected", bg=BG2, fg=ACCENT,
                                   font=("Segoe UI", 10, "bold"))
        self.status_lbl.pack(side=tk.LEFT, padx=4)

        # ── Main area ──
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=6)

        # Left: Control | Right: Calibration
        left  = tk.Frame(main, bg=BG, width=320)
        right = tk.Frame(main, bg=BG)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0,6))
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._build_control_panel(left)
        self._build_calib_panel(right)

        # ── Servo angle monitor ──
        self._build_servo_monitor(self.root)

        # ── Log ──
        log_frame = tk.LabelFrame(self.root, text=" Log ", bg=BG2, fg=TEXT_DIM,
                                  font=("Segoe UI", 9), bd=1, relief="groove")
        log_frame.pack(fill=tk.BOTH, expand=False, padx=10, pady=(0,6), ipady=2)
        self.log = scrolledtext.ScrolledText(log_frame, height=5, bg="#0d0d1a", fg=TEXT,
                                             font=("Consolas", 9), state=tk.DISABLED, bd=0)
        self.log.pack(fill=tk.BOTH, expand=True)

    def _build_control_panel(self, parent):
        tk.Label(parent, text="ROBOT CONTROL", bg=BG, fg=TEXT_DIM,
                 font=("Segoe UI", 9, "bold")).pack(pady=(4,2))

        def ctrl_btn(frame, text, color, cmd, **kwargs):
            b = tk.Button(frame, text=text, font=("Segoe UI", 12, "bold"),
                          bg=color, fg="white", height=2,
                          command=cmd, cursor="hand2", relief="flat", **kwargs)
            return b

        # Stand / Sit
        row1 = tk.Frame(parent, bg=BG); row1.pack(fill=tk.X, pady=3)
        self.stand_btn = ctrl_btn(row1, "▲ Stand Up",   ACCENT2, self.stand_up,  width=14)
        self.sit_btn   = ctrl_btn(row1, "▼ Sit Down",   ACCENT,  self.sit_down,  width=14)
        self.stand_btn.pack(side=tk.LEFT, padx=3)
        self.sit_btn.pack(side=tk.LEFT, padx=3)

        # Lean
        tk.Label(parent, text="── Lean ──", bg=BG, fg=TEXT_DIM,
                 font=("Segoe UI", 9)).pack(pady=(6,2))
        row2 = tk.Frame(parent, bg=BG); row2.pack(fill=tk.X, pady=2)
        self.lean_l = ctrl_btn(row2, "← Left",   "#9C27B0", self.lean_left,   width=8)
        self.lean_c = ctrl_btn(row2, "● Center",  "#607D8B", self.lean_center, width=8)
        self.lean_r = ctrl_btn(row2, "Right →",   "#9C27B0", self.lean_right,  width=8)
        for b in (self.lean_l, self.lean_c, self.lean_r): b.pack(side=tk.LEFT, padx=2)

        # Walk
        tk.Label(parent, text="── Walk ──", bg=BG, fg=TEXT_DIM,
                 font=("Segoe UI", 9)).pack(pady=(6,2))
        row3 = tk.Frame(parent, bg=BG); row3.pack(fill=tk.X, pady=2)
        self.walk_f = ctrl_btn(row3, "↑ Fwd",    ACCENT3,   self.walk_forward,  width=8)
        self.walk_c = ctrl_btn(row3, "● Stop",   "#607D8B",  self.walk_center,   width=8)
        self.walk_b = ctrl_btn(row3, "↓ Back",   ACCENT3,   self.walk_backward, width=8)
        for b in (self.walk_f, self.walk_c, self.walk_b): b.pack(side=tk.LEFT, padx=2)

        # Crawl
        tk.Label(parent, text="── Crawl (Walk) ──", bg=BG, fg=TEXT_DIM,
                 font=("Segoe UI", 9)).pack(pady=(6,2))
        row4 = tk.Frame(parent, bg=BG); row4.pack(fill=tk.X, pady=2)
        self.crawl_l  = ctrl_btn(row4, "⇐ Left",  "#FF9800", self.crawl_left,  width=8)
        self.crawl_st = ctrl_btn(row4, "■ Stop",  "#795548",  self.crawl_stop,  width=8)
        self.crawl_r  = ctrl_btn(row4, "Right ⇒", "#FF9800", self.crawl_right, width=8)
        for b in (self.crawl_l, self.crawl_st, self.crawl_r): b.pack(side=tk.LEFT, padx=2)

        self._ctrl_buttons = [
            self.stand_btn, self.sit_btn,
            self.lean_l, self.lean_c, self.lean_r,
            self.walk_f, self.walk_c, self.walk_b,
            self.crawl_l, self.crawl_st, self.crawl_r,
        ]

        self.robot_status = tk.Label(parent, text="Not Ready", bg=BG, fg=TEXT_DIM,
                                     font=("Segoe UI", 10, "bold"))
        self.robot_status.pack(pady=10)

    def _build_servo_monitor(self, parent):
        """Live servo angle table: 4 legs × 3 joints = 12 cells."""
        frame = tk.LabelFrame(parent, text=" 📊 Live Servo Angles (raw deg) ",
                              bg=BG2, fg=TEXT_DIM, font=("Segoe UI", 9), bd=1, relief="groove")
        frame.pack(fill=tk.X, padx=10, pady=(0,4))

        cols = ["FR", "FL", "RR", "RL"]
        rows = ["Shoulder", "Leg", "Foot"]
        # ch order: Shoulder=[0,4,8,12], Leg=[1,5,9,13], Foot=[2,6,10,14]
        monitor_channels = [
            [0, 4,  8, 12],
            [1, 5,  9, 13],
            [2, 6, 10, 14],
        ]

        self._angle_labels = {}  # ch -> tk.Label

        # Header
        tk.Label(frame, text="", bg=BG2, width=8).grid(row=0, column=0, padx=4)
        for ci, col in enumerate(cols):
            tk.Label(frame, text=col, bg=BG2, fg=ACCENT3,
                     font=("Segoe UI", 9, "bold"), width=8).grid(row=0, column=ci+1, padx=4)

        for ri, (row_name, chs) in enumerate(zip(rows, monitor_channels)):
            tk.Label(frame, text=row_name, bg=BG2, fg=TEXT_DIM,
                     font=("Segoe UI", 9), width=8, anchor="w").grid(row=ri+1, column=0, padx=(8,4), pady=2)
            for ci, ch in enumerate(chs):
                lbl = tk.Label(frame, text="--", bg="#0d0d1a", fg=TEXT,
                               font=("Consolas", 10, "bold"), width=8, relief="flat")
                lbl.grid(row=ri+1, column=ci+1, padx=4, pady=2)
                self._angle_labels[ch] = lbl

    def _update_angle_monitor(self, line):
        """Parse ANGLES 0:120,1:100,... and update labels."""
        data = line[7:]  # strip 'ANGLES '
        for pair in data.split(","):
            try:
                ch_str, val_str = pair.split(":")
                ch, val = int(ch_str), int(val_str)
                if ch in self._angle_labels:
                    lbl = self._angle_labels[ch]
                    # Color-code: near center=white, far=yellow/red
                    center = SERVO_DEFAULTS.get(ch, (None, 90))[1]
                    diff = abs(val - center)
                    if diff < 15:   color = "#eaeaea"
                    elif diff < 40: color = "#FFD600"
                    else:           color = "#FF6B6B"
                    lbl.config(text=str(val), fg=color)
            except Exception:
                pass

    def _build_calib_panel(self, parent):
        hdr = tk.Frame(parent, bg=BG2); hdr.pack(fill=tk.X, pady=(0,4))
        tk.Label(hdr, text="SERVO CALIBRATION  (center angles)",
                 bg=BG2, fg=TEXT, font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=8, pady=4)
        tk.Label(hdr, text="⚠ Foot: keep non-inv ~30°, inv ~145°", bg=BG2, fg="#FF9800",
                 font=("Segoe UI", 8)).pack(side=tk.RIGHT, padx=8)

        grid = tk.Frame(parent, bg=BG); grid.pack(fill=tk.BOTH, expand=True)

        groups = [
            ("FRONT RIGHT", [0, 1, 2]),
            ("FRONT LEFT",  [4, 5, 6]),
            ("REAR RIGHT",  [8, 9, 10]),
            ("REAR LEFT",   [12, 13, 14]),
        ]
        for col, (title, channels) in enumerate(groups):
            gf = tk.LabelFrame(grid, text=title, bg=BG2, fg=TEXT_DIM,
                               font=("Segoe UI", 8, "bold"), bd=1, relief="groove",
                               padx=6, pady=4)
            gf.grid(row=0, column=col, padx=4, pady=2, sticky="nsew")
            grid.columnconfigure(col, weight=1)

            for ch in channels:
                name, default_center, _ = SERVO_DEFAULTS[ch]
                is_foot = ch in FOOT_CHANNELS

                sf = tk.Frame(gf, bg=BG2); sf.pack(fill=tk.X, pady=5)

                color = "#FF9800" if is_foot else TEXT
                tk.Label(sf, text=f"CH{ch:02d} {name}", bg=BG2, fg=color,
                         font=("Segoe UI", 8), anchor="w").pack(anchor="w")

                row = tk.Frame(sf, bg=BG2); row.pack(anchor="w")

                var = tk.StringVar(value=str(default_center))
                self.entry_vars[ch] = var

                # − button
                tk.Button(row, text="−", width=2, font=("Segoe UI", 10, "bold"),
                          bg=BG3, fg=TEXT, relief="flat", cursor="hand2",
                          command=lambda c=ch: self._adjust(c, -1)).pack(side=tk.LEFT)

                e = tk.Entry(row, textvariable=var, width=5,
                             font=("Consolas", 11, "bold"),
                             bg="#0d0d1a", fg="#FF9800" if is_foot else ACCENT3,
                             insertbackground=TEXT,
                             relief="flat", justify="center")
                e.pack(side=tk.LEFT, padx=3)
                e.bind("<Return>",   lambda ev, c=ch: self._send_raw(c))
                e.bind("<FocusOut>", lambda ev, c=ch: self._sync_entry(c))

                tk.Button(row, text="+", width=2, font=("Segoe UI", 10, "bold"),
                          bg=BG3, fg=TEXT, relief="flat", cursor="hand2",
                          command=lambda c=ch: self._adjust(c, +1)).pack(side=tk.LEFT)

                tk.Button(row, text="Send", font=("Segoe UI", 8),
                          bg="#FF9800" if is_foot else ACCENT3,
                          fg="white", relief="flat", cursor="hand2", padx=4,
                          command=lambda c=ch: self._send_raw(c)).pack(side=tk.LEFT, padx=(6,0))

        # Bottom buttons
        btn_row = tk.Frame(parent, bg=BG); btn_row.pack(fill=tk.X, pady=6)
        tk.Button(btn_row, text="📡  Get Current",
                  font=("Segoe UI", 10), bg=BG3, fg=TEXT, padx=10,
                  command=self._get_centers, cursor="hand2", relief="flat").pack(side=tk.LEFT, padx=6)
        tk.Button(btn_row, text="↺  Reset Defaults",
                  font=("Segoe UI", 10), bg="#455A64", fg=TEXT, padx=10,
                  command=self._reset_defaults, cursor="hand2", relief="flat").pack(side=tk.LEFT, padx=6)
        self.save_btn = tk.Button(btn_row, text="💾  Set as Home",
                                  font=("Segoe UI", 10, "bold"), bg=ACCENT2, fg="white", padx=14,
                                  command=self._save_home, cursor="hand2", relief="flat",
                                  state=tk.DISABLED)
        self.save_btn.pack(side=tk.RIGHT, padx=6)

    # ─── Connection ──────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports: self.port_cb.current(0)

    def _on_mode_change(self):
        if self.conn_mode.get() == "wifi":
            self.serial_frame.pack_forget()
            self.wifi_frame.pack(side=tk.LEFT)
        else:
            self.wifi_frame.pack_forget()
            self.serial_frame.pack(side=tk.LEFT)

    def _is_connected(self):
        return ((self.serial_conn and self.serial_conn.is_open) or
                (self.wifi_conn is not None))

    def _toggle_connection(self):
        if self._is_connected():
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        mode = self.conn_mode.get()
        try:
            if mode == "wifi":
                ip   = self.wifi_ip_var.get().strip()
                port = int(self.wifi_port_var.get().strip())
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                sock.connect((ip, port))
                sock.settimeout(0.1)
                self.wifi_conn = sock
                self._log(f"Connected to {ip}:{port} via WiFi")
            else:
                port = self.port_var.get()
                if not port:
                    messagebox.showerror("Error", "Please select a COM port")
                    return
                self.serial_conn = serial.Serial(port, 115200, timeout=0.1)
                time.sleep(1.5)
                self._log(f"Connected to {port} via Serial")

            self.status_lbl.config(text="● Connected", fg=ACCENT2)
            self.conn_btn.config(text="Disconnect", bg="#607D8B")
            self.running = True
            threading.Thread(target=self._read_loop, daemon=True).start()
            time.sleep(0.2)
            self._send("PING")
            # Auto-sync calib UI with ESP32 after 1.5s
            self.root.after(1500, self._get_centers)
            self.root.after(3000, self._fallback_ready)
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def _fallback_ready(self):
        if not self.robot_ready:
            self._log("(Auto-enabled: READY not received, assuming board is up)")
            self._on_ready()

    def _disconnect(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.serial_conn = None
        if self.wifi_conn:
            try: self.wifi_conn.close()
            except: pass
            self.wifi_conn = None
        self.robot_ready = False
        self.status_lbl.config(text="● Disconnected", fg=ACCENT)
        self.conn_btn.config(text="Connect", bg=ACCENT)
        self._set_controls_state(False)
        self._log("Disconnected")

    def _read_loop(self):
        buf = ""
        while self.running:
            try:
                data = b""
                if self.serial_conn and self.serial_conn.is_open:
                    if self.serial_conn.in_waiting:
                        data = self.serial_conn.read(self.serial_conn.in_waiting)
                elif self.wifi_conn:
                    try: data = self.wifi_conn.recv(512)
                    except socket.timeout: pass
                    except OSError: break

                if data:
                    buf += data.decode("utf-8", "ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line: continue
                    if line.startswith("ANGLES "):
                        # Silent update – don't spam the log
                        self.root.after(0, lambda l=line: self._update_angle_monitor(l))
                        continue
                    self._log(f"← {line}")
                    if line == "READY":
                        self.robot_ready = True
                        self.root.after(0, self._on_ready)
                    elif line.startswith("WIFI_READY "):
                        self._log(f"ESP32 WiFi ready at {line[11:]}")
                    elif line.startswith("CENTERS "):
                        self.root.after(0, lambda l=line: self._parse_centers(l))
            except Exception:
                pass
            time.sleep(0.01)

    def _on_ready(self):
        self._set_controls_state(True)
        self.robot_status.config(text="Ready ✓", fg=ACCENT2)

    # ─── Commands ────────────────────────────────────────────────────────────
    def _send(self, cmd):
        if not self._is_connected():
            self._log("Not connected"); return
        try:
            payload = (cmd + "\n").encode()
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.write(payload)
            elif self.wifi_conn:
                self.wifi_conn.sendall(payload)
            self._log(f"→ {cmd}")
        except Exception as e:
            self._log(f"Send error: {e}")

    def stand_up(self):
        self._send("STAND")
        self.robot_status.config(text="Standing Up...", fg="#FF9800")

    def sit_down(self):
        self._send("SIT")
        self.robot_status.config(text="Sitting Down...", fg="#FF9800")

    def lean_left(self):    self._send("LEAN_LEFT");    self.robot_status.config(text="Lean Left",    fg="#9C27B0")
    def lean_center(self):  self._send("LEAN_CENTER");  self.robot_status.config(text="Centered",     fg=ACCENT2)
    def lean_right(self):   self._send("LEAN_RIGHT");   self.robot_status.config(text="Lean Right",   fg="#9C27B0")
    def walk_forward(self): self._send("WALK_FORWARD"); self.robot_status.config(text="Moving Fwd",   fg=ACCENT3)
    def walk_center(self):  self._send("WALK_CENTER");  self.robot_status.config(text="Stopped",      fg=ACCENT2)
    def walk_backward(self):self._send("WALK_BACKWARD");self.robot_status.config(text="Moving Back",  fg=ACCENT3)
    def crawl_left(self):   self._send("CRAWL_LEFT");   self.robot_status.config(text="Walk Left 🚶", fg="#FF9800")
    def crawl_stop(self):   self._send("CRAWL_STOP");   self.robot_status.config(text="Stopped",      fg=ACCENT2)
    def crawl_right(self):  self._send("CRAWL_RIGHT");  self.robot_status.config(text="Walk Right 🚶",fg="#FF9800")

    # ─── Calibration ─────────────────────────────────────────────────────────
    def _sync_entry(self, ch):
        """Clamp entry value to 0-180 when focus leaves."""
        try:
            v = int(self.entry_vars[ch].get())
            v = max(0, min(180, v))
            self.entry_vars[ch].set(str(v))
            self.calib_centers[ch] = v
        except ValueError:
            self.entry_vars[ch].set(str(self.calib_centers[ch]))

    def _adjust(self, ch, delta):
        """Increment/decrement entry by delta then send RAW."""
        try:
            v = int(self.entry_vars[ch].get()) + delta
            v = max(0, min(180, v))
            self.entry_vars[ch].set(str(v))
            self.calib_centers[ch] = v
            self._send(f"RAW:{ch}:{v}")
        except ValueError:
            pass

    def _send_raw(self, ch):
        """Read entry value and send RAW command."""
        try:
            v = int(self.entry_vars[ch].get())
            v = max(0, min(180, v))
            self.entry_vars[ch].set(str(v))
            self.calib_centers[ch] = v
            self._send(f"RAW:{ch}:{v}")
        except ValueError:
            pass

    def _get_centers(self):
        self._send("GET_CENTERS")

    def _parse_centers(self, line):
        # CENTERS 0:120,1:110,...
        data = line.replace("CENTERS ", "")
        for pair in data.split(","):
            try:
                ch_str, val_str = pair.split(":")
                ch, val = int(ch_str), int(val_str)
                if ch in self.entry_vars and ch not in FOOT_CHANNELS:
                    self.entry_vars[ch].set(str(val))
                    self.calib_centers[ch] = val
            except Exception:
                pass

    def _reset_defaults(self):
        for ch in SERVO_CHANNELS:
            default = SERVO_DEFAULTS[ch][1]
            self.entry_vars[ch].set(str(default))
            self.calib_centers[ch] = default
        self._log("Reset to defaults")

    def _save_home(self):
        """Send CALIB commands to ESP32 and update esp32_robot_receiver.py on disk."""
        # 1) Send CALIB commands
        for ch in SERVO_CHANNELS:
            center = self.calib_centers[ch]
            self._send(f"CALIB:{ch}:{center}")
            time.sleep(0.05)
        self._send("SAVE_HOME")

        # 2) Update esp32_robot_receiver.py on disk
        try:
            self._update_esp32_file()
            self._log("✓ esp32_robot_receiver.py updated on disk")
            self._log("→ Upload to ESP32: mpremote connect COM9 cp esp32_robot_receiver.py :main.py + reset")
        except Exception as e:
            self._log(f"File update error: {e}")

        messagebox.showinfo("Home Saved",
            "New home angles sent to ESP32 (RAM).\n"
            "esp32_robot_receiver.py updated on disk.\n\n"
            "To persist after reboot:\n"
            "mpremote connect COM9 cp micropython/esp32_robot_receiver.py :main.py + reset")

    def _update_esp32_file(self):
        """Patch esp32_robot_receiver.py with current calib_centers."""
        if not os.path.exists(ESP32_FILE):
            return

        with open(ESP32_FILE, "r", encoding="utf-8") as f:
            content = f.read()

        # Servo name mapping
        names = {
            0: "front_right_shoulder", 1: "front_right_leg",  2: "front_right_foot",
            4: "front_left_shoulder",  5: "front_left_leg",   6: "front_left_foot",
            8: "rear_right_shoulder",  9: "rear_right_leg",  10: "rear_right_foot",
           12: "rear_left_shoulder",  13: "rear_left_leg",   14: "rear_left_foot",
        }
        for ch, var_name in names.items():
            center  = self.calib_centers[ch]
            _, _, inv = SERVO_DEFAULTS[ch]
            inv_str = "True" if inv else "False"
            # Match pattern: var_name = Servo(ch, 0, <old_center>, 180, inv_str)
            pattern = rf'({re.escape(var_name)}\s*=\s*Servo\({ch},\s*0,\s*)\d+(\s*,\s*180,\s*{inv_str}\))'
            replacement = rf'\g<1>{center}\2'
            content = re.sub(pattern, replacement, content)

        with open(ESP32_FILE, "w", encoding="utf-8") as f:
            f.write(content)

    # ─── Helpers ─────────────────────────────────────────────────────────────
    def _set_controls_state(self, enabled):
        state = tk.NORMAL if enabled else tk.DISABLED
        for b in self._ctrl_buttons if hasattr(self, "_ctrl_buttons") else []:
            b.config(state=state)
        self.save_btn.config(state=state)

    def _log(self, msg):
        self.log.config(state=tk.NORMAL)
        self.log.insert(tk.END, f"{time.strftime('%H:%M:%S')}  {msg}\n")
        self.log.see(tk.END)
        self.log.config(state=tk.DISABLED)


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotDashboard(root)
    root.mainloop()
