"""
SpotMicro Robot Controller GUI
Simple controller to make robot stand/sit via serial commands

Usage:
1. Upload robot_receiver.py to PyBoard as main.py
2. Reset PyBoard  
3. Run this GUI: python robot_controller.py
4. Click "Stand Up" or "Sit Down"
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time

class RobotControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SpotMicro Robot Controller")
        self.root.geometry("600x500")
        
        self.serial_port = None
        self.serial_thread = None
        self.running = False
        self.robot_ready = False
        
        self.create_widgets()
    
    def create_widgets(self):
        # Connection frame
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, padx=5)
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()
        
        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="Not connected", foreground="red")
        self.status_label.grid(row=0, column=4, padx=10)
        
        # Control frame
        ctrl_frame = ttk.LabelFrame(self.root, text="Robot Control", padding=20)
        ctrl_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Big buttons
        self.stand_btn = tk.Button(ctrl_frame, text="Stand Up", font=("Arial", 24, "bold"),
                                   bg="#4CAF50", fg="white", height=3, width=15,
                                   command=self.stand_up, state=tk.DISABLED)
        self.stand_btn.pack(pady=10)
        
        self.sit_btn = tk.Button(ctrl_frame, text="Sit Down", font=("Arial", 24, "bold"),
                                bg="#f44336", fg="white", height=3, width=15,
                                command=self.sit_down, state=tk.DISABLED)
        self.sit_btn.pack(pady=10)
        
        # Lean buttons frame
        lean_frame = ttk.Frame(ctrl_frame)
        lean_frame.pack(pady=10)
        
        self.lean_left_btn = tk.Button(lean_frame, text="← Left", font=("Arial", 14, "bold"),
                                       bg="#9C27B0", fg="white", height=2, width=8,
                                       command=self.lean_left, state=tk.DISABLED)
        self.lean_left_btn.pack(side=tk.LEFT, padx=5)
        
        self.lean_center_btn = tk.Button(lean_frame, text="● Center", font=("Arial", 14, "bold"),
                                         bg="#607D8B", fg="white", height=2, width=8,
                                         command=self.lean_center, state=tk.DISABLED)
        self.lean_center_btn.pack(side=tk.LEFT, padx=5)
        
        self.lean_right_btn = tk.Button(lean_frame, text="Right →", font=("Arial", 14, "bold"),
                                        bg="#9C27B0", fg="white", height=2, width=8,
                                        command=self.lean_right, state=tk.DISABLED)
        self.lean_right_btn.pack(side=tk.LEFT, padx=5)
        
        # Walk buttons frame (Forward/Backward)
        walk_frame = ttk.Frame(ctrl_frame)
        walk_frame.pack(pady=10)
        
        self.walk_forward_btn = tk.Button(walk_frame, text="↑ Forward", font=("Arial", 14, "bold"),
                                          bg="#2196F3", fg="white", height=2, width=8,
                                          command=self.walk_forward, state=tk.DISABLED)
        self.walk_forward_btn.pack(side=tk.LEFT, padx=5)
        
        self.walk_center_btn = tk.Button(walk_frame, text="● Center", font=("Arial", 14, "bold"),
                                         bg="#607D8B", fg="white", height=2, width=8,
                                         command=self.walk_center, state=tk.DISABLED)
        self.walk_center_btn.pack(side=tk.LEFT, padx=5)
        
        self.walk_backward_btn = tk.Button(walk_frame, text="↓ Backward", font=("Arial", 14, "bold"),
                                           bg="#2196F3", fg="white", height=2, width=8,
                                           command=self.walk_backward, state=tk.DISABLED)
        self.walk_backward_btn.pack(side=tk.LEFT, padx=5)
        
        # Crawl buttons frame (Actual walking)
        crawl_frame = ttk.Frame(ctrl_frame)
        crawl_frame.pack(pady=10)
        
        ttk.Label(crawl_frame, text="WALK:", font=("Arial", 12, "bold")).pack(side=tk.LEFT, padx=5)
        
        self.crawl_left_btn = tk.Button(crawl_frame, text="⇐ Walk Left", font=("Arial", 14, "bold"),
                                        bg="#FF9800", fg="white", height=2, width=10,
                                        command=self.crawl_left, state=tk.DISABLED)
        self.crawl_left_btn.pack(side=tk.LEFT, padx=5)
        
        self.crawl_stop_btn = tk.Button(crawl_frame, text="■ Stop", font=("Arial", 14, "bold"),
                                        bg="#795548", fg="white", height=2, width=8,
                                        command=self.crawl_stop, state=tk.DISABLED)
        self.crawl_stop_btn.pack(side=tk.LEFT, padx=5)
        
        self.crawl_right_btn = tk.Button(crawl_frame, text="Walk Right ⇒", font=("Arial", 14, "bold"),
                                         bg="#FF9800", fg="white", height=2, width=10,
                                         command=self.crawl_right, state=tk.DISABLED)
        self.crawl_right_btn.pack(side=tk.LEFT, padx=5)
        
        # Status
        self.robot_status = ttk.Label(ctrl_frame, text="Robot: Not Ready", 
                                     font=("Arial", 14), foreground="gray")
        self.robot_status.pack(pady=10)
        
        # Log frame
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)
    
    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
    
    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Please select a COM port")
            return
        
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(1.5)  # Wait for PyBoard to initialize
            
            self.status_label.config(text="Connected", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.log("Connected to " + port)
            
            # Enable controls immediately (don't wait for READY signal)
            self.robot_ready = True
            self.update_controls()
            self.robot_status.config(text="Robot: Ready ✓", foreground="green")
            
            # Start reading thread
            self.running = True
            self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.serial_thread.start()
            
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            self.log(f"Connection error: {e}")
    
    def disconnect(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.status_label.config(text="Not connected", foreground="red")
        self.connect_btn.config(text="Connect")
        self.robot_ready = False
        self.update_controls()
        self.log("Disconnected")
    
    def read_serial(self):
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', 'ignore').strip()
                    if line:
                        self.log("← " + line)
                        if line == "READY":
                            self.robot_ready = True
                            self.update_controls()
                            self.robot_status.config(text="Robot: Ready ✓", foreground="green")
            except:
                pass
            time.sleep(0.01)
    
    def update_controls(self):
        state = tk.NORMAL if self.robot_ready else tk.DISABLED
        self.stand_btn.config(state=state)
        self.sit_btn.config(state=state)
        self.lean_left_btn.config(state=state)
        self.lean_center_btn.config(state=state)
        self.lean_right_btn.config(state=state)
        self.walk_forward_btn.config(state=state)
        self.walk_center_btn.config(state=state)
        self.walk_backward_btn.config(state=state)
        self.crawl_left_btn.config(state=state)
        self.crawl_stop_btn.config(state=state)
        self.crawl_right_btn.config(state=state)
    
    def send_command(self, command):
        if not self.serial_port or not self.serial_port.is_open:
            self.log("Not connected!")
            return
        
        if not self.robot_ready:
            self.log("Robot not ready!")
            return
        
        try:
            self.serial_port.write((command + "\n").encode())
            self.log(f"→ {command}")
        except Exception as e:
            self.log(f"Send error: {e}")
    
    def stand_up(self):
        self.send_command("STAND")
        self.robot_status.config(text="Robot: Standing Up...", foreground="orange")
    
    def sit_down(self):
        self.send_command("SIT")
        self.robot_status.config(text="Robot: Sitting Down...", foreground="orange")
    
    def lean_left(self):
        self.send_command("LEAN_LEFT")
        self.robot_status.config(text="Robot: Leaning Left", foreground="purple")
    
    def lean_center(self):
        self.send_command("LEAN_CENTER")
        self.robot_status.config(text="Robot: Centered", foreground="green")
    
    def lean_right(self):
        self.send_command("LEAN_RIGHT")
        self.robot_status.config(text="Robot: Leaning Right", foreground="purple")
    
    def walk_forward(self):
        self.send_command("WALK_FORWARD")
        self.robot_status.config(text="Robot: Moving Forward", foreground="blue")
    
    def walk_center(self):
        self.send_command("WALK_CENTER")
        self.robot_status.config(text="Robot: Centered", foreground="green")
    
    def walk_backward(self):
        self.send_command("WALK_BACKWARD")
        self.robot_status.config(text="Robot: Moving Backward", foreground="blue")
    
    def crawl_left(self):
        self.send_command("CRAWL_LEFT")
        self.robot_status.config(text="Robot: Walking Left 🚶", foreground="orange")
    
    def crawl_stop(self):
        self.send_command("CRAWL_STOP")
        self.robot_status.config(text="Robot: Stopped", foreground="green")
    
    def crawl_right(self):
        self.send_command("CRAWL_RIGHT")
        self.robot_status.config(text="Robot: Walking Right 🚶", foreground="orange")
    
    def log(self, message):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControllerGUI(root)
    root.mainloop()
