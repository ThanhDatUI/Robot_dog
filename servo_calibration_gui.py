"""
PC Servo Calibration GUI
Graphical interface to calibrate all 12 servos via serial connection to PyBoard

Usage:
1. Upload servo_receiver.py to PyBoard: mpremote cp servo_receiver.py :main.py
2. Reset PyBoard
3. Run this script: python servo_calibration_gui.py
4. Select COM port and connect
5. Use sliders to adjust servo angles
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time

class ServoCalibrationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SpotMicro Servo Calibration Tool")
        self.root.geometry("900x700")
        
        self.serial_port = None
        self.serial_thread = None
        self.running = False
        
        # Default angles from main.py
        self.default_angles = {
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
        
        self.current_angles = self.default_angles.copy()
        self.sliders = {}
        self.labels = {}
        self.entry_vars = {}
        
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
        
        # Servo control frame
        servo_frame = ttk.LabelFrame(self.root, text="Servo Angles", padding=10)
        servo_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create sliders for each servo
        servo_names = {
            0: "FR Shoulder", 1: "FR Leg", 2: "FR Foot",
            4: "FL Shoulder", 5: "FL Leg", 6: "FL Foot",
            8: "RR Shoulder", 9: "RR Leg", 10: "RR Foot",
            12: "RL Shoulder", 13: "RL Leg", 14: "RL Foot",
        }
        
        row = 0
        col = 0
        for channel in [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]:
            frame = ttk.Frame(servo_frame)
            frame.grid(row=row, column=col, padx=10, pady=5, sticky=tk.W)
            
            ttk.Label(frame, text=f"CH{channel}: {servo_names[channel]}", width=15).pack(anchor=tk.W)
            
            # Horizontal container for slider and entry
            control_frame = ttk.Frame(frame)
            control_frame.pack()
            
            slider = tk.Scale(control_frame, from_=0, to=180, orient=tk.HORIZONTAL, length=150,
                            command=lambda val, ch=channel: self.on_slider_change(ch, val))
            slider.set(self.default_angles[channel])
            slider.pack(side=tk.LEFT)
            
            # Entry box for direct input
            entry_var = tk.StringVar(value=str(self.default_angles[channel]))
            entry = ttk.Entry(control_frame, textvariable=entry_var, width=5)
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind('<Return>', lambda e, ch=channel, ev=entry_var: self.on_entry_change(ch, ev))
            entry.bind('<FocusOut>', lambda e, ch=channel, ev=entry_var: self.on_entry_change(ch, ev))
            
            label = ttk.Label(frame, text=f"{self.default_angles[channel]}°")
            label.pack()
            
            self.sliders[channel] = slider
            self.labels[channel] = label
            self.entry_vars[channel] = entry_var
            
            col += 1
            if col > 2:
                col = 0
                row += 1
        
        # Button frame
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(btn_frame, text="Reset to Defaults", command=self.reset_defaults).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Center All (90°)", command=self.center_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Export Config", command=self.export_config).pack(side=tk.LEFT, padx=5)
        
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
            time.sleep(1)  # Wait longer for PyBoard to be ready
            
            self.status_label.config(text="Connected", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.log("Connected to " + port)
            
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
        self.log("Disconnected")
    
    def read_serial(self):
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', 'ignore').strip()
                    if line:
                        self.log("← " + line)
            except:
                pass
            time.sleep(0.01)
    
    def send_command(self, channel, angle):
        if not self.serial_port or not self.serial_port.is_open:
            self.log("Not connected!")
            return
        
        command = f"CH{channel}:{angle}\n"
        try:
            self.serial_port.write(command.encode())
            self.log(f"→ CH{channel}:{angle}")
        except Exception as e:
            self.log(f"Send error: {e}")
    
    def on_slider_change(self, channel, value):
        angle = int(float(value))
        self.current_angles[channel] = angle
        self.labels[channel].config(text=f"{angle}°")
        self.entry_vars[channel].set(str(angle))
        self.send_command(channel, angle)
    
    def on_entry_change(self, channel, entry_var):
        try:
            angle = int(entry_var.get())
            if 0 <= angle <= 180:
                self.sliders[channel].set(angle)
                # on_slider_change will handle the rest
            else:
                # Reset to current value
                entry_var.set(str(self.current_angles[channel]))
                self.log(f"Angle must be 0-180!")
        except ValueError:
            # Reset to current value
            entry_var.set(str(self.current_angles[channel]))
            self.log(f"Invalid number!")
    
    def reset_defaults(self):
        for channel, angle in self.default_angles.items():
            self.sliders[channel].set(angle)
        self.log("Reset to default angles")
    
    def center_all(self):
        for channel in self.sliders.keys():
            self.sliders[channel].set(90)
        self.log("All servos set to 90°")
    
    def export_config(self):
        config = f"""
# Copy to main.py (lines 161-175):

front_right_shoulder = Servo(0, 0, {self.current_angles[0]}, 180, False)
front_right_leg = Servo(1, 0, {self.current_angles[1]}, 180, False)
front_right_foot = Servo(2, 0, {self.current_angles[2]}, 180, False)

front_left_shoulder = Servo(4, 0, {self.current_angles[4]}, 180, True)
front_left_leg = Servo(5, 0, {self.current_angles[5]}, 180, True)
front_left_foot = Servo(6, 0, {self.current_angles[6]}, 180, True)

rear_right_shoulder = Servo(8, 0, {self.current_angles[8]}, 180, False)
rear_right_leg = Servo(9, 0, {self.current_angles[9]}, 180, False)
rear_right_foot = Servo(10, 0, {self.current_angles[10]}, 180, False)

rear_left_shoulder = Servo(12, 0, {self.current_angles[12]}, 180, True)
rear_left_leg = Servo(13, 0, {self.current_angles[13]}, 180, True)
rear_left_foot = Servo(14, 0, {self.current_angles[14]}, 180, True)
"""
        
        # Show in new window
        export_win = tk.Toplevel(self.root)
        export_win.title("Export Configuration")
        export_win.geometry("600x400")
        
        text = scrolledtext.ScrolledText(export_win, wrap=tk.WORD)
        text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        text.insert(tk.END, config)
        text.config(state=tk.DISABLED)
        
        ttk.Button(export_win, text="Copy to Clipboard", 
                  command=lambda: self.copy_to_clipboard(config)).pack(pady=5)
        
        self.log("Configuration exported")
    
    def copy_to_clipboard(self, text):
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        messagebox.showinfo("Success", "Copied to clipboard!")
    
    def log(self, message):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

if __name__ == "__main__":
    root = tk.Tk()
    app = ServoCalibrationGUI(root)
    root.mainloop()
