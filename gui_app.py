import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import struct
import collections
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time
import csv


# Data transfer variables
BAUD_RATE = 921600 
CHUNK_SAMPLES = 2000 
N_LUT = 1000            

# Communication protocol headers 
HEADER_SYNC = b'\xAA\xBB'
MODE_STEP = 1
MODE_FREQ = 2

class RealTimeScope:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Real-Time Oscilloscope")
        self.root.geometry("1200x800")
        
        # Variables to handle correct closing
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.is_closing = False
        self.animate_future = None

        # Interface state
        self.ser = None
        self.is_running = False
        self.stop_thread = False
        
        # Circular buffer initialization
        self.max_points = 10000
        self.data_buffer = collections.deque([0]*self.max_points, maxlen=self.max_points)

        # Record variables
        self.is_recording = False
        self.record_file = None
        self.record_cnt = 0
        self.sample_idx = 0
        self.sample_time = 0
        self.record_time = 0

        self._setup_ui()
        
        # Avvia il timer grafico
        self.animate_loop()

    def _setup_ui(self):
        # Control section
        control_frame = ttk.LabelFrame(self.root, text="Control panel", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        # COM Port management panel
        ttk.Label(control_frame, text="COM Port:").pack(anchor=tk.W)
        self.port_combo = ttk.Combobox(control_frame, width=15)
        self.port_combo.pack(pady=5)
        self.refresh_ports()
        ttk.Button(control_frame, text="Refresh Ports", command=self.refresh_ports).pack(pady=5)

        # Connection panel
        self.btn_connect = ttk.Button(control_frame, text="Connect", command=self.toggle_connection) 
        self.btn_connect.pack(pady=20, fill=tk.X)

        ttk.Separator(control_frame, orient='horizontal').pack(fill='x', pady=10)

        # Step response management panel
        ttk.Label(control_frame, text="Step Response (One-Shot):").pack(anchor=tk.W)
        self.btn_step = ttk.Button(control_frame, text="Shoot step", command=self.send_step, state=tk.DISABLED)
        self.btn_step.pack(pady=5, fill=tk.X)

        ttk.Separator(control_frame, orient='horizontal').pack(fill='x', pady=10)

        # Sine analysis panel
        ttk.Label(control_frame, text="Armonic response (Streaming):").pack(anchor=tk.W)

        sine_box = ttk.Frame(control_frame)
        sine_box.pack(fill=tk.X, pady=5)

        self.sine_entry = ttk.Entry(sine_box, width=8)
        self.sine_entry.insert(0, "3")
        self.sine_entry.pack(side=tk.LEFT, padx=5)

        ttk.Label(sine_box, text="Hz").pack(side=tk.LEFT)

        self.btn_sine = ttk.Button(control_frame, text="Start/Update", command=self.send_sine, state=tk.DISABLED)
        self.btn_sine.pack(pady=5, fill=tk.X)
        self.btn_stop_sine = ttk.Button(control_frame, text="STOP", command=self.send_stop, state=tk.DISABLED)
        self.btn_stop_sine.pack(pady=5, fill=tk.X)

        ttk.Separator(control_frame, orient='horizontal').pack(fill='x', pady=10)

        # Sawtooth analysis panel
        ttk.Label(control_frame, text="Sawtooth response (Streaming):").pack(anchor=tk.W)

        sawtooth_box = ttk.Frame(control_frame)
        sawtooth_box.pack(fill=tk.X, pady=5)

        self.sawtooth_entry = ttk.Entry(sawtooth_box, width=8)
        self.sawtooth_entry.insert(0, "3")
        self.sawtooth_entry.pack(side=tk.LEFT, padx=5)

        ttk.Label(sawtooth_box, text="Hz").pack(side=tk.LEFT)

        self.btn_sawtooth = ttk.Button(control_frame, text="Start/Update", command=self.send_sawtooth, state=tk.DISABLED)
        self.btn_sawtooth.pack(pady=5, fill=tk.X)
        self.btn_stop_sawtooth = ttk.Button(control_frame, text="STOP", command=self.send_stop, state=tk.DISABLED)
        self.btn_stop_sawtooth.pack(pady=5, fill=tk.X)

        ttk.Separator(control_frame, orient='horizontal').pack(fill='x', pady=10)

        # Save data panel
        ttk.Label(control_frame, text="Record data").pack(anchor=tk.W)
        btns_record_row = ttk.Frame(control_frame)
        btns_record_row.pack(fill=tk.X, pady=5)
        self.btn_record = ttk.Button(btns_record_row, text="Record", command=self.start_record, state=tk.DISABLED)
        self.btn_record.pack(side=tk.LEFT, expand=True, fill=tk.X)
        self.btn_record_stop = ttk.Button(btns_record_row, text="STOP", command=self.stop_record, state=tk.DISABLED)
        self.btn_record_stop.pack(side=tk.LEFT, expand=True, fill=tk.X)
        
        # Status Label
        self.lbl_status = ttk.Label(control_frame, text="Disconnected", foreground="red", wraplength=150)
        self.lbl_status.pack(side=tk.BOTTOM, pady=20)

        # Plot area initialization
        self.fig, self.ax = plt.subplots(figsize=(8, 6), dpi=100)
        self.ax.set_facecolor('black')
        self.ax.grid(True, color='#333333')
        
        # Initialize line
        self.line, = self.ax.plot([], [], color='#00ff00', lw=1.5)
        
        self.ax.set_ylim(-0.1, 3.5)
        self.ax.set_xlim(0, self.max_points)
        self.ax.set_title("Waiting for data...")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    def refresh_ports(self):
        # Save for available COM ports
        ports = [port.device for port in serial.tools.list_ports.comports()]

        # Show COM ports in the combobox
        self.port_combo['values'] = ports

        # If some ports are available, automatically show the first in the box
        if ports: self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_running:
            try:
                # Initialize serial
                self.ser = serial.Serial(self.port_combo.get(), BAUD_RATE, timeout=0.1)
                self.ser.reset_input_buffer()
                self.is_running = True
                self.stop_thread = False

                # Create and start serial listener thread                
                self.thread = threading.Thread(target=self.serial_listener, daemon=True)
                self.thread.start()

                self.btn_connect.config(text="Disconnect")
                self.lbl_status.config(text="Connected (Listening)", foreground="green")

                # Enable control buttons
                self.btn_step.config(state=tk.NORMAL)
                self.btn_sine.config(state=tk.NORMAL)
                self.btn_sawtooth.config(state=tk.NORMAL)
                self.btn_record.config(state=tk.NORMAL)
            except Exception as e:
                messagebox.showerror("Error", str(e))
        else:
            self.stop_thread = True
            if self.ser: 
                try:
                    self.ser.close()
                except: pass
            self.is_running = False
            self.btn_connect.config(text="Connect")
            self.lbl_status.config(text="Disconnected", foreground="red")

            # Disable control buttons
            self.btn_step.config(state=tk.DISABLED)
            self.btn_sine.config(state=tk.DISABLED)
            self.btn_sawtooth.config(state=tk.DISABLED)
            self.btn_stop_sine.config(state=tk.DISABLED)
            self.btn_stop_sawtooth.config(state=tk.DISABLED)
            self.btn_record.config(state=tk.DISABLED)
        

    # --- STEP RESPONSE BUTTON CALLBACK --- 
    def send_step(self):
        if self.ser and self.ser.is_open:

            # Clean data buffer and visualization trace
            self.data_buffer.clear()
            self.line.set_data([], [])

            # Set appropriate window size
            self.ax.set_xlim(0, CHUNK_SAMPLES) 
            self.canvas.draw()

            self.sample_time = 0.0001
            self.ser.write(b'S\n')
            self.lbl_status.config(text="Step response", foreground="blue")

    # --- ARMONIC ANLYSIS BUTTON CALLBACK ---
    def send_sine(self):
        if self.ser and self.ser.is_open:
            try:
                # Clean data buffer and visualization trace
                self.data_buffer.clear()
                self.line.set_data([], [])
                
                # Set appropriate window size
                self.ax.set_xlim(0, self.max_points) 
                self.canvas.draw()

                # Read frequency entry
                freq = int(self.sine_entry.get())

                # Enable stop button
                self.btn_stop_sine.config(state=tk.NORMAL)

                # Disable step and sawtooth button to avoid conflict
                self.btn_step.config(state=tk.DISABLED)  
                self.btn_sawtooth.config(state=tk.DISABLED)

                # Compute ADC sample time (useful for data log)
                self.sample_time = 1/(freq*N_LUT)

                # Send command to MCU
                cmd = f"F{freq}\n"
                self.ser.write(cmd.encode())
                self.lbl_status.config(text=f"Streaming Sine @ {freq}Hz", foreground="green")
            except ValueError:
                messagebox.showerror("Error", "Illegal frequency")

    # --- SAWTOOTH ANLYSIS BUTTON CALLBACK ---
    def send_sawtooth(self):
        if self.ser and self.ser.is_open:
            try:
                # Clean data buffer and visualization trace
                self.data_buffer.clear()
                self.line.set_data([], [])

                # Set appropriate window size
                self.ax.set_xlim(0, self.max_points) 
                self.canvas.draw()
                
                # Read frequency entry
                freq = int(self.sawtooth_entry.get())
                
                # Enable stop button
                self.btn_stop_sawtooth.config(state=tk.NORMAL)

                # Disable step and sine button to avoid conflict
                self.btn_step.config(state=tk.DISABLED)  
                self.btn_sine.config(state=tk.DISABLED)

                # Compute ADC sample time (useful for data log)
                self.sample_time = 1/(freq*N_LUT)

                # Send command to MCU
                cmd = f"T{freq}\n"
                self.ser.write(cmd.encode())
                self.lbl_status.config(text=f"Streaming Sawtooth @ {freq}Hz", foreground="green")
            except ValueError:
                messagebox.showerror("Error", "Illegal frequency")
    
    # --- STOP BUTTON CALLBACK ---
    def send_stop(self):
        if self.ser and self.ser.is_open:
            self.data_buffer.clear()
            self.ser.write(b'A\n')
            self.lbl_status.config(text="System stopped", foreground="orange")

            # Disable stop buttons
            self.btn_stop_sine.config(state=tk.DISABLED)
            self.btn_stop_sawtooth.config(state=tk.DISABLED)

            # Enable buttons back
            self.btn_step.config(state=tk.NORMAL)
            self.btn_sine.config(state=tk.NORMAL)
            self.btn_sawtooth.config(state=tk.NORMAL)

    # --- START RECORD CALLBACK ---
    def start_record(self):
        if not self.is_recording:
            try:
                # Create and sequential filename
                filename = f"Datalog_{self.record_cnt}.csv"
                self.record_file = open(filename, 'w')

                # Write header
                self.record_file.write("Time[s],Voltage[V]\n")

                # Reset sample index and time
                self.sample_idx = 0
                self.record_time = 0 

                # Set recording flag and handle buttons
                self.is_recording = True
                self.btn_record.config(state=tk.DISABLED)
                self.btn_record_stop.config(state=tk.NORMAL)
            except Exception as e:
                messagebox.showerror("File error", str(e))
    
    # --- STOP RECORD CALLBACK ---
    def stop_record(self):
        if self.is_recording:
            # Reset recording flag
            self.is_recording = False

            # If is there an open file, close it
            if self.record_file:
                self.record_file.close()
                self.record_file = None

            # Increment sequential naming counter 
            self.record_cnt += 1

            # Handle buttons
            self.btn_record.config(state=tk.NORMAL)
            self.btn_record_stop.config(state=tk.DISABLED)
            
    # --- DATA RECEPTION THREAD ---
    def serial_listener(self):
        while not self.stop_thread and self.ser and self.ser.is_open:
            try:
                # Search for the protocol header
                if self.ser.read(1) == b'\xAA':
                    if self.ser.read(1) == b'\xBB':
                        mode_byte = self.ser.read(1)
                        if not mode_byte: continue
                        mode = ord(mode_byte)
                        
                        # Each sample is a uint16_t (two bytes) and the samples are equal to CHUNK_SAMPLES
                        n_bytes = CHUNK_SAMPLES * 2 
                        
                        # Read the raw bytes from the UART
                        raw_data = self.ser.read(n_bytes)
                        
                        if len(raw_data) == n_bytes:
                            values = struct.unpack(f'<{CHUNK_SAMPLES}H', raw_data)

                            # Record data
                            if self.is_recording and self.record_file:
                                try:    
                                    lines = []
                                    for val in values:
                                        volt = val * (3.3/4095.0)
                                        lines.append(f"{self.record_time},{volt:.4f}") 
                                        self.sample_idx += 1
                                        self.record_time += self.sample_time

                                    # One shot write a whole chunk of data    
                                    self.record_file.write("\n".join(lines) + "\n")
                                except Exception as e:
                                    print(f"File error: {e}")

                            # If MODE_STEP we must clear the buffer, since step is one shot and not streaming
                            if mode == MODE_STEP:
                                self.data_buffer.clear()
                                self.data_buffer.extend(values)
                            elif mode == MODE_FREQ:
                                self.data_buffer.extend(values)
            except Exception:
                pass

    # --- WRAPPER FUNCTION TO HANDLE THE ANIMATION ---
    def animate_loop(self):
        # Skip if the app is closing
        if self.is_closing:
            return

        # Update plot
        self.update_plot()

        # Self launch again in 50 ms
        self.animate_future = self.root.after(50, self.animate_loop)

    # --- WRAPPER FUNCTION TO PLOT THE GRAPH ---
    def update_plot(self):
        if self.is_running and len(self.data_buffer) > 0:
            try:
                data = np.array(self.data_buffer)
                
                # Convert ADC raw values to Volts
                volt = data * (3.3 / 4095.0)
                x = np.arange(len(volt))
                
                self.line.set_data(x, volt)
                self.canvas.draw()
            except Exception:
                pass

    # --- CLOSE WINDOW CALLBACK ---
    def on_closing(self):
        # Send stop command 
        self.send_stop()

        # Delete animate_loop timer callback if any pendent
        if self.animate_future:
            self.root.after_cancel(self.animate_future)

        # Set the flag to disable the graphic loop
        self.is_closing = True  
        
        # Set the flag to stop threading
        self.stop_thread = True 

        # Close the file if we were recording
        if self.is_recording and self.record_file:
            self.record_file.close()
        
        # Close serial
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except: pass
            
        # Destroy gui
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = RealTimeScope(root)
    root.mainloop()