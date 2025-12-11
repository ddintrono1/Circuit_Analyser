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

# --- CONFIGURAZIONE ---
BAUD_RATE = 921600 
CHUNK_SAMPLES = 2000 

# Costanti Protocollo
HEADER_SYNC = b'\xAA\xBB'
MODE_STEP = 1
MODE_FREQ = 2

class RealTimeScope:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Real-Time Oscilloscope")
        self.root.geometry("1200x800")
        
        # Gestione chiusura pulita
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.is_closing = False

        # --- DATI ---
        self.ser = None
        self.is_running = False
        self.stop_thread = False
        
        # Buffer Circolare
        self.max_points = 10000
        self.data_buffer = collections.deque([0]*self.max_points, maxlen=self.max_points)
        
        self.current_mode = "IDLE"
        self.freq_target = 100

        self._setup_ui()
        
        # Avvia il timer grafico
        self.animate_loop()

    def _setup_ui(self):
        # Pannello Sinistro (Controlli)
        control_frame = ttk.LabelFrame(self.root, text="Controlli", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        # Selezione Porta
        ttk.Label(control_frame, text="Porta COM:").pack(anchor=tk.W)
        self.port_combo = ttk.Combobox(control_frame, width=15)
        self.port_combo.pack(pady=5)
        self.refresh_ports()
        ttk.Button(control_frame, text="Aggiorna Porte", command=self.refresh_ports).pack(pady=5)

        # Connessione
        self.btn_connect = ttk.Button(control_frame, text="Connetti", command=self.toggle_connection)
        self.btn_connect.pack(pady=20, fill=tk.X)

        ttk.Separator(control_frame, orient='horizontal').pack(fill='x', pady=10)

        # STEP RESPONSE
        ttk.Label(control_frame, text="Step Response (One-Shot):").pack(anchor=tk.W)
        self.btn_step = ttk.Button(control_frame, text="Spara Gradino", command=self.send_step, state=tk.DISABLED)
        self.btn_step.pack(pady=5, fill=tk.X)

        ttk.Separator(control_frame, orient='horizontal').pack(fill='x', pady=10)

        # FREQUENCY ANALYSIS
        ttk.Label(control_frame, text="Generatore (Streaming):").pack(anchor=tk.W)
        
        freq_box = ttk.Frame(control_frame)
        freq_box.pack(fill=tk.X, pady=5)
        self.freq_entry = ttk.Entry(freq_box, width=8)
        self.freq_entry.insert(0, "2")
        self.freq_entry.pack(side=tk.LEFT, padx=5)
        ttk.Label(freq_box, text="Hz").pack(side=tk.LEFT)

        self.btn_freq = ttk.Button(control_frame, text="Avvia/Aggiorna Streaming", command=self.send_freq, state=tk.DISABLED)
        self.btn_freq.pack(pady=5, fill=tk.X)
        
        self.btn_stop = ttk.Button(control_frame, text="STOP", command=self.send_stop, state=tk.DISABLED)
        self.btn_stop.pack(pady=5, fill=tk.X)

        # Status Label
        self.lbl_status = ttk.Label(control_frame, text="Disconnesso", foreground="red", wraplength=150)
        self.lbl_status.pack(side=tk.BOTTOM, pady=20)

        # Area Grafico
        self.fig, self.ax = plt.subplots(figsize=(8, 6), dpi=100)
        self.ax.set_facecolor('black')
        self.ax.grid(True, color='#333333')
        
        # Linea del grafico
        self.line, = self.ax.plot([], [], color='#00ff00', lw=1.5)
        
        self.ax.set_ylim(-0.1, 3.5)
        self.ax.set_xlim(0, self.max_points)
        self.ax.set_title("Waiting for data...")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports: self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_running:
            try:
                self.ser = serial.Serial(self.port_combo.get(), BAUD_RATE, timeout=0.1)
                self.ser.reset_input_buffer()
                self.is_running = True
                self.stop_thread = False
                
                self.thread = threading.Thread(target=self.serial_listener, daemon=True)
                self.thread.start()

                self.btn_connect.config(text="Disconnetti")
                self.lbl_status.config(text="Connesso (Listening)", foreground="green")
                self.enable_controls(True)
            except Exception as e:
                messagebox.showerror("Errore", str(e))
        else:
            self.stop_thread = True
            if self.ser: 
                try:
                    self.ser.close()
                except: pass
            self.is_running = False
            self.btn_connect.config(text="Connetti")
            self.lbl_status.config(text="Disconnesso", foreground="red")
            self.enable_controls(False)

    def enable_controls(self, enable):
        state = tk.NORMAL if enable else tk.DISABLED
        self.btn_step.config(state=state)
        self.btn_freq.config(state=state)
        self.btn_stop.config(state=state)

    def send_step(self):
        if self.ser and self.ser.is_open:
            self.current_mode = "STEP"
            self.data_buffer.clear()
            self.ser.write(b'S\n')
            self.lbl_status.config(text="Richiesta Step...", foreground="blue")

    def send_freq(self):
        if self.ser and self.ser.is_open:
            try:
                freq = int(self.freq_entry.get())
                self.freq_target = freq
                self.current_mode = "FREQ"
                self.data_buffer.clear()
                cmd = f"F{freq}\n"
                self.ser.write(cmd.encode())
                self.lbl_status.config(text=f"Streaming @ {freq}Hz", foreground="green")
            except ValueError:
                messagebox.showerror("Errore", "Frequenza non valida")
                
    def send_stop(self):
        if self.ser and self.ser.is_open:
            self.current_mode = "IDLE"
            self.data_buffer.clear()
            self.ser.write(b'A\n')
            self.lbl_status.config(text="System stopped", foreground="orange")
            
    # --- THREAD DI RICEZIONE ---
    def serial_listener(self):
        while not self.stop_thread and self.ser and self.ser.is_open:
            try:
                if self.ser.read(1) == b'\xAA':
                    if self.ser.read(1) == b'\xBB':
                        mode_byte = self.ser.read(1)
                        if not mode_byte: continue
                        mode = ord(mode_byte)
                        
                        n_bytes = CHUNK_SAMPLES * 2 
                        raw_data = self.ser.read(n_bytes)
                        
                        if len(raw_data) == n_bytes:
                            values = struct.unpack(f'<{CHUNK_SAMPLES}H', raw_data)
                            
                            if mode == MODE_STEP:
                                self.data_buffer.clear()
                                self.data_buffer.extend(values)
                            elif mode == MODE_FREQ:
                                self.data_buffer.extend(values)
            except Exception:
                pass

    # --- LOOP GRAFICO SICURO ---
    def animate_loop(self):
        """Wrapper per gestire il loop grafico con controllo di chiusura"""
        # 1. Se stiamo chiudendo, STOP! Non ripianificare.
        if self.is_closing:
            return

        # 2. Aggiorna il grafico
        self.update_plot()

        # 3. Ripianifica tra 50ms
        self.root.after(50, self.animate_loop)

    def update_plot(self):
        if self.is_running and len(self.data_buffer) > 0:
            try:
                data = np.array(self.data_buffer)
                volt = data * (3.3 / 4095.0)
                x = np.arange(len(volt))
                
                self.line.set_data(x, volt)
                self.ax.set_xlim(0, max(len(volt), 100)) # Evita crash se len=0
                self.canvas.draw()
            except Exception:
                pass

    # --- GESTORE CHIUSURA FINESTRA ---
    def on_closing(self):
        """Chiamata quando clicchi la X della finestra"""
        self.is_closing = True # Blocca il loop grafico
        self.stop_thread = True # Blocca il thread seriale
        
        # Chiudi la seriale se aperta
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except: pass
            
        # Distruggi la finestra
        self.root.destroy()
        print("Applicazione chiusa correttamente.")

if __name__ == "__main__":
    root = tk.Tk()
    app = RealTimeScope(root)
    root.mainloop()