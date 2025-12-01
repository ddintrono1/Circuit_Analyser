import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

# --- CONFIGURAZIONE ---
BAUD_RATE = 115200

class CircuitAnalyzerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Circuit Analyzer Dashboard")
        self.root.geometry("1000x700")

        self.ser = None
        self.is_connected = False
        self.rx_buffer = []
        self.is_acquiring = False
        
        # Variabili per i dati
        self.x_data = []
        self.y_data = []

        self._setup_ui()

    def _setup_ui(self):
        # 1. Pannello di Controllo (Sinistra)
        control_frame = ttk.LabelFrame(self.root, text="Controls", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        # Selezione Porta COM
        ttk.Label(control_frame, text="COM Port:").pack(anchor=tk.W)
        self.port_combo = ttk.Combobox(control_frame, width=15)
        self.port_combo.pack(pady=5)
        self.refresh_ports()
        
        btn_refresh = ttk.Button(control_frame, text="Update Ports", command=self.refresh_ports)
        btn_refresh.pack(pady=5)

        # Pulsante Connetti
        self.btn_connect = ttk.Button(control_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(pady=20)

        # Sezione Comandi
        ttk.Separator(control_frame, orient='horizontal').pack(fill='x', pady=10)
        ttk.Label(control_frame, text="Measurements:").pack(anchor=tk.W)

        self.btn_step = ttk.Button(control_frame, text="Step Response", command=self.send_step_cmd, state=tk.DISABLED)
        self.btn_step.pack(pady=10, fill=tk.X)

        self.btn_freq = ttk.Button(control_frame, text="Frequency Analysis", command=self.send_freq_cmd, state=tk.DISABLED)
        self.btn_freq.pack(pady=10, fill=tk.X)

        # Status
        self.lbl_status = ttk.Label(control_frame, text="Status: Disconnected", foreground="red")
        self.lbl_status.pack(side=tk.BOTTOM, pady=10)

        # 2. Pannello Grafico (Destra)
        plot_frame = ttk.Frame(self.root)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.fig, self.ax = plt.subplots(figsize=(8, 6), dpi=100)
        self.ax.set_title("Waiting for data...")
        self.ax.grid(True)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Voltage (V)")

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            try:
                port = self.port_combo.get()
                self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
                self.is_connected = True
                self.btn_connect.config(text="Disconnect")
                self.lbl_status.config(text="Status: Connected", foreground="green")
                self.btn_step.config(state=tk.NORMAL)
                self.btn_freq.config(state=tk.NORMAL)
                
                # Avvia thread di ascolto
                self.listen_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
                self.listen_thread.start()
                
            except Exception as e:
                messagebox.showerror("Error", f"Unable to connect: {e}")
        else:
            if self.ser:
                self.ser.close()
            self.is_connected = False
            self.btn_connect.config(text="Connect")
            self.lbl_status.config(text="Status: Disconnected", foreground="red")
            self.btn_step.config(state=tk.DISABLED)
            self.btn_freq.config(state=tk.DISABLED)

    def send_step_cmd(self):
        if self.ser and self.ser.is_open:
            self.prepare_acquisition("Step Response Acquisition...")
            self.ser.write(b'S') # Manda 'S' all'STM32

    def send_freq_cmd(self):
        if self.ser and self.ser.is_open:
            self.prepare_acquisition("Frequency response...")
            self.ser.write(b'F') 

    def prepare_acquisition(self, status_msg):
        self.is_acquiring = True
        self.rx_buffer = []
        self.lbl_status.config(text=status_msg, foreground="blue")
        self.btn_step.config(state=tk.DISABLED)
        self.btn_freq.config(state=tk.DISABLED)

    def read_serial_loop(self):
        """Thread separato che ascolta la seriale senza bloccare la GUI"""
        current_mode = "UNKNOWN"
        
        while self.is_connected:
            if self.ser and self.ser.is_open:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue

                    # Parsing Header Modalità
                    if "MODE: STEP" in line:
                        current_mode = "STEP"
                        self.rx_buffer = []
                    elif "MODE: FREQ" in line:
                        current_mode = "FREQ"
                        self.rx_buffer = []
                    
                    # Parsing Fine Trasmissione
                    elif "COMPLETED" in line:
                        self.is_acquiring = False
                        # Chiamiamo l'aggiornamento GUI dal thread principale
                        self.root.after(0, self.update_plot, current_mode)
                        
                    # Parsing Dati
                    elif "," in line:
                        try:
                            parts = line.split(',')
                            if len(parts) == 2:
                                # Salviamo raw data: Index, ADC
                                self.rx_buffer.append((int(parts[0]), int(parts[1])))
                        except ValueError:
                            pass
                            
                except Exception as e:
                    print(f"Serial error: {e}")
                    break

    def update_plot(self, mode):
        # Riabilita i pulsanti
        self.lbl_status.config(text="Status: Connected (Received data)", foreground="green")
        self.btn_step.config(state=tk.NORMAL)
        self.btn_freq.config(state=tk.NORMAL)

        if not self.rx_buffer:
            return

        # Elaborazione Dati
        raw_data = np.array(self.rx_buffer)
        indices = raw_data[:, 0]
        adc_values = raw_data[:, 1]
        
        # Conversione in Volt
        voltage = adc_values * (3.3 / 4095.0)
        
        # Conversione Tempo (Stimata 10kHz)
        time = indices * (1/10000.0)

        # Plotting
        self.ax.clear()
        
        if mode == "STEP":
            self.ax.plot(time, voltage, color='blue', label='Step Response')
            self.ax.set_title("Step response (Time Domain)")
            
            # Bonus: Calcola valore finale
            v_fin = np.mean(voltage[-50:])
            self.ax.axhline(v_fin, color='red', linestyle='--', alpha=0.5, label=f'V_fin={v_fin:.2f}V')
            
        elif mode == "FREQ":
            # Tagliamo il transitorio (es. primi 500 punti) per visualizzazione pulita
            cut_idx = 500
            if len(time) > cut_idx:
                self.ax.plot(time[cut_idx:], voltage[cut_idx:], color='green', label='Steady State')
                self.ax.plot(time[:cut_idx], voltage[:cut_idx], color='gray', alpha=0.3, label='Transient')
            else:
                self.ax.plot(time, voltage, color='green')
                
            self.ax.set_title("Frequency analysis (Sinusoid)")

        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Voltage (V)")
        self.ax.legend()
        self.ax.grid(True)
        self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = CircuitAnalyzerApp(root)
    root.mainloop()