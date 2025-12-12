import pandas as pd
import matplotlib.pyplot as plt

file_path = "Datalog_0.csv"

def plot_data():

    df = pd.read_csv(file_path)

    plt.figure(figsize=(10, 6))
    
    plt.plot(df['Time[s]'], df['Voltage[V]'], label='Time after time', linewidth=1)


    plt.title(f"Waveform analysis")
    plt.xlabel("Time [s]")
    plt.ylabel("Waveform [V]")
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.legend()
    
    # Auto-scale axis
    plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    plot_data()