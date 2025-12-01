import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


# Variables
FILE_NAME = 'adc_data.csv'
VOLT_REF = 3.3
RES_REF = 10000
ADC_TO_VOLT = VOLT_REF/(2**12)
SAMPLE_FREQ = 10000
SAMPLE_TIME = 1 / SAMPLE_FREQ
N_SAMPLES = 2000

# Import data from file
data_table = pd.read_csv(FILE_NAME)

# Create volt column
data_table['Volt'] = data_table['ADC_raw'] * ADC_TO_VOLT

# Compute final value
VOLT_FIN = data_table['Volt'].tail(5).mean()

# Create time column
t = 0
time = []
for i in range(len(data_table)):
    t += SAMPLE_TIME
    time.append(t)
data_table['Time'] = time

# Linearize Volt, filtering out the noisy initial and final parts of the curve
volt_log = np.log(VOLT_FIN - data_table['Volt'][int(N_SAMPLES*0.1):int(N_SAMPLES*0.75)])
time = data_table['Time'][int(N_SAMPLES*0.1):int(N_SAMPLES*0.75)]

# Fitting a line x = volt_log, y = time 
m, q = np.polyfit(time, volt_log,  1)
fitted_volt_log = m*time + q

# Plot results
plt.plot(time, volt_log, 'b-o', alpha=0.5, label='Linearized circuit voltage')
plt.plot(time, fitted_volt_log, color='red', label='Regression')
plt.xlabel('Time[s]')
plt.ylabel('Volt[V]')
plt.legend()
plt.show()

# Compute circuit time constant
tau = -1/m

# Estimate circuit Resistance
Rx = VOLT_FIN * RES_REF / (VOLT_REF - VOLT_FIN)

# Estimate circuit Capacity
Cx = tau * (1/Rx + 1/RES_REF)

print(f'The circuit resistance is: {Rx} Ohm')
print(f'The circuit capacity is: {Cx} Farad')
print(f'The circuit time constant is: {tau} s')



    
