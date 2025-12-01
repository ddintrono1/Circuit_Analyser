import pandas as pd
import matplotlib.pyplot as plt

FILE_NAME = 'adc_freq_data.csv'
VOLT_REF = 3.3
ADC_TO_VOLT = VOLT_REF/(2**12)
SAMPLE_TIME = 0.0001


data_table = pd.read_csv(FILE_NAME)

data_table['Volt'] = data_table['ADC_raw'] * ADC_TO_VOLT

# Create time column
t = 0
time = []
for i in range(len(data_table)):
    t += SAMPLE_TIME
    time.append(t)
data_table['Time'] = time

plt.plot(data_table['Time'], data_table['Volt'])
plt.show()
