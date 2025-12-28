# To do: 
* The waveforms never stop, except if aborted. Consider stopping when launching the step, separating the gui / microcontroller app
* Consider using just one ADC, for both step and waveforms, to decrease costs and MCU demand


# Development of a Low-Cost STM32-Based Embedded System for Signal Generation and Acquisition

This repository contains all the materials regarding the design, implementation, and testing of the embedded system. Below is a guide to the project structure and files.

## 📂 Repository Contents

* **Firmware \& Configuration**: All source code and MCU configuration files required to run the project on the STM32 board.
* **`gui\_app.py`**: The host-side Python script that launches the Graphical User Interface for controlling the oscilloscope and signal generator.
* **`Project\_report.pdf`**: A comprehensive report detailing the system architecture, design choices, and experimental results.
* **`Circuit scheme.pdf`**: A kicad scheme of the hardware configuration
* **Experimental Data**:

  * `sawtooth.csv`
  * `sine.csv`
  * `step.csv`

   *Note: These files contain raw data collected during the system validation tests, **which are already displayed in the report**.*

* **`plot\_csv.py`**: A utility script to visualize the collected data.

## 📊 How to use the Plotting Script	

To visualize the experimental results using `plot\_csv.py`:

1. Open the script in your text editor.
2. Locate the filename variable and change it to the desired CSV file (e.g., `'sine.csv'`).
3. Run the script to generate the plot.
