# Interface Tracking Experiment

Automated system for tracking a growing interface using computer vision and closed-loop motor control.

The system maintains the interface in the center of the camera field while logging temperature and growth rate.

Designed for long-duration crystal growth or directional solidification experiments.

---

## Hardware

Controller

- Raspberry Pi 400

Motion control

- Arduino Uno
- DRV8825 stepper driver
- Linear motion stage

Sensors

- Adafruit MAX31865 with PT100

Imaging

- Raspberry Pi Camera Module

Cooling

- Peltier element with PWM control

---

## Features

Interface detection

- Sobel gradient detection
- sub-pixel edge estimation
- Kalman filtering

Motor control

- predictive camera centering
- automatic recovery when interface disappears

Temperature control

- PID controlled cooling

Logging

- CSV logging every second
- periodic disk flushing

Visualization

- live plots
- annotated camera stream

---

## Installation

Clone repository

`git clone https://github.com/YOURNAME/interface-tracking-experiment.git`

Install dependencies

`pip install -r requirements.txt`

---

## Running the experiment

`python python/interface_tracking_system.py`

---

## Data output

CSV log fields

- timestamp
- temperature_C
- interface_mm
- filtered_interface_mm
- growth_rate_mm_min
- motor_position_mm
- pwm_duty

---

## Data analysis

Growth-rate analysis notebook is located in `notebooks/growth_rate_analysis.ipynb`

---

## License

MIT 