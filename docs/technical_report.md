# Technical Report

## Automated Interface Tracking and Stage Control System

### 1. Introduction

This document describes the methodology and control architecture of an automated interface-tracking system used for crystal growth or directional solidification experiments. The system continuously observes a moving phase boundary using a camera and automatically moves a linear stage so that the interface remains centered in the image.

The system is implemented on a **Raspberry Pi 400** controlling image processing, data logging, and temperature regulation. A stepper motor stage driven by an **Arduino Uno** and a **DRV8825 stepper driver** adjusts the camera position. Temperature measurements are obtained via a **Adafruit MAX31865 RTD interface**, while imaging is performed with a **Raspberry Pi Camera Module**.

The goal of the system is to:

- Track the interface position with sub-pixel accuracy
- Estimate growth velocity in real time
- Move the stage smoothly to keep the interface centered
- Record synchronized experimental data

---

# 2. System Architecture

The system contains four main subsystems:

1. Imaging subsystem
2. Interface detection algorithm
3. Stage control system
4. Temperature control system

Data flow:

Camera → Image Processing → Interface Position → Kalman Filter → Stage Controller → Stepper Motor

---

# 3. Imaging and Interface Detection

## 3.1 Image Acquisition

Images are captured every ( $\Delta t = 2 s$ ).

Each frame is represented as an intensity matrix:

$$
I(x,y)
$$

where:

* (x) is horizontal pixel coordinate
* (y) is vertical pixel coordinate

The interface appears as a strong intensity gradient because two phases have different optical properties.

---

# 4. Gradient-Based Edge Detection

To detect the interface, a Sobel operator is applied in the vertical direction.

$$
G_y(x,y) =
\begin{bmatrix}
-1 & -2 & -1 \\
0 & 0 & 0 \\
1 & 2 & 1
\end{bmatrix}
\cdot I(x,y)
$$

where ( * ) denotes convolution.

The magnitude of the gradient is:

$$
|G_y(x,y)|
$$

This highlights horizontal edges such as the phase boundary.

---

# 5. Row Averaging

The interface is assumed to be approximately horizontal.

Therefore the gradient is averaged across each row:

$$
S(y) = \frac{1}{N_x} \sum_{x=1}^{N_x} |G_y(x,y)|
$$

where:

* (S(y)) is the edge strength for row (y)
* (N_x) is image width.

The interface location corresponds to the maximum gradient:

$$
y_0 = \arg\max_y S(y)
$$

---

# 6. Sub-Pixel Interface Detection

The raw detection produces integer pixel resolution.

To improve precision, a quadratic interpolation is performed around the maximum.

Let:

$$
g_{-1} = S(y_0-1)
$$
$$
g_0 = S(y_0)
$$
$$
g_{+1} = S(y_0+1)
$$

The sub-pixel correction is:

$$
\delta =
\frac{g_{-1} - g_{+1}}
{2(g_{-1} - 2g_0 + g_{+1})}
$$

The refined interface position is:

$$
y_{sub} = y_0 + \delta
$$

Typical precision achieved:

≈ 0.1 pixel.

---

# 7. Conversion to Physical Position

Pixel position is converted to millimetres using calibration.

$$
h = y_{sub} \cdot c
$$

where

(c) = mm per pixel.

Example:

$$
c = 0.02 , mm/pixel
$$

---

# 8. Kalman Filtering

Image measurements contain noise.

The interface dynamics are modelled using a state vector:

$$
x =
\begin{bmatrix}
h \
v
\end{bmatrix}
$$

where

* (h) = interface height
* (v) = interface velocity.

State propagation:

$$
x_{k+1} =
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
x_k
$$

Measurement equation:

$$
z_k =
\begin{bmatrix}
1 & 0
\end{bmatrix}
x_k
$$

The Kalman filter performs:

Prediction:

$$
x^- = F x
$$

Update:

$$
K = P H^T (H P H^T + R)^{-1}
$$

$$
x = x^- + K(z - Hx^-)
$$

This produces smoothed estimates of:

- interface position
- growth velocity

---

# 9. Growth Rate Estimation

The instantaneous growth velocity is obtained directly from the Kalman state:

$$
v = \frac{dh}{dt}
$$

Growth rate in mm/min:

$$
v_{mm/min} = v \cdot 60
$$

---

# 10. Predictive Interface Tracking

To avoid stage oscillation, the system predicts where the interface will be at the next frame.

$$
h_{future} = h + v \Delta t
$$

The control error becomes:

$$
e = h_{future} - h_{target}
$$

where:

($h_{target}$) is the desired interface position in the image.

---

# 11. Stage Motion Control

The camera is mounted on a vertical stage driven by a stepper motor.

Motor displacement is related to steps:

$$
d = \frac{s}{N}
$$

where

* (s) = number of steps
* (N) = steps per mm.

Control law:

$$
s = k \cdot e \cdot N
$$

where (k) is a gain parameter (typically 0.5).

This ensures smooth motion without overshoot.

---

# 12. Stepper Motor Execution

Commands are sent from the Raspberry Pi to the Arduino via serial:

MOVE steps speed

The Arduino generates STEP pulses for the DRV8825 driver.

Pulse frequency:

$$
f = \frac{speed}{2}
$$

Motor position is tracked internally:

$$
p_{k+1} = p_k + s
$$

---

# 13. Interface Loss Detection

If the gradient confidence drops below threshold:

$$
\frac{S(y_0)}{\bar S} < C_{min}
$$

the interface is considered lost.

Recovery procedure:

1. hold predicted position briefly
2. if still lost → perform search motion

Search motion:

small step increments:

$$
\pm 0.1  mm
$$

until interface reappears.

---

# 14. Temperature Control

Temperature is measured using the MAX31865.

A PID controller regulates cooling power.

Control law:

$$
u(t) = K_p e + K_i \int e,dt + K_d \frac{de}{dt}
$$

where

$$
e = T_{set} - T
$$

Output (u(t)) controls PWM duty cycle applied to the Peltier element.

---

# 15. Data Logging

The system logs experimental parameters every second.

Logged variables include:

- timestamp
- temperature
- interface position
- filtered interface position
- growth rate
- motor position
- PWM duty cycle

Disk writes are buffered and flushed every 10 minutes to reduce SD card wear.

---

# 16. Visualization

The system provides live feedback plots of

- temperature
- interface height
- growth rate

as well as the camera image with the detected interface overlay.

---

# 17. System Performance

Typical precision:

| Parameter          | Precision    |
| ------------------ | ------------ |
| interface position | ±0.1 pixel   |
| interface height   | ±2 µm        |
| growth rate        | ±0.02 mm/min |

---

# 18. Conclusion

The presented system combines computer vision, predictive control, and motorized positioning to maintain a phase boundary within the camera field during crystal growth experiments.

Key advantages include:

- sub-pixel interface detection
- predictive stage motion
- real-time growth analysis
- autonomous recovery from detection loss

The architecture is suitable for long-duration experiments where accurate measurement of interface dynamics is required.

---
