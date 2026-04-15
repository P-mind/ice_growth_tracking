"""
The system will start

- temperature control
- interface detection
- motor tracking
- logging
- live visualization

Press `CTRL+C` to safely stop the experiment.

---

## Data Output

The experiment log contains

| column | description |
|------|-------------|
timestamp | system time |
temperature_C | measured temperature |
interface_mm | detected interface |
filtered_interface_mm | Kalman filtered position |
growth_rate_mm_min | estimated growth velocity |
motor_position_mm | camera stage position |
pwm_duty | cooling power |

---

## Safety Notes

Before running experiments:

- set the DRV8825 current limit
- verify stepper motion limits
- confirm Peltier cooling polarity

---

## Future Improvements

Possible extensions

- automatic exposure control
- template matching interface detection
- multi-camera systems
- network experiment monitoring

---

## License

MIT License
"""
import cv2
import numpy as np
import time
import threading
import csv
import os
import serial
from collections import deque
from datetime import datetime

import board
import digitalio
import busio
import pwmio

import adafruit_max31865

import matplotlib.pyplot as plt

import argparse

import sys

########################################
# USER CONFIGURATION
########################################

CAMERA_INDEX = 0

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

CAPTURE_INTERVAL = 0.5 #1.0
LOG_INTERVAL = 1.0
FLUSH_INTERVAL = 20
DISPLAY_INTERVAL = 0.5
PLOT_UPDATE_INTERVAL = 1.0

TARGET_INTERFACE_MM = None  # None keeps the detected interface centered in the frame
INTERFACE_CONF_THRESHOLD = 5.0  # Pixel
INTERFACE_SEARCH_RANGE_PX = 50  # Restrict detection to a central horizontal band

MAX_TRAVEL_MM = 80      # Prevent motor from moving too far in case of tracking loss

MOTOR2_SPEED_INIT = 200      # Initial speed for the secondary motor in steps/sec
MOTOR2_SPEED_STEP = 1000   # Live adjustment step for keyboard control

# Predefine minimum, maximum, and default speeds for motor 2 to ensure safe operation
MOTOR2_SPEED_MAX = 20000
MOTOR2_SPEED_MIN = 2200  
MOTOR2_SPEED_AVG = 5000

MOTOR1_MANUAL_STEP_MM = 0.25   # Manual nudge per keypress for motor 1
MOTOR1_MANUAL_SPEED = 400     # Speed for manual motor 1 nudges in steps/sec
MOTOR1_TRACK_SPEED = 120      # Slower speed for automatic tracking moves in steps/sec
MOTOR1_TRACK_GAIN = 0.2       # Position correction gain for automatic tracking
MOTOR1_LOST_GAIN = 0.1        # Smaller gain while running on prediction fallback
MOTOR1_MAX_STEP_PER_CYCLE = 12  # Limit tracking correction per capture cycle (steps)
MOTOR1_ACCEL_INIT = 600       # Motor 1 acceleration in steps/s^2
MOTOR1_ACCEL_STEP = 100       # Live adjustment step for motor 1 acceleration
MOTOR1_ACCEL_MIN = 50
MOTOR1_ACCEL_MAX = 5000
MANUAL_OVERRIDE_DURATION = 5.0 # Seconds to pause auto-tracking after a manual nudge

SERIAL_CMD_TIMEOUT_S = 3.0
SERIAL_MAX_RESPONSE_LINES = 12

MM_PER_PIXEL = 0.02
STEPS_PER_MM = 400

SERIAL_PORT = "/dev/ttyACM0"
TEMP_SENSOR_CS_PINS = [board.D5, board.D6, board.D12, board.D13]
NUM_TEMP_SENSORS = len(TEMP_SENSOR_CS_PINS)

MAX_LENGTH_DISPLAY = 100000

CSV_HEADER = [
                "timestamp",
                "temperature_peltier_C",
                "temperature_top_out_C",
                "temperature_bottom_middle_C",
                "temperature_bottom_out_C",
                "interface_mm",
                "filtered_interface_mm",
                "growth_rate_mm_min",
                "motor_position_mm",
                "motor2_speed_steps_s",
                "interface_source",
                "motor_status"
            ]

########################################
# GLOBAL STATE
########################################

class SystemState:

    def __init__(self):
        """
        Initializes the SystemState class, which holds global state variables for the system.

        Attributes:
            lock (threading.Lock): Lock for thread-safe access to state variables.
            temperatures (list): List of current temperature readings in Celsius (one per sensor).
            interface_mm (float or None): Raw detected interface position in millimeters.
            interface_filtered (float or None): Kalman-filtered interface position in millimeters.
            growth_rate (float or None): Estimated growth rate in mm/min.
            motor_position_mm (float): Current motor position in millimeters (default: 0).
            pwm_duty (float): Current PWM duty cycle for cooling (default: 0).
            image (numpy.ndarray or None): Current camera frame.
        """
        self.lock = threading.Lock()

        self.temperatures = [None] * NUM_TEMP_SENSORS
        self.interface_mm = None
        self.interface_filtered = None
        self.interface_fallback = False
        self.interface_source = "detected"
        self.growth_rate = None
        self.motor_position_mm = 0
        self.pwm_duty = 0
        self.motor_status = "running"  # "running", "stopped_wall", "lost", "manual", "fallback"
        self.motor2_speed = MOTOR2_SPEED_INIT
        self.manual_override_until = 0.0

        self.image = None

state = SystemState()

########################################
# ARDUINO STEPPER
########################################

class ArduinoStepper:

    def __init__(self):
        """
        Initializes the ArduinoStepper class for controlling a stepper motor via serial communication with an Arduino.

        Attributes:
            ser (serial.Serial): Serial connection to the Arduino.
            position_steps (int): Current motor position in steps.
            lock (threading.Lock): Lock for thread-safe access to position.
        """
        self.ser = serial.Serial(SERIAL_PORT,115200,timeout=2)
        time.sleep(2)

        # Clear boot-time banners/noise so command parsing starts from a clean buffer.
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.position_steps = 0
        self.lock = threading.Lock()
        self.command_lock = threading.Lock()

    def _send_command_wait_reply(self, cmd, expected_prefixes, timeout_s=SERIAL_CMD_TIMEOUT_S):
        """
        Send a single command and consume serial lines until one matches expected prefixes.

        This protects against occasional startup banners or noisy lines that can appear
        before the real protocol response.
        """
        expected_prefixes = tuple(expected_prefixes)
        deadline = time.time() + timeout_s
        unexpected_lines = []

        with self.command_lock:
            self.ser.reset_input_buffer()
            self.ser.write(cmd.encode())
            self.ser.flush()

            lines_seen = 0
            while time.time() < deadline and lines_seen < SERIAL_MAX_RESPONSE_LINES:
                raw = self.ser.readline()
                if not raw:
                    continue

                lines_seen += 1
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue

                if line.lower().endswith("ready"):
                    continue

                if any(line.startswith(prefix) for prefix in expected_prefixes):
                    return line

                unexpected_lines.append(line)

        detail = unexpected_lines[-1] if unexpected_lines else "timeout/no response"
        expected_text = " or ".join(expected_prefixes)
        raise RuntimeError(f"Expected '{expected_text}', got '{detail}'")

    def move_steps(self,steps,speed=200):
        """
        Moves the stepper motor by a specified number of steps at a given speed.

        Args:
            steps (int): Number of steps to move (positive for one direction, negative for the other).
            speed (int, optional): Speed of movement in steps per second (default: 1500).
        """
        cmd=f"MOVE {steps} {speed}\n"
        line = self._send_command_wait_reply(cmd, ("POS",))

        try:
            pos=int(line.split()[1])
        except (IndexError, ValueError) as e:
            raise RuntimeError(f"Failed to parse motor position from: {line}") from e

        with self.lock:
            self.position_steps=pos

    def move_steps2(self, steps, speed=400):
        """
        Moves the secondary motor by a specified number of steps at a given speed.

        Args:
            steps (int): Number of steps to move.
            speed (int, optional): Speed in steps per second.
        """
        cmd=f"MOVE2 {steps} {speed}\n"
        line = self._send_command_wait_reply(cmd, ("POS2",))
        if line.startswith("POS2"):
            return int(line.split()[1])
        raise RuntimeError(f"Failed to move motor2: {line}")

    def start_motor2(self, speed=400):
        """
        Starts the secondary motor continuously at the requested speed in steps/sec.
        """
        cmd=f"START2 {speed}\n"
        line = self._send_command_wait_reply(cmd, ("START2 OK",))
        if not line.startswith("START2 OK"):
            raise RuntimeError(f"Failed to start motor2: {line}")

    def set_motor2_speed(self, speed):
        """
        Updates the secondary motor speed while it is running.
        """
        self.start_motor2(speed)

    def set_motor1_accel(self, accel_steps_s2):
        """
        Updates motor 1 acceleration in steps/s^2.
        """
        accel_steps_s2 = int(accel_steps_s2)
        cmd = f"ACCEL1 {accel_steps_s2}\n"
        line = self._send_command_wait_reply(cmd, ("ACCEL1 OK",))
        if not line.startswith("ACCEL1 OK"):
            raise RuntimeError(f"Failed to set motor1 acceleration: {line}")

    def stop_motor2(self):
        """
        Stops the secondary motor.
        """
        line = self._send_command_wait_reply("STOP2\n", ("STOP2 OK",))
        if not line.startswith("STOP2 OK"):
            raise RuntimeError(f"Failed to stop motor2: {line}")

    def reset_system(self):
        """
        Resets the system state (can be used to clear motor stopped flags).
        """
        line = self._send_command_wait_reply("RESET\n", ("RESET OK",))
        if not line.startswith("RESET OK"):
            raise RuntimeError(f"Failed to reset system: {line}")

    def get_position_mm(self):
        """
        Retrieves the current motor position in millimeters.

        Returns:
            float: Current position in millimeters.
        """
        with self.lock:
            return self.position_steps/STEPS_PER_MM

########################################
# TEMPERATURE READER
########################################

class TemperatureReader(threading.Thread):

    def __init__(self):
        """
        Initializes the TemperatureReader class, a threading class that continuously reads temperature from a MAX31865 sensor.

        Attributes:
            running (bool): Flag to control the thread execution.
            sensor (adafruit_max31865.MAX31865): Temperature sensor instance.
        """
        super().__init__()

        self.running=True

        spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
        self.cs_pins = [digitalio.DigitalInOut(pin) for pin in TEMP_SENSOR_CS_PINS]
        self.sensors = [adafruit_max31865.MAX31865(spi, cs) for cs in self.cs_pins]

    def run(self):
        """
        Runs the temperature reading loop, updating the global state with temperature readings every second.
        """
        while self.running:

            temps = [sensor.temperature for sensor in self.sensors]

            with state.lock:
                state.temperatures = temps

            time.sleep(1)

########################################
# KALMAN FILTER
########################################

class InterfaceKalman:

    def __init__(self,dt=1.0):
        """
        Initializes the InterfaceKalman class, implementing a Kalman filter for smoothing and predicting interface position.

        Args:
            dt (float, optional): Time step for the filter (default: 1.0).

        Attributes:
            dt (float): Time step.
            x (numpy.ndarray): State vector [position, velocity].
            F (numpy.ndarray): State transition matrix.
            H (numpy.ndarray): Measurement matrix.
            P (numpy.ndarray): Covariance matrix.
            Q (numpy.ndarray): Process noise covariance.
            R (numpy.ndarray): Measurement noise covariance.
        """
        self.dt=dt

        self.x=np.array([[0.0],[0.0]])

        self.F=np.array([[1,dt],[0,1]])
        self.H=np.array([[1,0]])

        self.P=np.eye(2)

        self.Q=np.array([[1e-4,0],[0,1e-4]])
        self.R=np.array([[0.5]])

    def update(self,measurement):
        """
        Updates the Kalman filter with a new measurement and returns the filtered state.

        Args:
            measurement (float): New interface position measurement.

        Returns:
            numpy.ndarray: Filtered state vector [position, velocity].
        """
        z=np.array([[measurement]])

        self.x=self.F@self.x
        self.P=self.F@self.P@self.F.T+self.Q

        y=z-self.H@self.x
        S=self.H@self.P@self.H.T+self.R
        K=self.P@self.H.T@np.linalg.inv(S)

        self.x=self.x+K@y
        self.P=(np.eye(2)-K@self.H)@self.P

        return self.x.flatten()

########################################
# SUBPIXEL INTERFACE DETECTION
########################################

def get_interface_search_bounds(frame_height, search_range_px=INTERFACE_SEARCH_RANGE_PX):
    """
    Return the start/end rows of the central horizontal band used for interface detection.
    """
    band_height = max(3, min(int(search_range_px), int(frame_height)))
    center_row = int(frame_height) // 2
    half_band = band_height // 2

    row_start = max(0, center_row - half_band)
    row_end = min(int(frame_height), row_start + band_height)
    row_start = max(0, row_end - band_height)

    return row_start, row_end


def get_target_interface_mm(frame_height, target_interface_mm=TARGET_INTERFACE_MM):
    """
    Return the target interface position in mm.

    If `TARGET_INTERFACE_MM` is None, the center of the search band/frame is used.
    """
    if target_interface_mm is not None:
        return float(target_interface_mm)

    row_start, row_end = get_interface_search_bounds(frame_height)
    target_row = 0.5 * (row_start + row_end - 1)
    return target_row * MM_PER_PIXEL


def detect_interface(frame):
    """
    Detect the ice-water interface using only the middle horizontal search band.

    Args:
        frame: Input camera frame (BGR image).

    Returns:
        tuple: (interface_position_pixels, confidence) where interface_position_pixels is the subpixel-accurate
               position in pixels, or None if detection fails, and confidence is based on the gradient peak
               strength within the center horizontal band.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    sobel = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=3)
    gradient = np.abs(sobel)

    row_strength = gradient.mean(axis=1)
    row_start, row_end = get_interface_search_bounds(frame.shape[0])
    search_strength = row_strength[row_start:row_end]

    if len(search_strength) < 3:
        return None, 0

    local_idx = int(np.argmax(search_strength))
    idx = row_start + local_idx

    peak = row_strength[idx]
    mean = np.mean(search_strength)
    confidence = peak / (mean + 1e-6)

    if local_idx <= 1 or local_idx >= len(search_strength) - 2:
        return None, 0

    g1 = row_strength[idx - 1]
    g2 = row_strength[idx]
    g3 = row_strength[idx + 1]

    denom = (g1 - 2 * g2 + g3)

    if abs(denom) < 1e-6:
        sub = idx
    else:
        sub = idx + 0.5 * (g1 - g3) / denom

    return sub, confidence

########################################
# DUAL INTERFACE DETECTION
########################################

def detect_upper_interface(frame, min_distance=20, height_threshold=0.3):
    """
    Detect the upper interface when two interfaces are present.
    
    Finds all local peaks in the vertical gradient profile and returns
    the uppermost (topmost) peak with highest confidence.
    
    Args:
        frame: Input image frame
        min_distance: Minimum pixel distance between peaks (default: 20)
        height_threshold: Minimum height relative to mean to be considered a peak (default: 0.3)
    
    Returns:
        tuple: (upper_interface_pixels, confidence) or (None, 0) if detection fails
    """
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    
    sobel = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=3)
    gradient = np.abs(sobel)
    
    row_strength = gradient.mean(axis=1)
    row_start, row_end = get_interface_search_bounds(frame.shape[0])
    search_strength = row_strength[row_start:row_end]

    if len(search_strength) < 3:
        return None, 0

    mean_strength = np.mean(search_strength)

    # Find local maxima (peaks) only in the center horizontal band
    peaks = []
    for local_idx in range(1, len(search_strength) - 1):
        if (search_strength[local_idx] > search_strength[local_idx - 1] and
            search_strength[local_idx] > search_strength[local_idx + 1] and
            search_strength[local_idx] > mean_strength * height_threshold):
            idx = row_start + local_idx
            peaks.append((idx, row_strength[idx]))
    
    if len(peaks) == 0:
        return None, 0
    
    # Filter peaks by minimum distance
    filtered_peaks = []
    for idx, strength in peaks:
        if not filtered_peaks or (idx - filtered_peaks[-1][0]) >= min_distance:
            filtered_peaks.append((idx, strength))
    
    # Calculate confidence for each peak
    peak_data = []
    for idx, strength in filtered_peaks:
        confidence = strength / (mean_strength + 1e-6)
        
        # Subpixel refinement
        if idx > 0 and idx < len(row_strength) - 1:
            g1 = row_strength[idx - 1]
            g2 = row_strength[idx]
            g3 = row_strength[idx + 1]
            denom = (g1 - 2*g2 + g3)
            
            if abs(denom) < 1e-6:
                sub = idx
            else:
                sub = idx + 0.5 * (g1 - g3) / denom
        else:
            sub = idx
        
        peak_data.append((sub, confidence))
    
    # Return the uppermost (topmost) peak
    upper_interface = min(peak_data, key=lambda x: x[0])
    
    return upper_interface[0], upper_interface[1]


def build_interface_filtered_view(frame, interface_mm=None, interface_fallback=False):
    """
    Build a visualization of the filtered image used for interface extraction.

    The view is based on the vertical Sobel gradient magnitude so the detected
    interface appears as a bright horizontal feature.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    sobel = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=3)
    gradient = np.abs(sobel)
    gradient_u8 = cv2.normalize(gradient, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    filtered_view = cv2.cvtColor(gradient_u8, cv2.COLOR_GRAY2BGR)

    img_height, img_width = filtered_view.shape[:2]
    search_start, search_end = get_interface_search_bounds(img_height)

    overlay = filtered_view.copy()
    cv2.rectangle(overlay, (0, search_start), (img_width - 1, search_end - 1), (255, 255, 0), -1)
    cv2.addWeighted(overlay, 0.12, filtered_view, 0.88, 0, filtered_view)
    cv2.rectangle(filtered_view, (0, search_start), (img_width - 1, search_end - 1), (255, 255, 0), 1)

    if interface_mm is not None and interface_mm > 0:
        row = int(round(interface_mm / MM_PER_PIXEL))
        row = max(0, min(img_height - 1, row))
        line_color = (0, 165, 255) if interface_fallback else (0, 255, 0)
        cv2.line(filtered_view, (0, row), (img_width - 1, row), line_color, 2)

    cv2.putText(filtered_view, "Filtered: Sobel |dI/dy|", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return filtered_view

########################################
# INTERFACE TRACKING
########################################

class InterfaceTracker(threading.Thread):

    def __init__(self,motor,motor_worker=None):
        """
        Initializes the InterfaceTracker class, a threading class that tracks the interface using camera and motor.

        Args:
            motor: Motor control object (e.g., ArduinoStepper or DummyMotor).

        Attributes:
            motor: Motor control instance.
            running (bool): Flag to control thread execution.
            kalman (InterfaceKalman): Kalman filter for interface smoothing.
            cap (cv2.VideoCapture): Camera capture object.
            lost_counter (int): Counter for consecutive interface detection failures.
            search_step (int): Step size for search mode in steps.
        """
        super().__init__()

        self.motor=motor
        self.motor_worker = motor_worker
        self.running=True

        self.kalman=InterfaceKalman(dt=CAPTURE_INTERVAL)

        self.cap=cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,IMAGE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,IMAGE_HEIGHT)
        # Keep camera buffering minimal to reduce display lag.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Recovery Variables
        self.lost_counter = 0
        self.search_step = int(STEPS_PER_MM * 0.1)
        self.last_detected_interface_mm = None
        self.last_detected_filtered_mm = None
        self.prev_motor_pos = None
        self.motor_pos_origin = self.motor.get_position_mm()

    @staticmethod
    def _limited_tracking_steps(error_mm, gain, max_step_per_cycle):
        """
        Convert tracking error to a bounded step command so motor motion stays smooth.
        """
        raw_steps = int((-error_mm) * STEPS_PER_MM * gain)
        if raw_steps == 0:
            return 0
        return int(np.clip(raw_steps, -max_step_per_cycle, max_step_per_cycle))

    def move_with_limit(self, steps, speed=200):
        """
        Move the motor within the configured travel limits.

        Positive steps move upward; negative steps move backward/downward.

        Args:
            steps (int): Requested travel in steps.
            speed (int, optional): Stepper speed in steps/s.
        """
        if steps == 0:
            return

        current_pos_mm = self.motor.get_position_mm()

        if steps > 0:
            remaining_mm = MAX_TRAVEL_MM - current_pos_mm
            if remaining_mm <= 0:
                return
            max_allowed_steps = int(remaining_mm * STEPS_PER_MM)
            limited_steps = min(steps, max_allowed_steps)
        else:
            available_mm = max(0.0, current_pos_mm)
            if available_mm <= 0:
                return
            max_allowed_steps = int(available_mm * STEPS_PER_MM)
            limited_steps = -min(abs(steps), max_allowed_steps)

        if limited_steps == 0:
            return

        if self.motor_worker is not None:
            self.motor_worker.request_move(limited_steps, speed=speed)
            return

        try:
            self.motor.move_steps(limited_steps, speed=speed)
        except Exception as e:
            with state.lock:
                state.motor_status = "fallback"
            print(f"Warning: failed to move motor1: {e}")

    def run(self):
        """
        Runs the interface tracking loop, capturing frames, detecting interface, applying Kalman filter,
        and controlling motor to keep interface at target position while allowing manual keyboard nudges.
        """
        next_capture = time.perf_counter()

        while self.running:

            ret,frame=self.cap.read()

            if not ret:
                continue

            # Rotate camera frame 180 degrees for correct orientation
          #  frame = cv2.rotate(frame, cv2.ROTATE_180)

            # Use only the middle fifth of the camera frame for tracking and display
            frame = frame[frame.shape[0] // 5 : 4 * frame.shape[0] // 5, 2 * frame.shape[1] // 5 : 3 * frame.shape[1] // 5]

            row,confidence = detect_interface(frame)
            search_start, search_end = get_interface_search_bounds(frame.shape[0])
            target_interface_mm = get_target_interface_mm(frame.shape[0])
            fallback_row = 0.5 * (search_start + search_end - 1)
            fallback_mm = fallback_row * MM_PER_PIXEL
            fallback_used = row is None or confidence < INTERFACE_CONF_THRESHOLD
            fallback_height = None

            if fallback_used:
                self.lost_counter += 1
                if self.last_detected_interface_mm is not None:
                    interface_mm = self.last_detected_interface_mm
                    fallback_height = self.last_detected_filtered_mm if self.last_detected_filtered_mm is not None else interface_mm
                    interface_source = "fallback_last_detected"
                else:
                    interface_mm = fallback_mm
                    fallback_height = interface_mm
                    interface_source = "fallback_center"
            else:
                self.lost_counter = 0
                interface_mm = row * MM_PER_PIXEL
                interface_source = "detected"

            with state.lock:
                manual_override_active = time.time() < state.manual_override_until

            # Normal Tracking Mode
            if not fallback_used:
                height,velocity = self.kalman.update(interface_mm)
                # Store the filtered value so fallback stays continuous with displayed detection.
                self.last_detected_interface_mm = interface_mm
                self.last_detected_filtered_mm = height
                future = height + velocity * CAPTURE_INTERVAL
                error = future - target_interface_mm
                steps = self._limited_tracking_steps(
                    error_mm=error,
                    gain=MOTOR1_TRACK_GAIN,
                    max_step_per_cycle=MOTOR1_MAX_STEP_PER_CYCLE,
                )
                # Move up when the interface is above the middle line and backward when it is below.
                if not manual_override_active and steps != 0 and abs(error) > 0.02:
                    self.move_with_limit(steps, speed=MOTOR1_TRACK_SPEED)
            # Short Loss (Prediction Hold while displaying the center-band fallback)
            elif self.lost_counter < 3:
                height,velocity = self.kalman.x.flatten()
                predicted = height + velocity * CAPTURE_INTERVAL
                error = predicted - target_interface_mm
                steps = self._limited_tracking_steps(
                    error_mm=error,
                    gain=MOTOR1_LOST_GAIN,
                    max_step_per_cycle=max(1, MOTOR1_MAX_STEP_PER_CYCLE // 2),
                )
                if not manual_override_active and steps != 0:
                    self.move_with_limit(steps, speed=MOTOR1_TRACK_SPEED)
                height = fallback_height if fallback_height is not None else interface_mm
            # Long Loss: reset display to the center of the search band.
            else:
                # if not manual_override_active:
                #     print("Interface lost — using center of search band")
                interface_mm = fallback_mm
                interface_source = "fallback_center"
                height, velocity = fallback_mm, 0

            motor_pos = self.motor.get_position_mm()

            # Compute growth rate from motor 1 displacement only
            if self.prev_motor_pos is not None:
                motor_velocity = (motor_pos - self.prev_motor_pos) / CAPTURE_INTERVAL
            else:
                motor_velocity = 0.0
            self.prev_motor_pos = motor_pos

            # Update motor status
            if manual_override_active:
                motor_status = "manual"
            elif fallback_used:
                motor_status = "fallback"
            else:
                motor_status = "running"

            with state.lock:

                state.interface_mm=interface_mm
                state.interface_filtered=(motor_pos - self.motor_pos_origin) + height
                state.interface_fallback=fallback_used
                state.interface_source=interface_source
                state.growth_rate=motor_velocity*60
                state.motor_position_mm=motor_pos
                state.motor_status=motor_status
                state.image=frame

            next_capture += CAPTURE_INTERVAL
            sleep_s = next_capture - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                # If processing overruns, resync without accumulating additional delay.
                next_capture = time.perf_counter()

########################################
# LOGGER
########################################

class Logger(threading.Thread):

    def __init__(self):
        """
        Initializes the Logger class, a threading class that logs experimental data to a CSV file.

        Attributes:
            running (bool): Flag to control thread execution.
            file (file object): Open CSV file for writing.
            writer (csv.writer): CSV writer object.
            last_flush (float): Timestamp of last file flush.
        """
        super().__init__()

        self.running=True

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"./data/experiment_log_{timestamp}.csv"
        self.file=open(self.filename,"a",newline="")
        self.writer=csv.writer(self.file)

        if os.stat(self.filename).st_size==0:

            self.writer.writerow(CSV_HEADER)

        self.last_flush=time.time()

    def run(self):
        """
        Runs the logging loop, writing data rows to CSV at regular intervals and flushing periodically.
        """
        while self.running:

            time.sleep(LOG_INTERVAL)

            with state.lock:

                row=[
                    datetime.now().isoformat(),
                    *state.temperatures,
                    state.interface_mm,
                    state.interface_filtered,
                    state.growth_rate,
                    state.motor_position_mm,
                    state.motor2_speed,
                    state.interface_source,
                    state.motor_status
                ]

            self.writer.writerow(row)

            if time.time()-self.last_flush>FLUSH_INTERVAL:

                self.file.flush()
                os.fsync(self.file.fileno())

                self.last_flush=time.time()

########################################
# LIVE DISPLAY
########################################

class LiveDisplay(threading.Thread):

    def __init__(self):
        """
        Initializes the LiveDisplay class, a threading class that provides real-time visualization of the experiment.

        Attributes:
            running (bool): Flag to control thread execution.
            temp1_hist, temp2_hist, temp3_hist, temp4_hist (collections.deque): Histories of individual temperature readings.
            interface_hist (collections.deque): History of interface positions.
            growth_hist (collections.deque): History of growth rates.
            new_data (threading.Event): Event to signal when new data is available for display.
        """
        super().__init__()

        self.running=True

        self.time_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.temp1_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.temp2_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.temp3_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.temp4_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.interface_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.interface_fallback_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.motor_position_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.growth_hist=deque(maxlen=MAX_LENGTH_DISPLAY)

        self.history_deques = (
            self.time_hist,
            self.temp1_hist,
            self.temp2_hist,
            self.temp3_hist,
            self.temp4_hist,
            self.interface_hist,
            self.interface_fallback_hist,
            self.motor_position_hist,
            self.growth_hist,
        )
        self.history_lock = threading.Lock()

        self.start_time = time.time()
        self.new_data = threading.Event()

    def _sync_history_lengths(self):
        """
        Keep all history deques aligned to the same number of samples.
        """
        target_len = min(len(hist) for hist in self.history_deques)
        for hist in self.history_deques:
            while len(hist) > target_len:
                hist.popleft()

    def get_plot_data(self):
        """
        Return synchronized snapshots of the display histories for plotting.
        """
        with self.history_lock:
            self._sync_history_lengths()
            return {
                "time": list(self.time_hist),
                "temp1": list(self.temp1_hist),
                "temp2": list(self.temp2_hist),
                "temp3": list(self.temp3_hist),
                "temp4": list(self.temp4_hist),
                "interface": list(self.interface_hist),
                "interface_fallback": list(self.interface_fallback_hist),
                "motor_position": list(self.motor_position_hist),
                "growth": list(self.growth_hist),
            }

    def run(self):
        """
        Runs the display loop, collecting data and signaling when new data is available for display.
        """
        while self.running:

            with state.lock:

                temps=state.temperatures
                i=state.interface_filtered
                interface_fallback = state.interface_fallback
                motor_pos = state.motor_position_mm
                g=state.growth_rate
                img=state.image

            padded_temps = list(temps[:4]) + [None] * max(0, 4 - len(temps))
            has_sample = any(t is not None for t in padded_temps) or i is not None or g is not None or img is not None

            if has_sample:

                elapsed_min = (time.time() - self.start_time) / 60.0
                with self.history_lock:
                    self.time_hist.append(elapsed_min)
                    self.temp1_hist.append(np.nan if padded_temps[0] is None else padded_temps[0])
                    self.temp2_hist.append(np.nan if padded_temps[1] is None else padded_temps[1])
                    self.temp3_hist.append(np.nan if padded_temps[2] is None else padded_temps[2])
                    self.temp4_hist.append(np.nan if padded_temps[3] is None else padded_temps[3])
                    self.interface_hist.append(np.nan if i is None else i)
                    self.interface_fallback_hist.append(bool(interface_fallback))
                    self.motor_position_hist.append(np.nan if motor_pos is None else motor_pos)
                    self.growth_hist.append(np.nan if g is None else g)
                    self._sync_history_lengths()

                self.new_data.set()

            time.sleep(DISPLAY_INTERVAL)

class DummyMotor:
    """
    A mock motor class for testing purposes when hardware is not available.
    """
    def move_steps(self, steps, speed=200):
        """
        No-op method for moving steps (does nothing).

        Args:
            steps (int): Number of steps (ignored).
            speed (int, optional): Speed (ignored).
        """
        pass  # No motor control
    def get_position_mm(self):
        """
        Returns a dummy position of 0.0 mm.

        Returns:
            float: Always 0.0.
        """
        return 0.0


class MotorMoveWorker(threading.Thread):
    """
    Executes motor1 move commands on a dedicated thread.

    The worker stores only the most recent requested move to avoid command backlog
    when vision updates are faster than motor execution.
    """
    def __init__(self, motor):
        super().__init__()
        self.motor = motor
        self.running = True
        self.command_lock = threading.Lock()
        self.new_command = threading.Event()
        self.pending_command = None

    def request_move(self, steps, speed):
        with self.command_lock:
            self.pending_command = (int(steps), int(speed))
            self.new_command.set()

    @staticmethod
    def _apply_travel_limit(current_pos_mm, steps):
        if steps == 0:
            return 0

        if steps > 0:
            remaining_mm = MAX_TRAVEL_MM - current_pos_mm
            if remaining_mm <= 0:
                return 0
            max_allowed_steps = int(remaining_mm * STEPS_PER_MM)
            return min(steps, max_allowed_steps)

        available_mm = max(0.0, current_pos_mm)
        if available_mm <= 0:
            return 0
        max_allowed_steps = int(available_mm * STEPS_PER_MM)
        return -min(abs(steps), max_allowed_steps)

    def stop(self):
        self.running = False
        self.new_command.set()

    def run(self):
        while self.running:
            self.new_command.wait(timeout=0.2)
            if not self.running:
                break

            with self.command_lock:
                cmd = self.pending_command
                self.pending_command = None
                self.new_command.clear()

            if cmd is None:
                continue

            requested_steps, speed = cmd

            current_pos_mm = self.motor.get_position_mm()
            limited_steps = self._apply_travel_limit(current_pos_mm, requested_steps)
            if limited_steps == 0:
                continue

            try:
                self.motor.move_steps(limited_steps, speed=speed)
                with state.lock:
                    state.motor_position_mm = self.motor.get_position_mm()
            except Exception as e:
                with state.lock:
                    state.motor_status = "fallback"
                print(f"Warning: failed to move motor1: {e}")


def move_motor1_manual(motor, requested_steps, speed=MOTOR1_MANUAL_SPEED):
    """
    Manually nudge the primary motor from the keyboard without applying travel limits.
    """
    if not isinstance(motor, ArduinoStepper) or requested_steps == 0:
        return

    manual_steps = int(requested_steps)

    with state.lock:
        state.manual_override_until = time.time() + MANUAL_OVERRIDE_DURATION

    try:
        motor.move_steps(manual_steps, speed=speed)
    except Exception as e:
        print(f"Warning: failed to move motor1: {e}")
        return

    new_pos_mm = motor.get_position_mm()
    direction = "up" if manual_steps > 0 else "down"

    with state.lock:
        state.motor_position_mm = new_pos_mm
        state.motor_status = "manual"

    print(f"Motor1 moved {direction}: {manual_steps} steps ({new_pos_mm:.2f} mm)")


def update_motor2_speed(motor, new_speed):
    """
    Updates the secondary motor speed while the experiment is running.
    """
    global MOTOR2_SPEED

    new_speed = int(new_speed)

    try:
        if isinstance(motor, ArduinoStepper):
            motor.set_motor2_speed(new_speed)
    except Exception as e:
        print(f"Warning: failed to update motor2 speed: {e}")
        return

    MOTOR2_SPEED = new_speed

    with state.lock:
        state.motor2_speed = MOTOR2_SPEED

    if MOTOR2_SPEED == 0:
        print("Motor2 stopped")
    else:
        print(f"Motor2 speed updated to {MOTOR2_SPEED} steps/s")


def update_motor1_accel(motor, new_accel):
    """
    Updates the primary motor acceleration while the experiment is running.
    """
    global MOTOR1_ACCEL

    new_accel = max(MOTOR1_ACCEL_MIN, min(MOTOR1_ACCEL_MAX, int(new_accel)))

    try:
        if isinstance(motor, ArduinoStepper):
            motor.set_motor1_accel(new_accel)
    except Exception as e:
        print(f"Warning: failed to update motor1 acceleration: {e}")
        return

    MOTOR1_ACCEL = new_accel
    print(f"Motor1 acceleration updated to {MOTOR1_ACCEL} steps/s^2")

########################################
# MAIN
########################################

def main():
    """
    The entry point of the program. Parses command-line arguments for configuration options, initializes hardware components 
    (motor, temperature sensor), starts background threads for tracking, logging, display, and temperature reading. 
    Runs an infinite loop until interrupted, then gracefully stops all threads.
    """
    global CAPTURE_INTERVAL, DISPLAY_INTERVAL, IMAGE_WIDTH, IMAGE_HEIGHT, MOTOR1_ACCEL, MOTOR2_SPEED

    parser = argparse.ArgumentParser(description="Ice Growth Tracking System")
    parser.add_argument('--no-motor', action='store_true', help='Disable motor control')
    parser.add_argument('--no-logger', action='store_true', help='Disable logging')
    parser.add_argument('--test-camera', action='store_true', help='Test camera and display only (disables motor, logger)')
    parser.add_argument('--test-temperature', action='store_true', help='Test temperature sensors only and print readings')
    parser.add_argument('--capture-interval', type=float, default=CAPTURE_INTERVAL, help=f'Capture interval in seconds (default: {CAPTURE_INTERVAL})')
    parser.add_argument('--display-interval', type=float, default=DISPLAY_INTERVAL, help=f'Display update interval in seconds (default: {DISPLAY_INTERVAL})')
    parser.add_argument('--image-width', type=int, default=640, help='Camera image width (default: 640)')
    parser.add_argument('--image-height', type=int, default=480, help='Camera image height (default: 480)')
    parser.add_argument('--motor1-accel', type=int, default=MOTOR1_ACCEL_INIT, help='Motor 1 acceleration in steps/s^2')
    parser.add_argument('--motor2-speed', type=int, default=MOTOR2_SPEED_INIT, help='Initial speed for the secondary motor in steps/sec')
    parser.add_argument('--test-arduino', action='store_true', help='Test Arduino serial communication and exit')

    args = parser.parse_args()

    if args.test_arduino:
        try:
            stepper = ArduinoStepper()
            # Test communication by sending a move command
            stepper.move_steps(6000, 500)  # Move 0 steps to get position
            print("Arduino communication successful. Current position:", stepper.get_position_mm())
        except Exception as e:
            print(f"Arduino communication failed: {e}")
        sys.exit(0)

    if args.test_camera:
        args.no_motor = True
        args.no_logger = False

    if args.test_temperature:
        temp_reader = TemperatureReader()
        temp_reader.start()
        try:
            while True:
                with state.lock:
                    temps = state.temperatures.copy()
                formatted = ", ".join(
                    f"T{i+1}={t:.2f}C" if t is not None else f"T{i+1}=None"
                    for i, t in enumerate(temps)
                )
                print(formatted)
                time.sleep(1)
        except KeyboardInterrupt:
            temp_reader.running = False
            temp_reader.join()
        sys.exit(0)

    # Update global config
    # CAPTURE_INTERVAL = args.capture_interval
    # DISPLAY_INTERVAL = args.display_interval
    # IMAGE_WIDTH = args.image_width
    # IMAGE_HEIGHT = args.image_height
    MOTOR1_ACCEL = max(MOTOR1_ACCEL_MIN, min(MOTOR1_ACCEL_MAX, int(MOTOR1_ACCEL_INIT)))
    MOTOR2_SPEED = MOTOR2_SPEED_INIT

    with state.lock:
        state.motor2_speed = MOTOR2_SPEED

    if args.no_motor:
        motor = DummyMotor()
        motor_worker = None
    else:
        motor = ArduinoStepper()
        motor_worker = MotorMoveWorker(motor)
        motor_worker.start()
        update_motor1_accel(motor, MOTOR1_ACCEL)
        print(f"Motor1 acceleration: {MOTOR1_ACCEL} steps/s^2")
        print(f"Motor2 start: speed={MOTOR2_SPEED} steps/s")
        print(f"Motor1 controls: 'w'/'s' or up/down arrows move ±{MOTOR1_MANUAL_STEP_MM:.2f} mm")
        print("Motor1 accel: ',' slower | '.' faster")
        print("Motor2 controls: '[' slower | ']' faster | '0' stop")
        motor.start_motor2(MOTOR2_SPEED)

    tracker = InterfaceTracker(motor, motor_worker=motor_worker)
    display = LiveDisplay()

    threads = [display, tracker]

    # Always read temperature
    temp_reader = TemperatureReader()
    threads.append(temp_reader)

    if not args.no_logger:
        logger = Logger()
        threads.append(logger)

    # Initialize matplotlib on main thread
    plt.ion()
    fig, ax = plt.subplots()
    fig2, ax2 = plt.subplots()
    ax_twin = ax.twinx()
    ax2_twin = ax2.twinx()
    last_plot_update = 0.0

    for thread in threads:
        thread.start()

    try:
        while True:
            # Handle display updates on main thread
            if display.new_data.is_set() and (time.time() - last_plot_update >= PLOT_UPDATE_INTERVAL):
                display.new_data.clear()
                last_plot_update = time.time()

                plot_data = display.get_plot_data()
                time_data = plot_data["time"]

                # Update main plot
                ax.clear()
                ax_twin.clear()
                if time_data:
                    interface_data = plot_data["interface"]
                    fallback_flags = plot_data["interface_fallback"]
                    detected_interface = [
                        value if not fallback else np.nan
                        for value, fallback in zip(interface_data, fallback_flags)
                    ]
                    fallback_interface = [
                        value if fallback else np.nan
                        for value, fallback in zip(interface_data, fallback_flags)
                    ]

                    if any(not np.isnan(value) for value in detected_interface):
                        ax.plot(time_data, detected_interface, label="Interface mm", color='blue')
                    if any(not np.isnan(value) for value in fallback_interface):
                        ax.plot(time_data, fallback_interface, label="Fallback last detected", color='lightblue')
                    handles, labels = ax.get_legend_handles_labels()
                    if handles:
                        ax.legend(loc='upper left')
                ax.set_xlabel('Time (min)')
                ax.set_ylabel('Interface (mm)', color='blue')
                ax.tick_params(axis='y', labelcolor='blue')

                if time_data:
                    ax_twin.plot(time_data, plot_data["growth"], label="Growth mm/min", color='red')
                ax_twin.set_ylabel('Growth Rate (mm/min)', color='red')
                ax_twin.tick_params(axis='y', labelcolor='red')

                plt.figure(fig.number)  # Switch to main figure
                plt.pause(0.01)

                # Update temperature plot
                ax2.clear()
                ax2_twin.clear()
                if time_data and plot_data["temp1"]:
                    ax2_twin.plot(time_data, plot_data["temp1"], label=CSV_HEADER[1], color='blue')
                ax2_twin.set_xlabel('Time (min)')
                ax2_twin.set_ylabel('Temperature Peltier (°C)', color='blue')
                ax2_twin.tick_params(axis='y', labelcolor='blue')
                if time_data and plot_data["temp2"]:
                    ax2.plot(time_data, plot_data["temp2"], label=CSV_HEADER[2], color='red', linestyle='--')
                if time_data and plot_data["temp3"]:
                    ax2.plot(time_data, plot_data["temp3"], label=CSV_HEADER[3], color='red', linestyle=':')
                if time_data and plot_data["temp4"]:
                    ax2.plot(time_data, plot_data["temp4"], label=CSV_HEADER[4], color='red', linestyle='-.')
                ax2.set_xlabel('Time (min)')
                ax2.set_ylabel('Temperature water (°C)', color='red')
                ax2.tick_params(axis='y', labelcolor='red')
                ax2.legend()

                plt.figure(fig2.number)  # Switch to temperature figure
                plt.pause(0.01)

            # Keep camera display responsive even when plot updates are throttled.
            with state.lock:
                img = state.image
                interface_mm = state.interface_mm
                interface_fallback = state.interface_fallback
                motor2_speed = state.motor2_speed

            if img is not None:
                annotated = img.copy()
                img_height, img_width = annotated.shape[:2]
                search_start, search_end = get_interface_search_bounds(img_height)

                overlay = annotated.copy()
                cv2.rectangle(overlay, (0, search_start), (img_width - 1, search_end - 1), (255, 255, 0), -1)
                cv2.addWeighted(overlay, 0.12, annotated, 0.88, 0, annotated)
                cv2.rectangle(annotated, (0, search_start), (img_width - 1, search_end - 1), (255, 255, 0), 1)

                if interface_mm is not None and interface_mm > 0:
                    row = int(round(interface_mm / MM_PER_PIXEL))
                    row = max(0, min(img_height - 1, row))
                    line_color = (0, 165, 255) if interface_fallback else (0, 0, 255)
                    cv2.line(annotated, (0, row), (img_width, row), line_color, 2)

                cv2.putText(annotated, f"Motor2: {motor2_speed} steps/s", (10, img_height - 145), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(annotated, f"Motor1: W/S or arrows move ±{MOTOR1_MANUAL_STEP_MM:.2f} mm", (10, img_height - 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(annotated, f"Motor1 accel: {MOTOR1_ACCEL} (',' slower | '.' faster)", (10, img_height - 98), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(annotated, "Motor2: [ slower | ] faster | 0 stop", (10, img_height - 76), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(annotated, f"Search band: center {search_end - search_start}px", (10, img_height - 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.putText(annotated, "View: left half only", (10, img_height - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                status_text = "Interface: fallback (last detected)" if interface_fallback else "Interface: detected"
                status_color = (0, 165, 255) if interface_fallback else (0, 255, 0)
                cv2.putText(annotated, status_text, (10, img_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)

                filtered_view = build_interface_filtered_view(
                    img,
                    interface_mm=interface_mm,
                    interface_fallback=interface_fallback,
                )

                combined_view = np.hstack((annotated, filtered_view))
                cv2.imshow("Interface (Live + Filtered)", combined_view)

            key = cv2.waitKeyEx(1)
            if not args.no_motor and isinstance(motor, ArduinoStepper):
                motor1_step = max(1, int(round(MOTOR1_MANUAL_STEP_MM * STEPS_PER_MM)))

                if key in (ord('w'), ord('W'), 82, 2490368, 65362):
                    move_motor1_manual(motor, motor1_step)
                elif key in (ord('s'), ord('S'), 84, 2621440, 65364):
                    move_motor1_manual(motor, -motor1_step)
                elif key in (ord(']'), ord('='), ord('+')):
                    update_motor2_speed(motor, MOTOR2_SPEED + MOTOR2_SPEED_STEP)
                elif key in (ord('['), ord('-'), ord('_')):
                    update_motor2_speed(motor, max(0, MOTOR2_SPEED - MOTOR2_SPEED_STEP))
                elif key in (ord('.'), ord('>')):
                    update_motor1_accel(motor, MOTOR1_ACCEL + MOTOR1_ACCEL_STEP)
                elif key in (ord(','), ord('<')):
                    update_motor1_accel(motor, MOTOR1_ACCEL - MOTOR1_ACCEL_STEP)
                elif key == ord('1'):
                    update_motor2_speed(motor, MOTOR2_SPEED_MIN)
                elif key == ord('2'):
                    update_motor2_speed(motor, MOTOR2_SPEED_MAX)
                elif key == ord('3'):
                    update_motor2_speed(motor, MOTOR2_SPEED_AVG)
                elif key == ord('0'):
                    update_motor2_speed(motor, 0)

            time.sleep(0.005)
    except KeyboardInterrupt:
        if not args.no_motor and isinstance(motor, ArduinoStepper):
            try:
                motor.stop_motor2()
            except Exception as e:
                print(f"Warning: failed to stop motor2 cleanly: {e}")

            if motor_worker is not None:
                motor_worker.stop()
                motor_worker.join(timeout=1.0)

        for thread in threads:
            thread.running = False

if __name__=="__main__":
    main()