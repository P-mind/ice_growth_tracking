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

TARGET_INTERFACE_MM = 10
INTERFACE_CONF_THRESHOLD = 2.0  # Pixel

MAX_TRAVEL_MM = 80      # Prevent motor from moving too far in case of tracking loss

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
                "motor_position_mm"
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
        self.growth_rate = None
        self.motor_position_mm = 0
        self.pwm_duty = 0

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

        self.position_steps = 0
        self.lock = threading.Lock()

    def move_steps(self,steps,speed=1500):
        """
        Moves the stepper motor by a specified number of steps at a given speed.

        Args:
            steps (int): Number of steps to move (positive for one direction, negative for the other).
            speed (int, optional): Speed of movement in steps per second (default: 1500).
        """
        cmd=f"MOVE {steps} {speed}\n"
        self.ser.write(cmd.encode())

        line=self.ser.readline().decode().strip()

        if line.startswith("POS"):

            pos=int(line.split()[1])

            with self.lock:
                self.position_steps=pos

    def set_rpm(self, rpm):
        """
        Sets the RPM for the secondary motor via Arduino serial.

        Args:
            rpm (int): Desired RPM for the second motor.
        """
        cmd=f"SETRPM {rpm}\n"
        self.ser.write(cmd.encode())
        line=self.ser.readline().decode().strip()
        if not line.startswith("RPM OK"):
            raise RuntimeError(f"Failed to set RPM: {line}")

    def start_motor2(self):
        """
        Starts the secondary motor after RPM has been configured.
        """
        self.ser.write(b"START2\n")
        line=self.ser.readline().decode().strip()
        if not line.startswith("START2 OK"):
            raise RuntimeError(f"Failed to start motor2: {line}")

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

def detect_interface(frame):
    """
    Detects the ice-water interface in a camera frame by converting to grayscale, applying Gaussian blur, 
    computing vertical gradients with Sobel operator, and finding the row with the strongest gradient.

    Args:
        frame: Input camera frame (BGR image).

    Returns:
        tuple: (interface_position_pixels, confidence) where interface_position_pixels is the subpixel-accurate 
               position in pixels, or None if detection fails, and confidence is a score based on peak strength 
               relative to mean gradient.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)

    sobel = cv2.Sobel(blur,cv2.CV_64F,0,1,ksize=3)
    gradient = np.abs(sobel)

    row_strength = gradient.mean(axis=1)

    idx = np.argmax(row_strength)

    peak = row_strength[idx]
    mean = np.mean(row_strength)

    confidence = peak / (mean + 1e-6)

    if idx<=1 or idx>=len(row_strength)-2:
        return None,0

    g1=row_strength[idx-1]
    g2=row_strength[idx]
    g3=row_strength[idx+1]

    denom=(g1-2*g2+g3)

    if abs(denom)<1e-6:
        sub=idx
    else:
        sub=idx + 0.5*(g1-g3)/denom

    return sub,confidence

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
    
    mean_strength = np.mean(row_strength)
    
    # Find local maxima (peaks)
    peaks = []
    for i in range(1, len(row_strength) - 1):
        if (row_strength[i] > row_strength[i-1] and 
            row_strength[i] > row_strength[i+1] and
            row_strength[i] > mean_strength * height_threshold):
            peaks.append((i, row_strength[i]))
    
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

########################################
# INTERFACE TRACKING
########################################

class InterfaceTracker(threading.Thread):

    def __init__(self,motor):
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
            search_direction (int): Direction for search mode (1 or -1).
            search_step (int): Step size for search mode in steps.
        """
        super().__init__()

        self.motor=motor
        self.running=True

        self.kalman=InterfaceKalman(dt=CAPTURE_INTERVAL)

        self.cap=cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,IMAGE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,IMAGE_HEIGHT)

        # Recovery Variables
        self.lost_counter = 0
        self.search_direction = 1
        self.search_step = int(STEPS_PER_MM * 0.1)


    def run(self):
        """
        Runs the interface tracking loop, capturing frames, detecting interface, applying Kalman filter, 
        and controlling motor to keep interface at target position. Includes recovery modes for lost interface.
        """
        while self.running:

            ret,frame=self.cap.read()

            if not ret:
                continue

            # Rotate camera frame 180 degrees for correct orientation
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            row,confidence = detect_interface(frame)

            if row is None or confidence < INTERFACE_CONF_THRESHOLD:
                self.lost_counter += 1
            else:
                self.lost_counter = 0

            # Normal Tracking Mode
            if self.lost_counter == 0:
                interface_mm = row * MM_PER_PIXEL
                height,velocity = self.kalman.update(interface_mm)
                future = height + velocity * CAPTURE_INTERVAL
                error = future - TARGET_INTERFACE_MM
                steps = int(error * STEPS_PER_MM * 0.5)
                if abs(error) > 0.02:
                    self.motor.move_steps(-steps)
            # Short Loss (Prediction Hold)
            elif self.lost_counter < 3:
                interface_mm = 0
                height,velocity = self.kalman.x.flatten()
                predicted = height + velocity * CAPTURE_INTERVAL
                error = predicted - TARGET_INTERFACE_MM
                steps = int(error * STEPS_PER_MM * 0.3)
                self.motor.move_steps(-steps)
            # Long Loss (Search Mode)
            else:
                print("Interface lost — searching")
                interface_mm = 0
                height, velocity = 0, 0
                self.motor.move_steps(self.search_direction * self.search_step)
                if self.lost_counter % 20 == 0:
                    self.search_direction *= -1
            
            # Prevent stage runaway
            if abs(self.motor.get_position_mm()) < MAX_TRAVEL_MM:
                print("Prevent stage runaway")
                self.motor.move_steps(20)

            motor_pos=self.motor.get_position_mm()

            # row=detect_interface(frame)
            # interface_mm=row*MM_PER_PIXEL
            # height,velocity=self.kalman.update(interface_mm)
            # future=height+velocity*CAPTURE_INTERVAL
            # error=future-TARGET_INTERFACE_MM
            # steps=int(error*STEPS_PER_MM*0.5)
            # if abs(error)>0.02:
            #     self.motor.move_steps(-steps)

            # motor_pos=self.motor.get_position_mm()

            with state.lock:

                state.interface_mm=interface_mm
                state.interface_filtered=height
                state.growth_rate=velocity*60
                state.motor_position_mm=motor_pos
                state.image=frame

            time.sleep(CAPTURE_INTERVAL)

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
                    state.motor_position_mm
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

        self.temp1_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.temp2_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.temp3_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.temp4_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.interface_hist=deque(maxlen=MAX_LENGTH_DISPLAY)
        self.growth_hist=deque(maxlen=MAX_LENGTH_DISPLAY)

        self.new_data = threading.Event()

    def run(self):
        """
        Runs the display loop, collecting data and signaling when new data is available for display.
        """
        while self.running:

            with state.lock:

                temps=state.temperatures
                i=state.interface_filtered
                g=state.growth_rate
                img=state.image

            if any(t is not None for t in temps):

                if len(temps) >= 1 and temps[0] is not None: self.temp1_hist.append(temps[0])
                if len(temps) >= 2 and temps[1] is not None: self.temp2_hist.append(temps[1])
                if len(temps) >= 3 and temps[2] is not None: self.temp3_hist.append(temps[2])
                if len(temps) >= 4 and temps[3] is not None: self.temp4_hist.append(temps[3])
                self.interface_hist.append(i)
                self.growth_hist.append(g)

                self.new_data.set()

            time.sleep(DISPLAY_INTERVAL)

class DummyMotor:
    """
    A mock motor class for testing purposes when hardware is not available.
    """
    def move_steps(self, steps, speed=1500):
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

########################################
# MAIN
########################################

def main():
    """
    The entry point of the program. Parses command-line arguments for configuration options, initializes hardware components 
    (motor, temperature sensor), starts background threads for tracking, logging, display, and temperature reading. 
    Runs an infinite loop until interrupted, then gracefully stops all threads.
    """
    parser = argparse.ArgumentParser(description="Ice Growth Tracking System")
    parser.add_argument('--no-motor', action='store_true', help='Disable motor control')
    parser.add_argument('--no-logger', action='store_true', help='Disable logging')
    parser.add_argument('--test-camera', action='store_true', help='Test camera and display only (disables motor, logger)')
    parser.add_argument('--test-temperature', action='store_true', help='Test temperature sensors only and print readings')
    parser.add_argument('--capture-interval', type=float, default=1.0, help='Capture interval in seconds (default: 1.0)')
    parser.add_argument('--display-interval', type=float, default=0.5, help='Display update interval in seconds (default: 0.5)')
    parser.add_argument('--image-width', type=int, default=640, help='Camera image width (default: 640)')
    parser.add_argument('--image-height', type=int, default=480, help='Camera image height (default: 480)')
    parser.add_argument('--motor2-rpm', type=int, default=120, help='RPM for the secondary motor')
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
    global CAPTURE_INTERVAL
    CAPTURE_INTERVAL = args.capture_interval
    global DISPLAY_INTERVAL
    DISPLAY_INTERVAL = args.display_interval
    global IMAGE_WIDTH
    IMAGE_WIDTH = args.image_width
    global IMAGE_HEIGHT
    IMAGE_HEIGHT = args.image_height

    if args.no_motor:
        motor = DummyMotor()
    else:
        motor = ArduinoStepper()
        motor.set_rpm(args.motor2_rpm)
        motor.start_motor2()

    tracker = InterfaceTracker(motor)
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

    for thread in threads:
        thread.start()

    try:
        while True:
            # Handle display updates on main thread
            if display.new_data.is_set():
                display.new_data.clear()

                # Update main plot
                ax.clear()
                ax.plot(display.interface_hist, label="Interface mm", color='blue')
                ax.set_ylabel('Interface (mm)', color='blue')
                ax.tick_params(axis='y', labelcolor='blue')

                ax_twin.plot(display.growth_hist, label="Growth mm/min", color='red')
                ax_twin.set_ylabel('Growth Rate (mm/min)', color='red')
                ax_twin.tick_params(axis='y', labelcolor='red')

                plt.figure(fig.number)  # Switch to main figure
                plt.pause(0.01)

                # Update temperature plot
                ax2.clear()
                if display.temp1_hist: ax2_twin.plot(display.temp1_hist, label=CSV_HEADER[1], color='blue')
                ax2_twin.set_ylabel('Temperature Peltier (°C)', color='blue')
                ax2_twin.tick_params(axis='y', labelcolor='blue')
                if display.temp2_hist: ax2.plot(display.temp2_hist, label=CSV_HEADER[2], color='red', linestyle='--')
                if display.temp3_hist: ax2.plot(display.temp3_hist, label=CSV_HEADER[3], color='red', linestyle=':')
                if display.temp4_hist: ax2.plot(display.temp4_hist, label=CSV_HEADER[4], color='red', linestyle='-.')
                ax2.set_ylabel('Temperature water (°C)', color='red')
                ax2.tick_params(axis='y', labelcolor='red')
                ax2.legend()

                plt.figure(fig2.number)  # Switch to temperature figure
                plt.pause(0.01)

                # Handle camera display
                with state.lock:
                    img = state.image
                    interface_mm = state.interface_mm

                if img is not None:
                    row = int((interface_mm or 0) / MM_PER_PIXEL)
                    annotated = img.copy()
                    cv2.line(annotated, (0, row), (IMAGE_WIDTH, row), (0, 0, 255), 2)
                    cv2.imshow("Interface", annotated)
                    cv2.waitKey(1)

            time.sleep(DISPLAY_INTERVAL)
    except KeyboardInterrupt:
        for thread in threads:
            thread.running = False

if __name__=="__main__":
    main()