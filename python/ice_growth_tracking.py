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

########################################
# USER CONFIGURATION
########################################

CAMERA_INDEX = 0

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

CAPTURE_INTERVAL = 2.0
LOG_INTERVAL = 1.0
FLUSH_INTERVAL = 600

TARGET_INTERFACE_MM = 10
INTERFACE_CONF_THRESHOLD = 2.0  # Pixel

MAX_TRAVEL_MM = 50      # Prevent motor from moving too far in case of tracking loss

MM_PER_PIXEL = 0.02
STEPS_PER_MM = 200

PID_TARGET_TEMP = -5.0

SERIAL_PORT = "/dev/ttyACM0"

########################################
# GLOBAL STATE
########################################

class SystemState:

    def __init__(self):

        self.lock = threading.Lock()

        self.temperature = None
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

        self.ser = serial.Serial(SERIAL_PORT,115200,timeout=2)
        time.sleep(2)

        self.position_steps = 0
        self.lock = threading.Lock()

    def move_steps(self,steps,speed=1500):

        cmd=f"MOVE {steps} {speed}\n"
        self.ser.write(cmd.encode())

        line=self.ser.readline().decode().strip()

        if line.startswith("POS"):

            pos=int(line.split()[1])

            with self.lock:
                self.position_steps=pos

    def get_position_mm(self):

        with self.lock:
            return self.position_steps/STEPS_PER_MM

########################################
# PID CONTROLLER
########################################

class PIDController(threading.Thread):

    def __init__(self):

        super().__init__()

        self.running=True

        self.Kp=15
        self.Ki=0.5
        self.Kd=2

        self.integral=0
        self.prev_error=0

        pwm_pin = board.D18
        self.pwm = pwmio.PWMOut(pwm_pin, frequency=20000)

        spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
        cs = digitalio.DigitalInOut(board.D5)

        self.sensor = adafruit_max31865.MAX31865(spi,cs)

    def run(self):

        while self.running:

            temp = self.sensor.temperature

            error = PID_TARGET_TEMP-temp

            self.integral += error
            derivative = error-self.prev_error

            output = self.Kp*error + self.Ki*self.integral + self.Kd*derivative

            duty = max(0,min(65535,int(output)))

            self.pwm.duty_cycle = duty

            with state.lock:

                state.temperature = temp
                state.pwm_duty = duty/65535

            self.prev_error = error

            time.sleep(1)

########################################
# KALMAN FILTER
########################################

class InterfaceKalman:

    def __init__(self,dt=2.0):

        self.dt=dt

        self.x=np.array([[0.0],[0.0]])

        self.F=np.array([[1,dt],[0,1]])
        self.H=np.array([[1,0]])

        self.P=np.eye(2)

        self.Q=np.array([[1e-4,0],[0,1e-4]])
        self.R=np.array([[0.5]])

    def update(self,measurement):

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
# INTERFACE TRACKING
########################################

class InterfaceTracker(threading.Thread):

    def __init__(self,motor):

        super().__init__()

        self.motor=motor
        self.running=True

        self.kalman=InterfaceKalman()

        self.cap=cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,IMAGE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,IMAGE_HEIGHT)

        # Recovery Variables
        self.lost_counter = 0
        self.search_direction = 1
        self.search_step = int(STEPS_PER_MM * 0.1)


    def run(self):

        while self.running:

            ret,frame=self.cap.read()

            if not ret:
                continue

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

        super().__init__()

        self.running=True

        self.file=open("experiment_log.csv","a",newline="")
        self.writer=csv.writer(self.file)

        if os.stat("experiment_log.csv").st_size==0:

            self.writer.writerow([
                "timestamp",
                "temperature_C",
                "interface_mm",
                "filtered_interface_mm",
                "growth_rate_mm_min",
                "motor_position_mm",
                "pwm_duty"
            ])

        self.last_flush=time.time()

    def run(self):

        while self.running:

            time.sleep(LOG_INTERVAL)

            with state.lock:

                row=[
                    datetime.now().isoformat(),
                    state.temperature,
                    state.interface_mm,
                    state.interface_filtered,
                    state.growth_rate,
                    state.motor_position_mm,
                    state.pwm_duty
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

        super().__init__()

        self.running=True

        self.temp_hist=deque(maxlen=1000)
        self.interface_hist=deque(maxlen=1000)
        self.growth_hist=deque(maxlen=1000)

        plt.ion()

        self.fig,self.ax=plt.subplots()

    def run(self):

        while self.running:

            with state.lock:

                t=state.temperature
                i=state.interface_filtered
                g=state.growth_rate
                img=state.image

            if t is not None:

                self.temp_hist.append(t)
                self.interface_hist.append(i)
                self.growth_hist.append(g)

                self.ax.clear()

                self.ax.plot(self.temp_hist,label="Temp C")
                self.ax.plot(self.interface_hist,label="Interface mm")
                self.ax.plot(self.growth_hist,label="Growth mm/min")

                self.ax.legend()

                plt.pause(0.01)

            if img is not None:

                row=int((state.interface_mm or 0)/MM_PER_PIXEL)

                annotated=img.copy()

                cv2.line(
                    annotated,
                    (0,row),
                    (IMAGE_WIDTH,row),
                    (0,0,255),
                    2
                )

                cv2.imshow("Interface",annotated)

                cv2.waitKey(1)

            time.sleep(1)

########################################
# MAIN
########################################

def main():

    motor=ArduinoStepper()

    pid=PIDController()
    tracker=InterfaceTracker(motor)
    logger=Logger()
    display=LiveDisplay()

    pid.start()
    tracker.start()
    logger.start()
    display.start()

    try:

        while True:
            time.sleep(1)

    except KeyboardInterrupt:

        pid.running=False
        tracker.running=False
        logger.running=False
        display.running=False

if __name__=="__main__":
    main()