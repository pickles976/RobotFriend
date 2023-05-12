#!/usr/bin/env python
from math import cos, sin, pi
import serial
import json
import time
import numpy as np
from sklearn.linear_model import LinearRegression

### chip is rotated so X is Z axis
### robot forward is Z, robot sideways is Y
ROTATION_AXIS = "X"
FORWARD_AXIS = "Z"

def integrate_rotation(data, dt):
    angular_z = data["G"][ROTATION_AXIS]
    return angular_z * dt

def integrate_velocity(data, dt):
    acc = data["A"][FORWARD_AXIS] * 9.81 #g's to m/s^2
    return acc * dt

def integrate_position(position, angle, velocity, dt):
    dvel = velocity * dt
    rad = angle * pi / 180.0
    position[0] += cos(rad) * dvel
    position[1] += sin(rad) * dvel
    return position    

class IMUReader:

    def __init__(self, port):

        self.port = port
        self.ser = None  

        self.velocity_raw = 0
        self.velocity = 0
        self.angle_raw = 0
        self.angle = 0
        self.dTheta = 0

        # These are used when we want to adjust our values from either outside measurements, or
        # just to correct for the drift that occurs when we aren't active
        self.rot_bias = 0 

        # y = mx + b correcting linear drift
        self.acc_coeff = 0
        self.acc_intercept = 0
        self.rot_coeff = 0
        self.rot_intercept = 0

        self.start = 0
        self.elapsed = 0
        self.dt = 0

    def initialize_connection(self):

        # Connect
        self.ser = serial.Serial(self.port, baudrate=921600)  # open serial port
        print("Connected on: %s"%self.ser.name)         # check which port was really used

        # Perform linear regression
        print("Detecting drift...")
        velocity = 0
        angle = 0
        velocity_measurements = []
        angle_measurements = []
        time_measurements = []

        # get first 2s of data
        start = time.time()
        while True:

            imu_string = self.ser.readline().decode('utf8')
            imu_data = {}

            try: 
                imu_data = json.loads(imu_string)
            except:
                print("String not JSON format: %s"%imu_string)
                self.elapsed = 0

            if "G" in imu_data:
                self.dt = time.time() - start # time since last measurement
                self.elapsed +=self.dt
                self.start = time.time()
                angle += integrate_rotation(imu_data, self.dt)
                velocity += integrate_velocity(imu_data, self.dt)
                angle_measurements.append(angle)
                velocity_measurements.append(velocity)
                time_measurements.append(self.elapsed)
                print(self.elapsed)

            if self.elapsed > 2.0:
                break
                
        # ACCELERATION get line of best fit
        # linear regression blah blah blah
        time_measurements = np.array(time_measurements, dtype=np.float32).reshape((-1, 1)) # X
        velocity_measurements = np.array(velocity_measurements, dtype=np.float32) # Y
        model = LinearRegression().fit(time_measurements, velocity_measurements)

        r_sq = model.score(time_measurements, velocity_measurements)
        self.acc_intercept = model.intercept_
        self.acc_coeff = model.coef_[0]

        print("Acceleration drift linear regression R2 value: %s"%r_sq)
        print("Acceleration drift regression slope: %s"%self.acc_coeff)
        print("Accelertaion drift y-intercept: %s"%self.acc_intercept)

        # ROTATION get line of best fit
        angle_measurements = np.array(angle_measurements, dtype=np.float32) # Y
        model = LinearRegression().fit(time_measurements, angle_measurements)

        r_sq = model.score(time_measurements, angle_measurements)
        self.rot_intercept = model.intercept_
        self.rot_coeff = model.coef_[0]

        print("Angular drift linear regression R2 value: %s"%r_sq)
        print("Angular drift regression slope: %s"%self.rot_coeff)
        print("Angular drift y-intercept: %s"%self.rot_intercept)

        self.elapsed = 0
        self.start = time.time()

    ### Update internal state
    def update(self):

        imu_string = self.ser.readline().decode('utf8')
        imu_data = {}

        try: 
            imu_data = json.loads(imu_string)
        except:
            print("Failed to read. String not JSON format: %s"%imu_string)

        if "G" in imu_data:
            dt = time.time() - self.start # time since last measurement
            self.elapsed += dt
            self.start = time.time()
            self.angle_raw += integrate_rotation(imu_data, dt)
            self.angle = self.angle_raw - ((self.elapsed * self.rot_coeff) + self.rot_intercept) - self.rot_bias
            self.velocity_raw += integrate_velocity(imu_data, dt)
            self.velocity = self.velocity_raw - ((self.elapsed * self.acc_coeff) + self.acc_intercept)

    def start_reading_accel(self):
        self.velocity_raw = 0
        self.elapsed = 0
        self.velocity = 0

    def stop(self):
        self.ser.close()