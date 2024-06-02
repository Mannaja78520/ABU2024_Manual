from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import time

from src.utilize import To_Radians, WrapDegs

class IMU:
    accel_data = [ax, ay, az] = [0.0, 0.0, 0.0]
    # velocity_data = [vx, vy, vz] = [0.0, 0.0, 0.0]
    # distance_data = [dx, dy, dz] = [0.0, 0.0, 0.0]
    gyro_data = [gx, gy, gz] = [0.0, 0.0, 0.0]
    yaw = 0.0
    CurrentTime = time.time()
    LastTime = CurrentTime
    def __init__(self):
        self.mpu = MPU9250( 
            address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
            address_mpu_slave=None, 
            bus=1, 
            gfs=GFS_1000, 
            afs=AFS_8G, 
            )
        self.configure()
    def configure(self):
        # Config imu
        # self.mpu.configure()
        self.mpu.calibrate()
        self.mpu.configure()
        self.mpu.abias
        self.mpu.gbias
        
    def reset_yaw(self):
        self.yaw = 0.0
    def read(self):        
        # Read sensor data
        # This library send Gyro_data as a Deg/sec
        self.CurrentTime = time.time()
        Dt = self.CurrentTime - self.LastTime
        self.LastTime = self.CurrentTime
        
        self.accel_data = self.mpu.readAccelerometerMaster()
        self.gyro_data = self.mpu.readGyroscopeMaster()
        
        # filter
        self.accel_data[0] = 0.0 if abs(self.accel_data[0]) < 0.05 else self.accel_data[0]
        self.accel_data[1] = 0.0 if abs(self.accel_data[1]) < 0.05 else self.accel_data[1]
        
        filter_gz = 0 if abs(self.gyro_data[2]) < 0.475 else self.gyro_data[2]
        
        self.yaw += WrapDegs(filter_gz * Dt)

        # Store the data in class attributes
        self.ax, self.ay, self.az = self.accel_data
        self.gx, self.gy, self.gz = self.gyro_data