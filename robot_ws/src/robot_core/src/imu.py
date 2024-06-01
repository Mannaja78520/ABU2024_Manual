from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import time

class IMU:
    accel_data = [0.0, 0.0, 0.0]
    gyro_data = [0.0, 0.0, 0.0]
    ax = 0.0
    ay = 0.0
    az = 0.0
    gx = 0.0
    gy = 0.0
    gz = 0.0
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
        self.mpu.configure()
        self.mpu.calibrate()
        self.mpu.configure()
        self.mpu.abias
        self.mpu.gbias
    def read(self):        
        # This library send data as a Deg/sec
        # Read sensor data
        self.accel_data = self.mpu.readAccelerometerMaster()
        self.gyro_data = self.mpu.readGyroscopeMaster()
        
        # Store the data in class attributes
        self.ax, self.ay, self.az = self.accel_data
        self.gx, self.gy, self.gz = self.gyro_data