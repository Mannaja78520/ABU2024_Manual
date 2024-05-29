from robot_core.utilize import *
import time

class Controller:
    # pidf Setting
    kp = 0
    ki = 0
    kd = 0
    kf = 0
    baseSpeed = 0
    
    Setpoint = 0
    Error = 0
    LastError = 0
    Integral = 0 
    Dt = 0
    CurrentTime = time.time()
    LastTime = CurrentTime
    
        
    def __init__(self, kp = 0, ki = 0, kd = 0, kf = 0, baseSpeed = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf
        self.baseSpeed = baseSpeed
        self.LastTime = time.time()
        
    def ConfigPIDF(self, kp = 0, ki = 0, kd = 0, kf = 0, baseSpeed = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf
        self.baseSpeed = baseSpeed
    
    def Calculate(self, setpoint, measurement):
        return self.Calculate(setpoint - measurement)
    
    def Calculate(self, error):
        self.CurrentTime = time.time()
        self.Dt = self.CurrentTime - self.LastTime
        self.LastTime  = self.CurrentTime
        self.Error = 0 if NormalizeRads(error) == 0 else error
        self.Integral += self.Error * self.Dt
        self.Integral = 0 if self.Error == 0 else clip(self.Integral, -1, 1)
        Derivative = (self.Error - self.LastError) / self.Dt
        self.LastError = self.Error
        return (self.Error * self.kp) + (self.Integral * self.ki) + (Derivative * self.kd) + (self.Setpoint * self.kf) + (self.baseSpeed * sig_num(self.Error))
    
    
    def ResetVariable(self):
        self.LastError = 0
        self.Integral = 0 
        self.CurrentTime = time.time()
        self.LastTime = self.CurrentTime
        
    def Reset(self, kp = 0, ki = 0, kd = 0, kf = 0, baseSpeed = 0):
        # pidf Setting
        self.ConfigPIDF(kp, ki, kd, kf, baseSpeed)
        
        # pid variable
        self.Setpoint = 0
        self.Error = 0
        self.LastError = 0
        self.Integral = 0 
        self.Dt = 0
        self.CurrentTime = time.time()
        self.LastTime = self.CurrentTime
        