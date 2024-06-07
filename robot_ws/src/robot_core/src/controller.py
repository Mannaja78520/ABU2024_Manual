from src.utilize import *
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
    ErrorTolerance = 0
    LastError = 0
    Integral = 0 
    Dt = 0
    CurrentTime = time.time()
    LastTime = CurrentTime
    
        
    def __init__(self, kp = 0, ki = 0, kd = 0, kf = 0, baseSpeed = 0, errorTolerance = 0):
        self.ConfigPIDF(kp, ki, kd, kf, baseSpeed)
        self.ErrorTolerance = abs(errorTolerance)
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
        Is_Error_In_Tolerance = AtTargetRange(error, 0, self.ErrorTolerance)
        self.Error = error if not Is_Error_In_Tolerance else 0
        self.Integral += self.Error * self.Dt
        self.Integral = clip(self.Integral, -1, 1) if not Is_Error_In_Tolerance else 0
        Derivative = (self.Error - self.LastError) / self.Dt if self.Dt != 0 else 0
        self.LastError = self.Error
        BaseSpeed = self.baseSpeed * sig_num(self.Error) if not Is_Error_In_Tolerance else 0
        return (self.Error * self.kp) + (self.Integral * self.ki) + (Derivative * self.kd) + (self.Setpoint * self.kf) + BaseSpeed
    
    def ResetVariable(self):
        self.LastError = 0
        self.Integral = 0 
        self.CurrentTime = time.time()
        self.LastTime = self.CurrentTime
        
    def Reset(self, kp = 0, ki = 0, kd = 0, kf = 0, baseSpeed = 0):
        # pidf Setting
        if kp != 0 or ki != 0 or kd != 0 or kf != 0 or baseSpeed != 0:
            if kp == 0:
                kp = self.kp
            if ki == 0:
                ki = self.ki
            if kd == 0:
                kd = self.kd
            if kf == 0:
                kf = self.kf
            if baseSpeed == 0:
                baseSpeed = self.baseSpeed
            self.ConfigPIDF(kp, ki, kd, kf, baseSpeed)
            
        
        # pid variable
        self.Setpoint = 0
        self.Error = 0
        self.LastError = 0
        self.Integral = 0 
        self.Dt = 0
        self.CurrentTime = time.time()
        self.LastTime = self.CurrentTime
        