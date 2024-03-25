#ifndef RobotAnalysis_h
#define RobotAnalysis_h

#include <Robot.h>

class Controller{
    public:
        double Kp, Ki, Kd, Kf, Setpoint, Error, LastError;
        double Dt, Integral, LastTime;
        Controller(double, double, double, double);
        void setPIDF(float, float, float, float);
        double Calculate(double);
        double Calculate(double, double);
};

#endif