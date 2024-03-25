#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller{
    double Dt, Integral, CurrentTime, LastTime;
    public:
        double Kp, Ki, Kd, Kf, Setpoint, Error, LastError;
        Controller(double, double, double, double);
        void setPIDF(float, float, float, float);
        double Calculate(double);
        double Calculate(double, double);
};

#endif