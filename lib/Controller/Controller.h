#ifndef CONTROLLER_H
#define CONTROLLER_H

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