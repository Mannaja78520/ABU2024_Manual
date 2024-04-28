#ifndef Controller_h
#define Controller_h

#include <Arduino.h>

class Controller {
  private:
    unsigned long long LastTime;
  public:
    float Kp, Ki, Kd, Kf, Setpoint, Error, LastError;
    float Dt, Integral;
    static byte SigNum(float number);
    Controller(float, float, float, float);
    void setPIDF(float, float, float, float);
    float Calculate(float, float);
    float Calculate(float);
};

#endif