#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID
{
  public:
    PID(double kpIn, double kiIn, double kdIn);
    double calculate(double currPos);
    void setTarget(double newTarget);
    double getError(double currPos);
    String getPIDPowers();
    void toggleMotor(bool motorToggler);
    
    double getCurrError();
    double getPrevError();
    double getTarget();
  private:
    double kp;
    double ki;
    double kd;
    
    double currError;
    double prevError;
    double testError;

    double derivativeError;
    unsigned long currTime;
    unsigned long prevTime;
    double integralSum;
    double maxIntegralSum;
    double target;
    bool motorToggle;
};

#endif