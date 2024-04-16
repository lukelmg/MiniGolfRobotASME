#include "Arduino.h"
#include "PID.h"

#define maxIWindup 0.2

PID::PID(double kpIn, double kiIn, double kdIn)
{
  kp = kpIn;
  ki = kiIn;
  kd = kdIn;

  currError = 0;
  prevError = 0;

  prevTime = 0;

  integralSum = 0;
  maxIntegralSum = maxIWindup / ki;

  target = 0;

  motorToggle = true;
}

double PID::calculate(double currPos)
{
  currTime = millis();
  
  if(!motorToggle)
    return 0.0;

  //Serial.println("curr: " + String(currError) + " prev: " + String(prevError));
  
  currError = target - currPos;
  double deltaTime = (float)(currTime - prevTime) * 1000;
  if(deltaTime <= 0.0)
    deltaTime = 0.001;

  derivativeError = (currError - prevError) / deltaTime;
  //double trapezoid_rule_area = (currError + prevError) / 2;
  integralSum = (0.996 * integralSum) + (currError * deltaTime);
  integralSum = constrain(integralSum, -maxIntegralSum, maxIntegralSum);

  double out = (kp * currError) + (ki * integralSum) + (kd * derivativeError);

  testError = prevError;
  prevError = currError;
  prevTime = currTime;

  return out;
}

void PID::setTarget(double newTarget)
{
  if(target != newTarget){
    target = newTarget;
    integralSum = 0;
    prevError = 0;
  }
}

double PID::getError(double currPos)
{
  return target - currPos;
}

String PID::getPIDPowers()
{
  return "P: " + String(kp * currError) + " I: " + String(ki * integralSum) + " D: " + String(kd * derivativeError);
}

void PID::toggleMotor(bool motorToggler)
{
  motorToggle = motorToggler;
}

double PID::getCurrError()  {return currError;}
double PID::getPrevError()  {return testError;}
double PID::getTarget()     {return target;}