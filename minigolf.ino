#include <Servo.h>
#include <AccelStepper.h>
#include "PID.h"
#include <Encoder.h>

AccelStepper stepper(AccelStepper::DRIVER, 23, 25);

// drivebase pins
const int dir1 = 8;
const int pwm1 = 9;
const int dir2 = 10;
const int pwm2 = 11;

const int launchEncoderA = 18;  // yellow
const int launchEncoderB = 19;  // white

PID Launch(0.0015, 0.0, 0.0002);
Encoder LaunchEncoder(launchEncoderA, launchEncoderB);
double launchTolerance = 40.0;

// launcher hitter pins
const int dir1Hitter = 31;
const int pwm1Hitter = 2;

// right joystick pins
const int RxPin = 7;  // channel 1
const int RyPin = 6;  // channel 2

// left joystick Y
const int LyPin = 5;  // channel 3

// stepper buttons
const int stepperInputPin = 4;  // channel 4
int stepperInputVal = 0;

int xval = 0;
int yval = 0;
int hitterVal = 0;

int xvalcorrected = 0;
int yvalcorrected = 0;

int leftDir = LOW;
int rightDir = HIGH;

Servo lifter;

const int servoPin = 12;
int servoVal = 0;
int lifterUpPos = 0;
int lifterDownPos = 90;

// mode: 0 = servo, 1 = hitter
int mode = 0;
int modeVal = 0;
const int modePin = 3;  // channel 5

int stepperPos = 1;
int prevStepperPos = 1;

int state = 0;

double launchPower = 0;

void setup() {
  pinMode(dir1, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm2, OUTPUT);

  pinMode(dir1Hitter, OUTPUT);
  pinMode(pwm1Hitter, OUTPUT);

  pinMode(RxPin, INPUT);
  pinMode(RyPin, INPUT);
  pinMode(LyPin, INPUT);
  pinMode(modePin, INPUT);
  pinMode(stepperInputPin, INPUT);

  lifter.attach(servoPin);

  stepper.setMaxSpeed(40000);
  stepper.setAcceleration(10000);

  Serial.begin(9600);
}

void loop() {
  xval = pulseIn(RxPin, HIGH);
  xvalcorrected = map(xval, 985, 2002, -245, 245);

  yval = pulseIn(RyPin, HIGH);
  yvalcorrected = map(yval, 985, 2002, 245, -245);

  int leftValue = yvalcorrected + xvalcorrected;
  int rightValue = yvalcorrected - xvalcorrected;

  if (leftValue > 0) {
    leftDir = HIGH;
  } else {
    leftDir = LOW;
    leftValue *= -1;
  }

  if (rightValue > 0) {
    rightDir = HIGH;
  } else {
    rightDir = LOW;
    rightValue *= -1;
  }

  if (leftValue < 30 || leftValue > 560) {
    leftValue = 0;
  }

  if (rightValue < 30 || rightValue > 560) {
    rightValue = 0;
  }

  leftValue = constrain(leftValue, 0, 250);
  rightValue = constrain(rightValue, 0, 250);

  //analogWrite(pwm1, leftValue);
  //analogWrite(pwm2, rightValue);
  digitalWrite(dir1, leftDir);
  digitalWrite(dir2, rightDir);

  // mode switch
  int modeVal = pulseIn(modePin, HIGH);
  mode = map(modeVal, 1200, 1800, 0, 1);

  if (mode == 0) {
    // servo lifter
    servoVal = pulseIn(LyPin, HIGH);
    servoVal = map(servoVal, 985, 2000, 180, 0);
    //Serial.println("servo mode");
    lifter.write(servoVal);
  } else {
    // run hitter
    hitterVal = pulseIn(LyPin, HIGH);
    hitterVal = map(hitterVal, 985, 2000, 0, 100);
    //Serial.println(hitterVal);

    if (hitterVal > 50) {
      if (state == 0) {
        Launch.setTarget(LaunchEncoder.read() + 9600 / 2);
        state = 1;
      }
    } else {
      if (state == 1) {
        Launch.setTarget(LaunchEncoder.read() + 9600 / 2);
        state = 0;
      }
    }
  }

  stepperInputVal = pulseIn(stepperInputPin, HIGH);
  //stepperInputVal = map(stepperInputVal, 985, 20002, 0, 10000);

  if (stepperInputVal < 1100) {
    stepperPos = 1;
  } else if (stepperInputVal >= 1100 && stepperInputVal < 1300) {
    stepperPos = 2;
  } else if (stepperInputVal >= 1300 && stepperInputVal < 1500) {
    stepperPos = 3;
  } else if (stepperInputVal >= 1500 && stepperInputVal < 1700) {
    stepperPos = 4;
  } else if (stepperInputVal >= 1700 && stepperInputVal < 1900) {
    stepperPos = 5;
  } else {
    stepperPos = 6;
  }

  if (stepperPos != prevStepperPos) {
    stepper.moveTo(4000 * (stepperPos - 1));
    if (stepper.distanceToGo() != 0) {
      stepper.run();
      launchPower = constrain(Launch.calculate(LaunchEncoder.read()), -1.0, 1.0);
      setPower(dir1Hitter, pwm1Hitter, launchPower);
    }
  }

  prevStepperPos = stepperPos;

  launchPower = constrain(Launch.calculate(LaunchEncoder.read()), -1.0, 1.0);
  setPower(dir1Hitter, pwm1Hitter, launchPower);
  Serial.println(" C-Encoder: " + String(LaunchEncoder.read()) + " C-Power: " + launchPower + " C-Error " + String(Launch.getError(LaunchEncoder.read())) + " " + Launch.getPIDPowers());
}


void setPower(int motorDir, int motorPWM, double power) {
  int absPower = (int)(abs(constrain(power, -1.0, 1.0)) * 255.0);
  int direct = (power < 0) ? HIGH : LOW;

  digitalWrite(motorDir, -direct);
  analogWrite(motorPWM, absPower);
}