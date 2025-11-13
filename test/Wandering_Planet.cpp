#include <Arduino.h>
#include <SoftPWM.h>
#include "Variables.h"
#include "Function.h"

void setup() {

  // --- Setup ---
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);

  pinMode(leftMotorDIR, OUTPUT);
  pinMode(rightMotorDIR, OUTPUT);

  SoftPWMBegin();
  SoftPWMSet(leftMotorPWM, 0);
  SoftPWMSet(rightMotorPWM, 0);
  SoftPWMSetFadeTime(leftMotorPWM, 0, 0);
  SoftPWMSetFadeTime(rightMotorPWM, 0, 0);

  // --- Function ---
  //moveMotor(80, 60, 300);
  Trace_to_Cross_T_Junction();
  // moveMotor(80, 80, 200);
  // Trace_to_Cross_T_Junction();
  // moveMotor(80, 80, 200);
  // turnRight();
  // Trace_to_Cross_T_Junction():
  // moveMotor(80, 80, 300);
  // turnLeft();
  // Trace_To_Left_Junction();
  // moveMotor(80, 80, 200);
  // turnLeft();
  // Trace_to_Right_Junction();
  // moveMotor(80, 80, 200);
  // turnRight();
  // Trace_to_Cross_T_Junction();
  // moveMotor(80, 80, 200);
  // turnRight();
  // setLinetrackingBaseSpeed(50, 30, 60);
  // Trace_to_Cross_T_Junction();
  // moveMotor(0, 0, 0);
}

void loop() {}

// --- Read the sensor then convert to bool value ---
void readSensors(){
  sensor[0] = analogRead(S1);
  sensor[1] = analogRead(S2);
  sensor[2] = analogRead(S3);
  sensor[3] = analogRead(S4);

  s1 = (sensor[0] < thresholdS1);
  s2 = (sensor[1] < thresholdS2);
  s3 = (sensor[2] < thresholdS3);
  s4 = (sensor[3] < thresholdS4);
}


void moveMotor(int leftSpeed, int rightSpeed, int duration){
  // --- This is the motor move function ---
  // How to use the function:
  // moveMotor(speed, speed, time);

  bool leftForward = leftSpeed > 0;
  bool rightForward = rightSpeed > 0;

  digitalWrite(leftMotorDIR, leftForward ? HIGH : LOW);
  digitalWrite(rightMotorDIR, rightForward ? HIGH : LOW);

  int leftPWM = map(abs(leftSpeed), 0, 100, 0, 255);
  int rightPWM = map(abs(rightSpeed), 0, 100, 0, 255);

  SoftPWMSet(leftMotorPWM, leftPWM);
  SoftPWMSet(rightMotorPWM, rightPWM);

  unsigned long start = millis();
  while (millis() - start < (unsigned long)duration) { }

  SoftPWMSet(leftMotorPWM, 0);
  SoftPWMSet(rightMotorPWM, 0);
}

void Trace_to_Cross_T_Junction(){
  readSensors();
  while(true){
    readSensors();
    if (s1 && s4) break;
    followLine();
  }
}

void Trace_to_Right_Junction(){
  readSensors();
  while(true){
    readSensors();
    if (s4) break;
    followLine();
  }
}

void Trace_to_Left_Junction(){
  readSensors();
  while(true){
    readSensors();
    if (s1) break;
    followLine();
  }
}

void turnLeft(){
  readSensors();
  while(!s1){
    readSensors();
    moveMotor(-80, 60, 10);
  }
  while(!s2){
    readSensors();
    moveMotor(-80, 60, 10);
  }
}

void turnRight(){
  readSensors();
  while(!s1){
    readSensors();
    moveMotor(80, -60, 10);
  }
  while(!s2){
    readSensors();
    moveMotor(80, -60, 10);
  }
}

void setLinetrackingBaseSpeed(int setBaseSpeed, int setMinSpeed, int setMaxSpeed){
  baseSpeed = setBaseSpeed;
  minSpeed = setMinSpeed;
  maxSpeed = setMaxSpeed;
}

void EnergyDetection(){
  delay(500);
  moveMotor(-25, 20, 100);
  delay(1000);
  //Servo Motor
  delay(1450);
  //Servo Motor
  delay(500);
  turnLeft();
}