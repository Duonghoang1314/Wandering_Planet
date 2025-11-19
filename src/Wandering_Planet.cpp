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

  pinMode(13, INPUT_PULLUP);
  pinMode(leftMotorDIR, OUTPUT);
  pinMode(rightMotorDIR, OUTPUT);

  SoftPWMBegin();
  SoftPWMSet(leftMotorPWM, 0);
  SoftPWMSet(rightMotorPWM, 0);
  SoftPWMSetFadeTime(leftMotorPWM, 0, 0);
  SoftPWMSetFadeTime(rightMotorPWM, 0, 0);

  Serial.begin(9600);
  // Wait for pressing the button to start
  while (digitalRead(13) == HIGH){}
  
  // --- Function ---
  moveMotor(30, 25, 400);
  Trace_to_Cross_T_Junction();
  moveMotor(30, 25, 300);

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  turnRight();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 37, 100);
  turnLeft();

  Trace_to_Left_Junction();
  moveMotor(40, 33, 100);
  turnLeft();

  Trace_to_Right_Junction();
  moveMotor(40, 33, 100);
  turnRight();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  turnRight();

  // --- Mission 1 ---
  Trace_to_Cross_T_Junction();
  moveMotor(-40, -33, 300);
  turnRight();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  
  // --- Mission 2 ---
  Trace_to_Cross_T_Junction();
  moveMotor(-40, -33, 300);
  turnRight();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  turnLeft();

  // --- Mission 3 ---
  Trace_to_Cross_T_Junction();
  moveMotor(-40, -33, 300);
  turnRight();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  turnRight();

  Trace_to_Left_Junction();
  moveMotor(40, 33, 100);
  turnLeft();

  // --- Mission 4 ---
  Trace_to_Left_Junction();
  moveMotor(40, 33, 100);
  turnRight();
  turnRight();

  Trace_to_Right_Junction();
  moveMotor(40, 33, 100);

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  turnRight();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  turnLeft();

  // --- Mission 5 ---
  Trace_to_Cross_T_Junction();
  moveMotor(-40, -37, 300);
  turnRight();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);

  Trace_to_Right_Junction();
  moveMotor(40, 33, 100);

  // --- Mission 6 ---
  Trace_to_Cross_T_Junction();
  moveMotor(-40, -37, 300);
  turnLeft();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  turnLeft();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  turnRight();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  turnRight();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  turnRight();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 100);
  turnLeft();

  Trace_to_Cross_T_Junction();
  moveMotor(40, 33, 200);
  moveMotor(0, 0, 0);
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

  Serial.print(s1);
  Serial.print(s2);
  Serial.print(s3);
  Serial.println(s4);
}
// --- Motor control function ---
void moveMotor(int leftSpeed, int rightSpeed, int duration){
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

// --- Trace to Cross T Junction Functions ---
// Read sensors continuously until both s1 and s4 are true(indicating a cross junction)
// Then exit the function
void Trace_to_Cross_T_Junction(){
  readSensors();
  while(true){
    readSensors();
    if (s1 && s4) break;
    followLine();
  }
  moveMotor(0, 0, 0);
}
// Trace to Right T Junction function
void Trace_to_Right_Junction(){
  readSensors();
  while(true){
    readSensors();
    followLine();
    if (s4) break;
  }
  moveMotor(0, 0, 0);
}
// Trace to Left T Junction function
void Trace_to_Left_Junction(){
  readSensors();
  while(true){
    readSensors();
    if (s1) break;
    followLine();
  }
  moveMotor(0, 0, 0);
}
// Turn left function
void turnLeft(){
  readSensors();
  while(!s1){
    readSensors();
    moveMotor(turnSpeed + 20, -turnSpeed, 10);
  }
  while(!s2){
    readSensors();
    moveMotor(turnSpeed, -turnSpeed, 10);
  }
}
// Turn right function
void turnRight(){
  readSensors();
  while(!s4){
    readSensors();
    moveMotor(-20 - turnSpeed, turnSpeed, 10);
  }
  while(!s3){
    readSensors();
    moveMotor(- turnSpeed, turnSpeed, 10);
  }
}

void EnergyDetection(){
}

// --- Line tracking function with PID control ---
void followLine(){
  int error = calculateError();
  float correction = computePID(error);

  float leftSpeed = baseSpeed - correction;
  float rightSpeed = baseSpeed + correction;

  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
  
  // Set motor direction based on sign (same behavior as moveMotor)
  bool leftForward = leftSpeed > 0;
  bool rightForward = rightSpeed > 0;

  digitalWrite(leftMotorDIR, leftForward ? HIGH : LOW);
  digitalWrite(rightMotorDIR, rightForward ? HIGH : LOW);

  int leftPWM = map(abs(leftSpeed), 0, 100, 0, 255);
  int rightPWM = map(abs(rightSpeed), 0, 100, 0, 255);
  SoftPWMSet(leftMotorPWM, leftPWM);
  SoftPWMSet(rightMotorPWM, rightPWM);
}
// --- PID Control Logic ---
int calculateError() {
  if (s2 && !s3) return 1;     // line is left
  if (!s2 && s3) return -1;     // line is right
  if (s2 && s3)  return 0;      // centered

  return lastError;
}
// Compute PID output based on the current error
float computePID(float error) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  // Guard against division by zero on first call
  if (lastTime == 0 || dt <= 0) {
    lastTime = now;
    lastError = error;
    return 0.0;
  }
  lastTime = now;

  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;

  lastError = error;
  return output;
}
// --- Set base, min, max speed for line tracking ---
void setLinetrackingBaseSpeed(int setBaseSpeed, int setMinSpeed, int setMaxSpeed){
  baseSpeed = setBaseSpeed;
  minSpeed = setMinSpeed;
  maxSpeed = setMaxSpeed;
}