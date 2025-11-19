#include <Arduino.h>
#include <SoftPWM.h>
#include "Variables.h"
#include "Function.h"

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
  bool leftForward = leftSpeed > 0;
  bool rightForward = rightSpeed > 0;

  FAST_PIN_WRITE(leftMotorDIR, leftForward ? HIGH : LOW);
  FAST_PIN_WRITE(rightMotorDIR, rightForward ? HIGH : LOW);

  uint8_t leftPWM = map(abs(leftSpeed), 0, 100, 0, 255);
  uint8_t rightPWM = map(abs(rightSpeed), 0, 100, 0, 255);

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
}

void Trace_to_Right_Junction(){
  readSensors();
  while(true){
    readSensors();
    followLine();
    if (s4) break;
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

void followLine(){
  int error = calculateError();
  float correction = computePID(error);

  float leftSpeed = baseSpeed - correction;
  float rightSpeed = baseSpeed + correction;

  // Debug output so we can see followLine running and corrections applied
  Serial.print("followLine -> error: ");
  Serial.print(error);
  Serial.print(" correction: ");
  Serial.print(correction);
  Serial.print(" left: ");
  Serial.print(leftSpeed);
  Serial.print(" right: ");
  Serial.println(rightSpeed);

  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  // Set motor direction based on sign (same behavior as moveMotor)
  bool leftForward = leftSpeed > 0;
  bool rightForward = rightSpeed > 0;

  FAST_PIN_WRITE(leftMotorDIR, leftForward ? HIGH : LOW);
  FAST_PIN_WRITE(rightMotorDIR, rightForward ? HIGH : LOW);

  uint8_t leftPWM = map((int)abs(leftSpeed), 0, 100, 0, 255);
  uint8_t rightPWM = map((int)abs(rightSpeed), 0, 100, 0, 255);
  SoftPWMSet(leftMotorPWM, leftPWM);
  SoftPWMSet(rightMotorPWM, rightPWM);
}

int calculateError() {
  int weight[2] = {-1, 1}; // weighted sensor positions for 2 middle sensors
  int sum = 0;
  int active = 0;

  for (int i = 1; i <= 2; i++) { // Use sensors S2 and S3 (middle sensors)
    int state = sensor[i] < threshold[i] ? 1 : 0;  // 1 = line detected (black)
    sum += weight[i - 1] * state; // Adjust index for weight array
    active += state;
  }

  if (active == 0) {
    // No middle sensors detected the line. Fallback to outer sensors to
    // guess a direction so the robot doesn't simply drive straight.
    if (sensor[0] < threshold[0]) return -1; // left outer detects line
    if (sensor[3] < threshold[3]) return 1;  // right outer detects line
    // If nothing detected, reuse lastError to maintain previous correction
    return lastError;
  }

  float error = (float)sum / active;
  return error;
}

float computePID(float error) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  // Guard against division by zero on first call
  if (lastTime == 0 || dt == 0) {
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

void setLinetrackingBaseSpeed(int setBaseSpeed, int setMinSpeed, int setMaxSpeed){
  baseSpeed = setBaseSpeed;
  minSpeed = setMinSpeed;
  maxSpeed = setMaxSpeed;
}
