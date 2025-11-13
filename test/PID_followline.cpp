#include <Arduino.h>
#include <SoftPWM.h>
#include "Variables.h"
#include "Function.h"

void followLine(){
  int error = calculateError();
  float correction = computePID(error);

  float leftSpeed = baseSpeed - correction;
  float rightSpeed = baseSpeed + correction;

  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  digitalWrite(leftMotorDIR, HIGH);
  digitalWrite(rightMotorDIR, HIGH);

  int leftPWM = map(abs(leftSpeed), 0, 100, 0, 255);
  int rightPWM = map(abs(rightSpeed), 0, 100, 0, 255);
  SoftPWMSet(leftMotorPWM, leftPWM);
  SoftPWMSet(rightMotorPWM, rightPWM);
}
int calculateError() {
  int weight[4] = {-3, -1, 1, 3}; // weighted sensor positions
  int sum = 0;
  int active = 0;

  for (int i = 0; i < 4; i++) {
    int state = sensor[i] < threshold[i] ? 1 : 0;  // 1 = line detected (black)
    sum += weight[i] * state;
    active += state;
  }

  if (active == 0) return 0;  // No line detected — return zero error

  float error = sum / active;
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