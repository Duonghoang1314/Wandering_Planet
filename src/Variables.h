#pragma once

// --- Pin Definitions ---
#define S1 A0 //Left outer sensor
#define S2 A1 //Left outer sensor
#define S3 A2 //Left outer sensor
#define S4 A3 //Left outer sensor

#define Servo1 9 // Rotate Servo
#define Servo2 10// Handle Servo

#define leftMotorPWM 8 //  Left Motor
#define leftMotorDIR 0 //  Left Motor
#define rightMotorPWM 6 // Left Motor
#define rightMotorDIR 1 // Left Motor

// Variables (declare only)
extern int thresholdS1;
extern int thresholdS2;
extern int thresholdS3;
extern int thresholdS4;

//Store the threshold value
extern int threshold[4];

extern float Kp;    // Proportional gain
extern float Ki;     // Integral gain
extern float Kd;     // Derivative gain

extern int baseSpeed;
extern int maxSpeed;
extern int minSpeed;

extern int counter;
//Store the sensor readings
extern int sensor[4];
extern bool s1, s2, s3, s4;

extern int position;
extern int lastError;
extern float integral;
extern unsigned long lastTime;

extern int turnSpeed;