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

// Variables
int thresholdS1 = 405;
int thresholdS2 = 568;
int thresholdS3 = 596;
int thresholdS4 = 389;

//Store the threshold value
int threshold[4] = {thresholdS1, thresholdS2, thresholdS3, thresholdS4};

float Kp = 15.0;    // Proportional gain
float Ki = 0.0;     // Integral gain
float Kd = 8.0;     // Derivative gain

int baseSpeed = 25;
int maxSpeed = 40;
int minSpeed = 10;

int counter = 0;
//Store the sensor readings
int sensor[4];
bool s1, s2, s3, s4;

int position = 0;
int lastError = 0;
float integral = 0;
unsigned long lastTime = 0;
