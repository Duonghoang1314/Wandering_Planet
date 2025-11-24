// --- Pin Definitions ---
#define S1 A0 //Left outer sensor
#define S2 A1 //Left outer sensor
#define S3 A2 //Left outer sensor
#define S4 A3 //Left outer sensor

#define leftMotorPWM 8 //  Left Motor
#define leftMotorDIR 0 //  Left Motor
#define rightMotorPWM 6 // Left Motor
#define rightMotorDIR 1 // Left Motor

// Threshold values for line tracking sensors in white side
int thresholdS1 = 680;
int thresholdS2 = 743;
int thresholdS3 = 575;
int thresholdS4 = 575;

// Threshold values for line tracking sensors in black side
int thresholdS1B = 783;
int thresholdS2B = 835;
int thresholdS3B = 647;
int thresholdS4B = 632;

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

int turnSpeed = 40;