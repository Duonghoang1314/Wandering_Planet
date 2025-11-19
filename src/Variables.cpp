#include "Variables.h"

// Variables (definitions)
int thresholdS1 = 401;
int thresholdS2 = 636;
int thresholdS3 = 538;
int thresholdS4 = 420;

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

int turnSpeed = 30;
