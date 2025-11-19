#include "Variables.h"

// Variables (definitions)
const uint16_t thresholdS1 = 401;
const uint16_t thresholdS2 = 636;
const uint16_t thresholdS3 = 538;
const uint16_t thresholdS4 = 420;

// Store the threshold value
const uint16_t threshold[4] = {thresholdS1, thresholdS2, thresholdS3, thresholdS4};

const float Kp = 15.0f;    // Proportional gain
const float Ki = 0.0f;     // Integral gain
const float Kd = 8.0f;     // Derivative gain

const uint8_t baseSpeed = 25;
const uint8_t maxSpeed = 40;
const uint8_t minSpeed = 10;

uint16_t counter = 0;
// Store the sensor readings
uint16_t sensor[4];
bool s1, s2, s3, s4;

int8_t position = 0;
int16_t lastError = 0;
float integral = 0.0f;
unsigned long lastTime = 0;

const uint8_t turnSpeed = 30;
