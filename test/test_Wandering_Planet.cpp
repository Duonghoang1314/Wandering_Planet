// #include <Arduino.h>
// #include <unity.h>

// // Mock definitions for the pins and variables used in the code
// #define S1 A0
// #define S2 A1
// #define S3 A2
// #define S4 A3
// #define leftMotorDIR 2
// #define rightMotorDIR 3
// #define leftMotorPWM 5
// #define rightMotorPWM 6

// int sensor[4];
// bool s1, s2, s3, s4;
// int thresholdS1 = 500, thresholdS2 = 500, thresholdS3 = 500, thresholdS4 = 500;
// int threshold[4] = {thresholdS1, thresholdS2, thresholdS3, thresholdS4};
// int baseSpeed = 50, minSpeed = 30, maxSpeed = 100;
// int lastError = 0;
// unsigned long lastTime = 0;
// float Kp = 1.0, Ki = 0.0, Kd = 0.0;
// float integral = 0;

// // Mock functions for SoftPWM library
// void SoftPWMBegin() {}
// void SoftPWMSet(int pin, int value) {}
// void SoftPWMSetFadeTime(int pin, int fadeUp, int fadeDown) {}

// // Include the code to test
// #include "Wandering_Planet.cpp"

// // Test setup function
// void test_setup() {
//     setup();
//     TEST_ASSERT_EQUAL(INPUT, getPinMode(S1));
//     TEST_ASSERT_EQUAL(INPUT, getPinMode(S2));
//     TEST_ASSERT_EQUAL(INPUT, getPinMode(S3));
//     TEST_ASSERT_EQUAL(INPUT, getPinMode(S4));
//     TEST_ASSERT_EQUAL(OUTPUT, getPinMode(leftMotorDIR));
//     TEST_ASSERT_EQUAL(OUTPUT, getPinMode(rightMotorDIR));
// }

// // Test readSensors function
// void test_readSensors() {
//     sensor[0] = 400; // Below threshold
//     sensor[1] = 600; // Above threshold
//     sensor[2] = 400; // Below threshold
//     sensor[3] = 600; // Above threshold

//     readSensors();

//     TEST_ASSERT_TRUE(s1);
//     TEST_ASSERT_FALSE(s2);
//     TEST_ASSERT_TRUE(s3);
//     TEST_ASSERT_FALSE(s4);
// }

// // Test moveMotor function
// void test_moveMotor() {
//     moveMotor(50, -50, 1000);

//     TEST_ASSERT_EQUAL(HIGH, digitalRead(leftMotorDIR));
//     TEST_ASSERT_EQUAL(LOW, digitalRead(rightMotorDIR));
// }

// // Test calculateError function
// void test_calculateError() {
//     sensor[0] = 400; // Below threshold
//     sensor[1] = 600; // Above threshold
//     sensor[2] = 400; // Below threshold
//     sensor[3] = 600; // Above threshold

//     int error = calculateError();
//     TEST_ASSERT_EQUAL(0, error);
// }

// // Test computePID function
// void test_computePID() {
//     int error = 10;
//     float output = computePID(error);

//     TEST_ASSERT_FLOAT_WITHIN(0.1, Kp * error, output);
// }

// // Test followLine function
// void test_followLine() {
//     baseSpeed = 50;
//     minSpeed = 30;
//     maxSpeed = 100;

//     followLine();

//     TEST_ASSERT_TRUE(digitalRead(leftMotorDIR));
//     TEST_ASSERT_TRUE(digitalRead(rightMotorDIR));
// }

// // Test Trace_to_Cross_T_Junction function
// void test_Trace_to_Cross_T_Junction() {
//     s1 = false;
//     s4 = false;

//     Trace_to_Cross_T_Junction();

//     TEST_ASSERT_TRUE(s1);
//     TEST_ASSERT_TRUE(s4);
// }

// // Test turnLeft function
// void test_turnLeft() {
//     s1 = false;
//     s2 = false;

//     turnLeft();

//     TEST_ASSERT_TRUE(s1);
//     TEST_ASSERT_TRUE(s2);
// }

// // Test turnRight function
// void test_turnRight() {
//     s1 = false;
//     s2 = false;

//     turnRight();

//     TEST_ASSERT_TRUE(s1);
//     TEST_ASSERT_TRUE(s2);
// }

// void setup() {
//     UNITY_BEGIN();

//     RUN_TEST(test_setup);
//     RUN_TEST(test_readSensors);
//     RUN_TEST(test_moveMotor);
//     RUN_TEST(test_calculateError);
//     RUN_TEST(test_computePID);
//     RUN_TEST(test_followLine);
//     RUN_TEST(test_Trace_to_Cross_T_Junction);
//     RUN_TEST(test_turnLeft);
//     RUN_TEST(test_turnRight);

//     UNITY_END();
// }

// void loop() {
//     // Empty loop
// }