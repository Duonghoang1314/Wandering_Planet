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

  // Set motor direction pins as outputs using fast macros
  FAST_PIN_MODE_OUTPUT(leftMotorDIR);
  FAST_PIN_MODE_OUTPUT(rightMotorDIR);

  SoftPWMBegin();
  SoftPWMSet(leftMotorPWM, 0);
  SoftPWMSet(rightMotorPWM, 0);
  SoftPWMSetFadeTime(leftMotorPWM, 0, 0);
  SoftPWMSetFadeTime(rightMotorPWM, 0, 0);

  // Initialize serial for debugging
  Serial.begin(9600);

  // --- Function ---
  Trace_to_Cross_T_Junction();
}

void loop() {}