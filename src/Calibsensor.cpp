#include <Arduino.h>

// Pin definitions
const int buttonPin = 13;
const int irSensorPins[] = {A0, A1, A2, A3};

// Variables to store sensor values
int whiteValues[4] = {0, 0, 0, 0};
int blackValues[4] = {0, 0, 0, 0};
int thresholdValues[4] = {0, 0, 0, 0};

// State variables
bool isCalibratingWhite = false;
bool isCalibratingBlack = false;

void readSensorValues(int *values);

void setup() {
    pinMode(buttonPin, INPUT_PULLUP); // Button with pull-up resistor
    for (int i = 0; i < 4; i++) {
        pinMode(irSensorPins[i], INPUT);
    }
    Serial.begin(9600);
    Serial.println("Press the button to start calibration.");
}

void loop() {
    while (!Serial); // Wait for Serial connection
    static bool lastButtonState = HIGH;
    bool currentButtonState = digitalRead(buttonPin);

    // Detect button press
    if (lastButtonState == HIGH && currentButtonState == LOW) {
        if (!isCalibratingWhite && !isCalibratingBlack) {
            isCalibratingWhite = true;
            Serial.println("Calibrating white values...");
            readSensorValues(whiteValues);
            Serial.println("White values calibrated.");
        } else if (isCalibratingWhite && !isCalibratingBlack) {
            isCalibratingBlack = true;
            Serial.println("Calibrating black values...");
            readSensorValues(blackValues);
            Serial.println("Black values calibrated.");
            Serial.println("Calibration complete.");

            // Calculate and print threshold values
            Serial.println("White, Black, and Threshold values:");
            for (int i = 0; i < 4; i++) {
                thresholdValues[i] = (whiteValues[i] + blackValues[i]) / 2;
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.print(" - White: ");
                Serial.print(whiteValues[i]);
                Serial.print(", Black: ");
                Serial.print(blackValues[i]);
                Serial.print(", Threshold: ");
                Serial.println(thresholdValues[i]);
            }
        }
    }

    lastButtonState = currentButtonState;
}

// Function to read sensor values
void readSensorValues(int *values) {
    for (int i = 0; i < 4; i++) {
        values[i] = analogRead(irSensorPins[i]);
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(values[i]);
    }
}