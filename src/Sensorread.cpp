// #include <Arduino.h>

// void setup() {
//     // Initialize serial communication at 9600 baud rate
//     Serial.begin(9600);

//     // Configure analog pins A0 to A3 as input
//     for (int pin = A0; pin <= A3; pin++) {
//         pinMode(pin, INPUT);
//     }
// }

// void loop() {
//     // Read values from IR sensors on pins A0 to A3
//     int sensorValues[4];
//     for (int i = 0; i < 4; i++) {
//         sensorValues[i] = analogRead(A0 + i);
//     }

//     // Print sensor values to the Serial Monitor
//     Serial.print("IR Sensor Values: ");
//     for (int i = 0; i < 4; i++) {
//         Serial.print(sensorValues[i]);
//         if (i < 3) {
//             Serial.print(", ");
//         }
//     }
//     Serial.println();

//     // Wait for 100ms
//     delay(100);
// }