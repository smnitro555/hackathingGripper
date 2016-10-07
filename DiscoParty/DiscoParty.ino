#include <Arduino.h>
#include "A4988.h"



#define echoPin 12 // Echo Pin
#define trigPin 13 // Trigger Pin
// using a 200-step motor (most common)
// pins used are DIR, STEP, MS1, MS2, MS3 in that order
A4988 stepper(200, 3, 2);
A4988 pincer_stepper(200, 9, 8);

// Variables for Arm Control
int arm1Position = 0;
int arm2Position = 0;
// Variables for Ultrasonic
int ops = 0; // Counter so we dont Check Distance Every Cycle
int ops2check = 50; // Cycles before Distance Refresh
int maximumRange = 400; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance


void setup() {
    // Set target motor RPM to 1RPM
    stepper.setRPM(200);
    pincer_stepper.setRPM(200);
//     Set full speed mode (microstepping also works for smoother hand movement
//    stepper.setMicrostep(1);
    Serial.begin(9600);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void loop() {
    // Measure Distance from Table
    if (ops == ops2check) {
      digitalWrite(trigPin, LOW); 
      delayMicroseconds(2); 
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10); 
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      //Calculate the distance (in cm) based on the speed of sound.
      distance = duration/58.2;
    }

    // Tell the Motor to Move based on Input
    if (Serial.available() > 0) {
      int byteIndex = 0;
      String inputData = "";
      arm1Position = Serial.parseInt();
      arm2Position = Serial.parseInt();

      Serial.print("I will move my arms by: (" + String(arm1Position) + ", " + String(arm2Position) + ")");
      Serial.println("");
      if (ops == 0) {
        Serial.print("Current Distance from Table is: " + String(distance));
        Serial.println("")
        // Change RGB Color Based on Distance
      }
      stepper.rotate(arm1Position);
      pincer_stepper.rotate(arm2Position);
    }

    ops = ops + 1;
}
