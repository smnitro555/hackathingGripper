#include <Arduino.h>
#include "A4988.h"

#define echoPin 12 // Echo Pin
#define trigPin 13 // Trigger Pin
// using a 200-step motor (most common)
// pins used are DIR, STEP, MS1, MS2, MS3 in that order
A4988 stepper(200, 3, 2);
A4988 pincer_stepper(200, 9, 8);
// Pins Used for RGB
int redPin = 5; //this sets the red led pin
int greenPin = 6; //this sets the green led pin
int bluePin = 7; //this sets the blue led pin

// Variables for Arm Control
int arm1Position = 0;
int arm2Position = 0;
// Variables for Ultrasonic
int maximumRange = 400; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance
// Calibration Constants for Distance and RGB
int range = 2;
int tomatoHeight = 20;
int bookHeight = 10;
int waterBottleHeight = 15; 


void setup() {
    // RGB LED Initialization
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);  
    // Set target motor RPM to 1RPM
    stepper.setRPM(200);
    pincer_stepper.setRPM(200);
//     Set full speed mode (microstepping also works for smoother hand movement
//    stepper.setMicrostep(1);
    Serial.begin(9600);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    // Code to Reset the Gripper to Full Open Position
}

void loop() {
    // Measure Distance from Table
      distance = 0;
      digitalWrite(trigPin, LOW);  // Added this line
      delayMicroseconds(2); // Added this line
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10); // Added this line
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = (duration/2) / 29.1;
      // Change RGB Color Based on Distance
      if ((distance <= (tomatoHeight + range)) && (distance >= (tomatoHeight - range))) {
        setColor(255, 0, 0);  // red
      } else if ((distance <= (bookHeight + range)) && (distance >= (bookHeight - range))) {
        setColor(0, 255, 0);  // green
      } else if ((distance <= (waterBottleHeight + range)) && (distance >= (waterBottleHeight - range))) {
        setColor(0, 0, 255);  // blue
      } else {
        setColor(255, 255, 0);  // yellow
      }

    // Tell the Motor to Move based on Input
    if (Serial.available() > 0) {
      int byteIndex = 0;
      String inputData = "";
      arm1Position = Serial.parseInt();
      arm2Position = Serial.parseInt();

      Serial.print("I will move my arms by: (" + String(arm1Position) + ", " + String(arm2Position) + ")");
      Serial.println("");
      Serial.print("Current Distance from Table is: " + String(distance));
      Serial.println("");
      stepper.rotate(arm1Position);
      pincer_stepper.rotate(arm2Position);
    }
}

void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
