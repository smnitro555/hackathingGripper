#include <Arduino.h>
#include "A4988.h"

#define echoPin 12 // Echo Pin
#define trigPin 13 // Trigger Pin
// using a 200-step motor (most common)
// pins used are DIR, STEP, MS1, MS2, MS3 in that order
A4988 pincer_stepper(200, 3, 2);
A4988 stepper(200, 9, 8);
// Pins Used for RGB
int redPin = 5; //this sets the red led pin
int greenPin = 6; //this sets the green led pin
int bluePin = 7; //this sets the blue led pin
// Force Sensivity Readings
int fsrAnalogPin1 = 0; // FSR is connected to analog 0
int fsrAnalogPin2 = 1; // FSR is connected to analog 1
int fsrAnalogPin3 = 2; // FSR is connected to analog 2
int fsrReading1, fsrReading2, fsrReading3;
// Variables for Arm Control
int arm1Position, arm2Position;
// Variables for Ultrasonic
int maximumRange = 400; // Maximum range needed
int minimumRange = 0; // Minimum range needed
//long duration, distance; // Duration used to calculate distance
float duration, distance;
int mode = 0;
int modeDrop = 0;
int triggerDrop = false;
// Calibration Constants for Distance and RGB
int range = 0;
int tomatoHeight = 20;
int bookHeight = 18;
int waterBottleHeight = 22;
// Limit Switch Set-Ups
int limitPincerPin = 38;
int limitUpperBarPin = 36;
// Trigger Input
int triggerInputPin = 50;



void setup() {
  // RGB LED Initialization
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Limit Switch Initialization
  pinMode(limitPincerPin, INPUT);
  pinMode(limitUpperBarPin, INPUT);

  // Input Signal Initialization
  pinMode(triggerInputPin, INPUT);

  // Set target motor RPM to 1RPM
  stepper.setRPM(200);
  pincer_stepper.setRPM(200);

  // Ultrasonic Initialization
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
  // Code to Reset the Gripper to Full Open Position
  resetGripper();
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
  distance = (duration / 2) / 29.1;


  // Change RGB Color Based on Distance
  Serial.println("Distance:" + String(distance));
  Serial.println(distance <= (tomatoHeight + range));
  Serial.println(distance >= (tomatoHeight - range));
  if ((int(distance) <= (tomatoHeight + range)) && (int(distance) >= (tomatoHeight - range))) {
    setColor(255, 0, 0);  // red
    mode = 0; // Tomato Mode
    Serial.println("i ran");
  } else if ((int(distance) <= (waterBottleHeight + range)) && (int(distance) >= (waterBottleHeight - range))) {
    setColor(0, 0, 255);  // blue
    mode = 1; // Water Bottle Mode
  } else if ((int(distance) <= (bookHeight + range)) && (int(distance) >= (bookHeight - range))) {
    setColor(0, 255, 0);  // green
    mode = 2; // Book Mode
  } else {
    setColor(165, 42, 42);  // purple
  }

  //  mode = 0;
//  Serial.println("Distance " + String(distance));

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

  // Approach to the object.
  if (digitalRead(triggerInputPin) == HIGH && triggerDrop == false) {
    // Gripper routines
    if (mode == 0) {
      // Tomato Routine
      stepper.rotate(2000);
      pincer_stepper.rotate(1500);
      stepper.rotate(800);
      pincer_stepper.rotate(600);
    } else if (mode == 1) {
      // Water Bottle Routine
      stepper.rotate(2500);
      pincer_stepper.rotate(2500);
      stepper.rotate(1000);
    } else if (mode == 2) {
      // Book Mode

    } else if (mode == 3) {
      // Water Bottle Mode

    }
    triggerDrop = true;
    modeDrop = mode;
  }


  //  Serial.println(digitalRead(triggerInputPin) == HIGH);
  if (digitalRead(triggerInputPin) == HIGH && triggerDrop == true) {
    // Gripper routines
    if (modeDrop == 0) {
      pincer_stepper.rotate(-1000);
    } else if (modeDrop == 1) {
      pincer_stepper.rotate(-1000);
    } else if (modeDrop == 2) {
      // Book Drop Routine Here

    } else if (modeDrop == 3) {
      // Water Bottle Drop Routine Here

    }
    triggerDrop = false;
    modeDrop = mode;
    resetGripper();
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

void resetGripper() {
  while (digitalRead(limitUpperBarPin) == LOW) {
    pincer_stepper.rotate(-20);
  }

  while (digitalRead(limitPincerPin) == LOW) {
    stepper.rotate(-20);
  }

  Serial.println("exit reset");
}


