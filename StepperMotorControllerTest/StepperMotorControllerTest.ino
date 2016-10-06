#include <AccelStepper.h>
#include <MultiStepper.h>

// Run a A4998 Stepstick from an Arduino UNOusing AccelStepper
// Paul Hurley Aug 2015

AccelStepper stepper(1,3,4);//initialise accelstepper for a two wire board, pin 5 step, pin 4 dir

void setup() {
  Serial.begin(9600);
  pinMode(2,OUTPUT); // Enable
  digitalWrite(2,LOW); // Set Enable low
  stepper.setMaxSpeed(10);
  stepper.setSpeed(5);
  stepper.runSpeed();
}

void loop() {
//  digitalWrite(2,LOW); // Set Enable low
//  if (stepper.distanceToGo() == 0)
//  {  // Random change to speed, position and acceleration
//    // Make sure we dont get 0 speed or accelerations
//    delay(1000);
//    stepper.moveTo(100);
//    stepper.setMaxSpeed(100);
//    stepper.setAcceleration(10);
//  }
//
//Serial.println(stepper.distanceToGo());
//stepper.run();  // Actually makes stepper move
}
