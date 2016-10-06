#include <Arduino.h>
#include "A4988.h"

// using a 200-step motor (most common)
// pins used are DIR, STEP, MS1, MS2, MS3 in that order
A4988 stepper(200, 4, 3);
A4988 pincer_stepper(200, 9, 5);

int arm1Position = 0;
int arm2Position = 0;

void setup() {
    // Set target motor RPM to 1RPM
    stepper.setRPM(200);
    pincer_stepper.setRPM(200);
//     Set full speed mode (microstepping also works for smoother hand movement
//    stepper.setMicrostep(1);
    Serial.begin(9600);
}

void loop() {
    // Tell motor to rotate 360 degrees. That's it.

    if (Serial.available() > 0) {
      int byteIndex = 0;
      String inputData = "";
      arm1Position = Serial.parseInt();
      arm2Position = Serial.parseInt();

      Serial.print("I will move my arms by: (" + String(arm1Position) + ", " + String(arm2Position) + ")");
      Serial.println("");

      stepper.rotate(arm1Position);
      pincer_stepper.rotate(arm2Position);
    }

}
