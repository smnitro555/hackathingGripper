/*
HC-SR04 Ping distance sensor]
VCC to arduino 5v GND to arduino GND
Echo to Arduino pin 13 Trig to Arduino pin 12
Red POS to Arduino pin 11
Green POS to Arduino pin 10
560 ohm resistor to both LED NEG and GRD power rail
More info at: http://goo.gl/kJ8Gl
Original code improvements to the Ping sketch sourced from Trollmaker.com
Some code and wiring inspired by http://en.wikiversity.org/wiki/User:Dstaub/robotcar
*/

#define trigPin 13
#define echoPin 12
#define led 11
#define led2 10

    // Pins Used for RGB
int redPin = 5; //this sets the red led pin
int greenPin = 6; //this sets the green led pin
int bluePin = 7; //this sets the blue led pin
    // Calibration Constants for Distance and RGB
int range = 2;
int tomatoHeight = 20;
int bookHeight = 10;
int waterBottleHeight = 15;

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
    // RGB LED Initialization
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT); 

}

void loop() {
  int duration, distance;
  distance = 0;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;

  // Change RGB Color Based on Distance
  if ((distance <= (tomatoHeight + range)) && (distance >= (tomatoHeight - range))) {
    setColor(255, 0, 0);  // red

  } else if ((distance <= (bookHeight + range)) && (distance >= (bookHeight - range))) {
    setColor(0, 255, 0);  // green
    Serial.println("Green");
  } else if ((distance <= (waterBottleHeight + range)) && (distance >= (waterBottleHeight - range))) {
    setColor(0, 0, 255);  // blue

  } else {
    setColor(80, 0, 80);  // purple
  }
  delay(500);
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
