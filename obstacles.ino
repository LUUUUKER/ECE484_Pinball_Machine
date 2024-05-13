#include <Servo.h>

// Constants for the servo and button pins
const int servoPin1 = 9;            // the pin where the servo is connected
const int servoPin2 = 10;
const int buttonCWPin = 6;         // the pin where the CW button is connected
const int buttonCCWPin = 3;        // the pin where the CCW button is connected
const int buttonLeftPin = 4;         // the pin where the CW button is connected
const int buttonRightPin = 5;        // the pin where the CCW button is connected
int current1 = 90;
int current2 = 90;

// Create a Servo object
Servo myServo1;
Servo myServo2;

void setup() {
  // Attach the Servo to the servo pin
  myServo1.attach(servoPin1);
  myServo2.attach(servoPin2);

  // Initialize the button pins as input
  pinMode(buttonCWPin, INPUT);
  pinMode(buttonCCWPin, INPUT);
  pinMode(buttonLeftPin, INPUT);
  pinMode(buttonRightPin, INPUT);
  
  // Initialize the servo to a neutral position, if needed
  myServo1.write(current1);
  myServo2.write(current2);
}

void loop() {
  // Check if the CW button is pressed (HIGH because of the pull-down resistor)
  if (digitalRead(buttonCWPin) == HIGH) {
    if (current1 >= 0 && current1 < 180) {
      current1 = current1 + 2;
      myServo1.write(current1);  // Rotate servo to 180 degrees
      delay(10);          // Add a delay to debounce the button press
    } else {
      current1 = 180;
      myServo1.write(current1);
      delay(10);
    }
    
  }

  // Check if the CCW button is pressed (HIGH because of the pull-down resistor)
  if (digitalRead(buttonCCWPin) == HIGH) {
    if (current1 >= 0 && current1 < 180) {
      current1 = current1 - 2;
      myServo1.write(current1);  // Rotate servo to 180 degrees
      delay(10);          // Add a delay to debounce the button press
    } else {
      current1 = 0;
      myServo1.write(current1);
      delay(10);
    }
  }

  // Check if the left button is pressed (HIGH because of the pull-down resistor)
  if (digitalRead(buttonLeftPin) == HIGH) {
    if (current2 >= 0 && current2 < 180) {
      current2 = current2 + 2;
      myServo2.write(current2);  // Rotate servo to 180 degrees
      delay(10);          // Add a delay to debounce the button press
    } else {
      current2 = 180;
      myServo2.write(current2);
      delay(10);
    }
    
  }

  // Check if the CCW button is pressed (HIGH because of the pull-down resistor)
  if (digitalRead(buttonRightPin) == HIGH) {
    if (current2 > 0 && current2 <= 180) {
      current2 = current2 - 2;
      myServo2.write(current2);  // Rotate servo to 180 degrees
      delay(10);          // Add a delay to debounce the button press
    } else {
      current2 = 0;
      myServo2.write(current2);
      delay(10);
    }
  }
}

