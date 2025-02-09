#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

RF24 radio(6, 7); // CE, CSN pins for the RF module
Servo ladderServo;  // Servo object for the ladder mechanism

// Define motor pins
const int motor1Pin1 = 2;
const int motor1Pin2 = 3;
const int motor2Pin1 = 4;
const int motor2Pin2 = 5;

// Define IR sensor pins
const int irFireDetectionPin = 43;  // IR Sensor for fire detection
const int irLineDetectionPin = 45;  // IR Sensor for line detection

// Define LED pins
const int greenLEDPin = 27;  // Green LED for line detection
const int redLEDPin = 26;   // Red LED for fire detection
const int blueLEDPin = 23;   // Blue LED for other indications
const int irFireLEDPin = 22;  // LED for fire detection indication

// Define servo pin
const int ladderServoPin = 9;  // Pin for the servo motor

// Define delays for waiting between detections and servo movement
const unsigned long detectionDelay1 = 350;
const unsigned long detectionDelay2 = 100;
const int servoDelay = 5;  // Delay between servo moves to slow down rotation

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  pinMode(irFireDetectionPin, INPUT);
  pinMode(irLineDetectionPin, INPUT);

  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  pinMode(irFireLEDPin, OUTPUT);

  digitalWrite(irFireLEDPin, LOW); // Ensure the fire indication LED is off at start

  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  const byte address[6] = "10110";
  radio.openReadingPipe(1, address);  
  radio.startListening();

  ladderServo.attach(ladderServoPin);
  ladderServo.write(135);  // Start with the ladder at neutral position (135 degrees)
}

void loop() {
  int fireSensorValue = digitalRead(irFireDetectionPin);

  if (fireSensorValue == HIGH) {
    digitalWrite(irFireLEDPin, LOW); // Turn on the fire indication LED when IR is detected
    moveServoGradually(0);  // Rotate servo 135 degrees counter-clockwise slowly
  } else {
    digitalWrite(irFireLEDPin, HIGH); // Turn off the fire indication LED when no IR is detected
    moveServoGradually(135);  // Rotate servo back to 135 degrees slowly
  }

     // Read values from IR sensors
   // Read values from IR sensors
  int lineSensorValue = digitalRead(irLineDetectionPin);
    // Check if both sensors detect a white line
  if (lineSensorValue == LOW) {
    // Stop the vehicle

    // Wait for a short period to check for another white line
    delay(detectionDelay1);

    // Read values from IR sensors again
    int lineSensorValue = digitalRead(irLineDetectionPin);

    // Check if only one sensor detects a white line
    if (lineSensorValue == HIGH) {
      // Blink green LED
      blinkGreenLED();
    } else if (lineSensorValue == LOW) {
      delay(detectionDelay1);
      int lineSensorValue = digitalRead(irLineDetectionPin);
      if (lineSensorValue == HIGH) {
        // Blink red LED
        blinkRedLED();
      }
    }
  }

  // Check if a large width white line is detected
  if (lineSensorValue == LOW) {
    delay(detectionDelay2);
    int lineSensorValue = digitalRead(irLineDetectionPin);

    // Check if only one sensor detects a white line
    if (lineSensorValue == LOW) {
      // Blink blue LED
      blinkBlueLED();
    }
  }
  if (radio.available()) {
    int command;
    radio.read(&command, sizeof(command));
    switch(command) {
      case 1: // Forward
        moveForward();
        Serial.println("Moving Forward");
        break;
      case 3: // Left
        turnLeft();
        Serial.println("Turning Left");
        break;
      case 2: // Right
        turnRight();
        Serial.println("Turning Right");
        break;
      case 4: // Backward
        moveBackward();
        Serial.println("Moving Backward");
        break;
      case 5: // stop
        stop();
        Serial.println("stop");
        break;
    }
  }
}

void moveServoGradually(int targetPosition) {
  int currentPosition = ladderServo.read();  // Read current servo position
  if (currentPosition < targetPosition) {
    for (int pos = currentPosition; pos <= targetPosition; pos++) {
      ladderServo.write(pos);
      delay(servoDelay);
    }
  } else if (currentPosition > targetPosition) {
    for (int pos = currentPosition; pos >= targetPosition; pos--) {
      ladderServo.write(pos);
      delay(servoDelay);
    }
  }
}

void blinkGreenLED() {
  digitalWrite(greenLEDPin, HIGH);
  delay(500);
  digitalWrite(greenLEDPin, LOW);
  delay(500);
}

void blinkRedLED() {
  digitalWrite(redLEDPin, HIGH);
  delay(500);
  digitalWrite(redLEDPin, LOW);
  delay(500);
}

void blinkBlueLED() {
  digitalWrite(blueLEDPin, HIGH);
  delay(500);
  digitalWrite(blueLEDPin, LOW);
  delay(500);
}

void moveForward() {
  digitalWrite(motor1Pin1,LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void moveBackward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1,  LOW);
  digitalWrite(motor2Pin2, HIGH);
}

void turnLeft() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  

}

void turnRight() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

}

void stop() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}
