#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(6, 7); // CE, CSN

const int joystickXPin = A0;
const int joystickYPin = A1;

int xValue;
int yValue;

int joystickXValue;
int joystickYValue;
#define joyX A0

#define joyY A1
 
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  const byte address[6] = "10110"; // Address for communication
  radio.openWritingPipe(address);
}

void loop() {
int data=0;

xValue = analogRead(joyX);
  yValue = analogRead(joyY);
  if(xValue == 1023 && yValue < 600){
    Serial.println("F");
    data = 1;
      radio.write(&data, sizeof(data));

  }
  if(xValue <10 && yValue < 600){
    Serial.println("B");
    data =4;
      radio.write(&data, sizeof(data));

  }
  if(xValue <600 && yValue ==1023){
    Serial.println("R");
      data=3;

      radio.write(&data, sizeof(data));
  }
  if(xValue <600 && yValue < 10){
    Serial.println("L"); 
      data =2;

      radio.write(&data, sizeof(data));
  }
  if (xValue >= 525 && xValue <= 532 && yValue >= 525 && yValue <= 532) {
    Serial.println("S");
    data = 5;

    radio.write(&data, sizeof(data));
  }
  
  //Serial.println(data[0]);

 // Adjust as needed
}