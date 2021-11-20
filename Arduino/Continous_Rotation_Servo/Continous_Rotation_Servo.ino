// Testing the modified continous rotation servos

#include <Servo.h>

Servo servo1;
const int servo1Pin = 5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo1.attach(servo1Pin);
}

void loop() {
  // put your main code here, to run repeatedly:
  servo1.write(160);
  delay(10000);
  servo1.write(20);  delay(10000);
}
