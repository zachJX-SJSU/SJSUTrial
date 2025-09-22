#include <Servo.h>

Servo servo1;
int SERV_PIN1 = 9;
int angle = 0;

void setup() {
  // put your setup code here, to run once:
  servo1.attach(SERV_PIN1);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  while (angle < 180) {
    Serial.println(angle);
    servo1.write(angle);
    angle += 10;
    
    delay(100);
  }
  while (angle > 0) {
    servo1.write(angle);
    angle -= 10;
    delay(100);
  }
}
