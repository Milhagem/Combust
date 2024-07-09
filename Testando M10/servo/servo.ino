#include <Servo.h>

#define SERVO 8

Servo s;
int pos;

void setup () {
  s.attach(SERVO);
  Serial.begin(9600);
  s.write(0);
}

void loop() {
  for(pos = 0; pos < 180; pos++) {
    s.write(pos);
  delay(15);
  }

  delay(1000);
  for(pos = 180; pos >= 0; pos--) {
    s.write(pos);
    delay(15);
  }
}
