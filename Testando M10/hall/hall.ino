#include <Arduino.h>
#include <stdlib.h>
#include <Wire.h>

#define hall_motor DD2

void setup(){
  Serial.begin(9600);
}

void loop(){
  float ampOpOUT = digitalRead(hall_motor);

  Serial.print("ampOpOUT:");
  Serial.println(ampOpOUT);
}