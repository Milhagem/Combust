#include "Servo.h"

Servo servo1;
#define pedalGND A8
#define pedal    A9
#define pedalVCC A10
#define pinServo 8


void setup() {
  servo1.attach(pinServo);
  
  pinMode(pedalGND, OUTPUT);
  pinMode(pedal, INPUT);
  pinMode(pedalVCC, OUTPUT);
  analogWrite(pedalGND, 0);    // Pino 8 -> GND 
  analogWrite(pedalVCC, 1023); // Pino 10 -> VCC
}

int angle = 0;

void loop() {
  // Leh o valor do Potenciometro no pedal
  angle = analogRead(pedal);
  
  // Mapeia o valor de 0 a 180 graus
  //angle = map(angle, 0, 1023, 0, 180);
  
  // Repassa o angulo ao ServoWrite
  servo1.write(map(angle, 0, 1023, 0, 180));
  
  delay(15);
}
