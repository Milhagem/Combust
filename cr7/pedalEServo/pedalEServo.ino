#include "Servo.h"

Servo servo1;
#define pedalGND A8
#define pedal A9
#define pedalVCC A10

void setup() {
  // Anexa o Servo ao Pin5
  servo1.attach(8);
  
  pinMode(pedalGND, OUTPUT);
  pinMode(pedal, INPUT);
  pinMode(pedalVCC, OUTPUT);
  analogWrite(pedalGND, 0); // Pino 8 -> GND 
  analogWrite(pedalVCC, 1023); // Pino 10 -> VCC
}

void loop() {
  // Lê o valor do Potenciometro
  int angle = analogRead(9);
  
  // Mapeia o valor de 0 a 180 graus
  angle = map(angle, 0, 1023, 0, 180);
  
  // Repassa o angulo ao ServoWrite
  servo1.write(angle);
  
  delay(15);
}
