#include "Servo.h"
#include "Wire.h"
#include "LiquidCrystal_I2C.h"

Servo servo1;
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2);


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

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Testando servo");
}

int tensaoPedal = 0;
int angle = 0;

void loop() {
  // Leh o valor do Potenciometro no pedal
  tensaoPedal = analogRead(pedal);
  
  // Mapei o valor do pedal para o angulo
  angle = map(tensaoPedal, 0, 1023, 0, 180);
  
  // Repassa o angulo ao ServoWrite
  servo1.write(angle);

  // Exibir leitura da posicao do Servo
  lcd.setCursor(0,0);
  lcd.print("pos:    ");
  lcd.setCursor(5,0);
  lcd.print(angle);

  
  delay(15);
}
