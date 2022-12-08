#include "Motor.h"

#include "Wire.h"
#include "LiquidCrystal_I2C.h"

#define pedalGND  A8
#define pedalVcc A10
#define pinServo   8
#define freio    A15
#define switchSS A14

#define FALSE         0
#define TRUE          1
#define PRESSIONADO   0
#define LEDVERMELHO   2
#define timeInterval 200 //ms

#define stateSS_off       0
#define stateMonitoraVec  1
#define stateIncrementVec 2
#define stateDesligaMotor 3
#define stateLigaMotor    4
#define stateFreiando     5

Motor motor;
LiquidCrystal_I2C lcd(0x27,16,2);

int FSMstate = stateSS_off;
int valorInicial; 
float tensao;
int pos;
unsigned int timeOld;

void setup() {
  motor.setEstadoMotor(DESLIGADO);
  
  pinMode(pinLigaMotor,OUTPUT);
  pinMode(pinDesligaMotor,OUTPUT);
  pinMode(freio,INPUT_PULLUP);
  pinMode(switchSS,INPUT_PULLUP);
  pinMode(vecAtual,INPUT); 
  pinMode(LM2907,INPUT);

  digitalWrite(pinLigaMotor,LOW);
  digitalWrite(pinDesligaMotor,LOW);
  analogWrite(pedalGND, 0); // Pino 8 -> GND Pedal
  analogWrite(pedalVcc, 1023); // Pino 10 -> Vcc Pedal

  // Servo
  motor.servoAttach(pinServo);
  motor.servoWrite(0);  // Poe o servo na posicao inicial
  
  // Display LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Iniciando CR7");
  delay(500);

  Serial.begin(9600);
}


void loop() {
  switch (FSMstate)
  { 
    case stateSS_off:
      Serial.println("FSMstate = StartStop OFF");

      if(digitalRead(switchSS) == LOW && analogRead(vecAtual)>vecMin){
        FSMstate = stateMonitoraVec;
      }
    break;
  

    case stateMonitoraVec: 
      Serial.println("FSMstate = Monitora velocidade");

      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) == PRESSIONADO) {
          FSMstate = stateFreiando;
        }

        if(analogRead(vecAtual)<vecMax && analogRead(vecAtual)>ZEROvec &&
           digitalRead(freio) != PRESSIONADO) {
            if(motor.getEstadoMotor()==DESLIGADO)
              FSMstate = stateLigaMotor;
            else
              FSMstate = stateIncrementVec;
        } else if(analogRead(vecAtual)>vecMax && digitalRead(freio) != PRESSIONADO) {
          FSMstate = stateDesligaMotor;
        }
      } else { FSMstate = motor.desligaStartStop(); }

      
    break;
  

    case stateIncrementVec: 
      Serial.println("FSMstate = Incrementa Velocidade");

      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) != PRESSIONADO){
          if(pos <= 80) {
            pos+=10;
            motor.servoWrite(pos);

            FSMstate = stateMonitoraVec;
          }
        } else{
          FSMstate = stateFreiando;
        }
      } 
      else { FSMstate = motor.desligaStartStop(); }
       
    break;
  

    case stateDesligaMotor:
      Serial.println("FSMstate = Desliga Motor");
      motor.desligaMotor();
      pos+=0;
      motor.servoWrite(pos);
      FSMstate = stateMonitoraVec; 

    break;
  

    case stateLigaMotor:
      Serial.println("FSMstate = Liga Motor"); 

      if(digitalRead(switchSS) == LOW){
        if(digitalRead(freio) != PRESSIONADO) {
          motor.ligaMotor();
          FSMstate = stateMonitoraVec;
        } else if(digitalRead(freio) == PRESSIONADO){
          FSMstate = stateFreiando;
        }    
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateFreiando:
      Serial.println("FSMstate = Freio Pressionado"); 
      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) == PRESSIONADO){
          digitalWrite(LEDVERMELHO,HIGH);
          pos = 0;
          motor.servoWrite(pos);
        } else if(digitalRead(freio) != PRESSIONADO ){
          digitalWrite(LEDVERMELHO,LOW);
          FSMstate = stateMonitoraVec;
        }
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  
    default: FSMstate = stateSS_off;

  }

  if (millis() - timeOld >=timeInterval){ // Atualiza o display com Tensão e Velocidade. Taxa de atualização = Time interval
    
    lcd.setCursor(0,0);
    lcd.print("Tensao:         ");    // Exibir leitura LM
    lcd.setCursor(8,0);
    lcd.print(motor.analisaTensao()); 
    lcd.setCursor(0,1);
    lcd.print("Velocidade:     ");    // Exibir leitura velocidade
    lcd.setCursor(12,1);
    lcd.print(analogRead(vecAtual));
    
    timeOld = millis();
  }

  motor.setEstadoMotor(motor.checaEstadoMotor()); 
}
