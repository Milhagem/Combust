#include "Motor.h"
#include "Display.h"

/**
 * PROBLEMA NA LOGICA:
 * O carro nunca vai chegar a velocidade maxima pelo startstop
 * 
 * @camposouza 08/12
*/


#define pedalGND  A8
#define pedalVcc A10
#define pinServo   8
#define freio    A15
#define switchSS A14
#define ledRed     2

#define PRESSIONADO   0

#define stateSS_off       0
#define stateMonitoraVec  1
#define stateIncrementVec 2
#define stateDesligaMotor 3
#define stateLigaMotor    4
#define stateFreiando     5

Motor motor;
Display display;

int FSMstate = stateSS_off;
int pos;

void setup() {
  motor.setEstadoMotor(DESLIGADO);
  
  pinMode(pinLigaMotor, OUTPUT);
  pinMode(pinDesligaMotor, OUTPUT);
  pinMode(freio, INPUT_PULLUP);
  pinMode(switchSS, INPUT_PULLUP);
  pinMode(vecAtual, INPUT); 
  pinMode(LM2907, INPUT);

  digitalWrite(pinLigaMotor, LOW);
  digitalWrite(pinDesligaMotor, LOW);
  analogWrite(pedalGND, 0); // Pino 8 -> GND Pedal
  analogWrite(pedalVcc, 1023); // Pino 10 -> Vcc Pedal

  // Servo
  motor.servoAttach(pinServo);
  motor.servoWrite(0);  // Poe o servo na posicao inicial
  
  // Display LCD
  display.iniciaDisplay();

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

        if(analogRead(vecAtual)<vecMin && analogRead(vecAtual)>ZEROvec &&
           digitalRead(freio) != PRESSIONADO) {
            if(motor.getEstadoMotor() == DESLIGADO) {
              FSMstate = stateLigaMotor;
            } else { 
              FSMstate = stateIncrementVec;
            }
        } else if(analogRead(vecAtual)>vecMax && 
                  digitalRead(freio) != PRESSIONADO) {
          FSMstate = stateDesligaMotor;
        } 

      } else { FSMstate = motor.desligaStartStop(); }

      
    break;
  

    case stateIncrementVec: 
      Serial.println("FSMstate = Incrementa Velocidade");

      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) == PRESSIONADO){
          FSMstate = stateFreiando;
        }
      
        if(pos <= 80) {
          pos+=10;
          motor.servoWrite(pos);

          FSMstate = stateMonitoraVec;
        }

      } else { FSMstate = motor.desligaStartStop(); }
       
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
        if(digitalRead(freio) == PRESSIONADO) {
          FSMstate = stateFreiando;
        }

        if(digitalRead(freio) != PRESSIONADO) {
          motor.ligaMotor();
          FSMstate = stateMonitoraVec;
        }
          
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateFreiando:
      Serial.println("FSMstate = Freio Pressionado"); 

      if(digitalRead(switchSS) == LOW) {
        digitalWrite(ledRed,HIGH);
        pos = 0;
        motor.servoWrite(pos);

        if(digitalRead(freio) != PRESSIONADO ){
          digitalWrite(ledRed,LOW);
          FSMstate = stateMonitoraVec;
        }
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  
    default: FSMstate = stateSS_off;
    break;

  }

  display.atualizaDisplay(motor);

  motor.setEstadoMotor(motor.checaEstadoMotor()); 
}
