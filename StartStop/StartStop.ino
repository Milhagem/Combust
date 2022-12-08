#include "Motor.h"
#include "Display.h"


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

      if(digitalRead(switchSS) == LOW && analogRead(vecAtual)>ZEROvec){
        FSMstate = stateMonitoraVec;
      }
    break;
  

    case stateMonitoraVec: 

      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) == PRESSIONADO) {
          FSMstate = stateFreiando;
        }

        if(analogRead(vecAtual)<vecMin && analogRead(vecAtual)>ZEROvec) {
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

      if(digitalRead(switchSS) == LOW) {

        while(analogRead(vecAtual)<=vecMax && pos <= 80) {
          if(digitalRead(freio) == PRESSIONADO){
          FSMstate = stateFreiando;
          }

          pos += 5;
          motor.servoWrite(pos);
        }

        FSMstate = stateMonitoraVec;

      } else { FSMstate = motor.desligaStartStop(); }
       
    break;
  

    case stateDesligaMotor:

      if(digitalRead(switchSS) == LOW){
        motor.desligaMotor();
        FSMstate = stateMonitoraVec;
        
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateLigaMotor:

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

  display.atualizaDisplay(motor, FSMstate);

  motor.setEstadoMotor(motor.checaEstadoMotor()); 
}
