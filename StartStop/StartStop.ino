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
#define stateSS_on        1
#define stateMonitoraVel  2
#define stateIncrementVel 3
#define stateDesligaMotor 4
#define stateLigaMotor    5
#define stateFreiando     6


Motor motor;
Display display;

int FSMstate = stateSS_off;
int pos;
long int time_ac = 0;

void setup() {
  motor.setEstadoMotor(DESLIGADO);
  
  pinMode(pinLigaMotor, OUTPUT);
  pinMode(pinDesligaMotor, OUTPUT);
  pinMode(freio, INPUT_PULLUP);
  pinMode(switchSS, INPUT_PULLUP);
  pinMode(VelAtual, INPUT); 
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

  //Serial.begin(9600);
  
}


void loop() {
  switch (FSMstate)
  { 
    case stateSS_off:
      if(digitalRead(switchSS) == LOW && analogRead(VelAtual)<ZEROVel){
          FSMstate = stateSS_on;
        }else{
          if(digitalRead(switchSS) == LOW && analogRead(VelAtual) > ZEROVel){
            FSMstate = stateMonitoraVel;
          }
        }

    break;
  
  case stateSS_on:
    if(analogRead(VelAtual) > ZEROVel){
        FSMstate = stateMonitoraVel;
    }
    if(digitalRead(switchSS) == HIGH){
      FSMstate = stateSS_off;
    } 
    break;

    case stateMonitoraVel: 

      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) == PRESSIONADO) {
          FSMstate = stateFreiando;
        }

        if(analogRead(VelAtual)<VelMin && analogRead(VelAtual)>ZEROVel) {
            if(motor.checaEstadoMotor() == DESLIGADO) {
              FSMstate = stateLigaMotor;
            } else { 
              pos = 15;
              FSMstate = stateIncrementVel;
            }
        } 
        if(analogRead(VelAtual)>VelMax && motor.checaEstadoMotor() == LIGADO) {
          FSMstate = stateDesligaMotor;
        } 

      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateIncrementVel: 

      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) == PRESSIONADO){
          FSMstate = stateFreiando;
        }

        if(analogRead(VelAtual)<VelMax && pos <= 35 && (millis()-time_ac>200)) {
          pos += 1;
          motor.servoWrite(pos);

          time_ac = millis();
        }
        if(analogRead(VelAtual)>=VelMax)
          FSMstate = stateMonitoraVel;

      } else { FSMstate = motor.desligaStartStop(); }
       
    break;
  

    case stateDesligaMotor:

      if(digitalRead(switchSS) == LOW){
        pos = 0;
        motor.desligaMotor();
        FSMstate = stateMonitoraVel;
        
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateLigaMotor:

      if(digitalRead(switchSS) == LOW){
        if(digitalRead(freio) == PRESSIONADO) {
          FSMstate = stateFreiando;
        } else {
          motor.ligaMotor();
          FSMstate = stateMonitoraVel;
        }
          
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateFreiando:
      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) == PRESSIONADO) {
          digitalWrite(ledRed,HIGH);
          pos = 0;
          motor.servoWrite(pos);
        }
        else{
          digitalWrite(ledRed,LOW);
          FSMstate = stateMonitoraVel;
        }

      } else { 
        digitalWrite(ledRed,LOW);
        FSMstate = motor.desligaStartStop();
      }

    break;
    
    default: FSMstate = stateSS_off;

    break;

  }

  display.atualizaDisplay(motor, FSMstate, pos);

  motor.setEstadoMotor(motor.checaEstadoMotor()); 
}
