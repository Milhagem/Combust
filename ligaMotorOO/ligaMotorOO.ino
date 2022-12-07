#include "Motor.h"

/**
 * Conferir funcao ligaMotor(); - Ela aciona os reles?
 * Conferir funcao desligaMotor(); - Ela desliga os reles?
*/


#define pedalGND  A8
#define pedalVcc A10
#define pinServo   8
#define freio    A15
#define switchSS A14

#define FALSE         0
#define TRUE          1
#define PRESSIONADO   0

#define stateSS_off       0
#define stateMonitoraVec  1
#define stateIncrementVec 2
#define stateDesligaMotor 3
#define stateLigaMotor    4
#define stateFreiando     5

Motor motor;
int pos;
boolean estadoMotor = DESLIGADO;
int FSMstate = stateSS_off;

int valorInicial; 
float tensao;

void setup() {
  motor.servoAttach(pinServo);
  
  pinMode(pinLigaMotor,OUTPUT);
  pinMode(pinDesligaMotor,OUTPUT);
  pinMode(freio,INPUT_PULLUP);
  pinMode(switchSS,INPUT_PULLUP);
  pinMode(vecAtual,INPUT); 
  pinMode(comparaTensao,INPUT);
  pinMode(9,OUTPUT);

  digitalWrite(pinLigaMotor,LOW);
  digitalWrite(pinDesligaMotor,LOW);
  analogWrite(pedalGND, 0); // Pino 8 -> GND Pedal
  analogWrite(pedalVcc, 1023); // Pino 10 -> Vcc Pedal

  motor.servoWrite(0);  // Poe o servo na posicao inicial
  estadoMotor = DESLIGADO;
  
  Serial.begin(9600);
}


void loop() {
  switch (FSMstate)
  { 
    case stateSS_off:
      Serial.println("FSMstate = StartStop OFF");
      Serial.print("Velocidade:" );
      Serial.println(analogRead(vecAtual));
      delay(1000);

      if(digitalRead(switchSS) == DESLIGADO && analogRead(vecAtual)>vecMin){
        FSMstate = stateMonitoraVec;
      }
    break;
  

    case stateMonitoraVec:
      Serial.println("FSMstate = Monitora velocidade");

      while(digitalRead(switchSS) == DESLIGADO) {
        if(digitalRead(freio) == PRESSIONADO) {
          FSMstate = stateFreiando;
        }

        if(analogRead(vecAtual)<vecMax && analogRead(vecAtual)>ZEROvec &&
           digitalRead(freio) != PRESSIONADO) {
          motor.ligaMotor(); 
          FSMstate = stateIncrementVec;
        } else if(analogRead(vecAtual)>vecMax && digitalRead(freio) != PRESSIONADO) {
          motor.desligaMotor(); 
          FSMstate = stateDesligaMotor;
        }
      }

      FSMstate = motor.desligaStartStop();
    break;
  

    case stateIncrementVec: 
      Serial.println("FSMstate = Incrementa Velocidade");

      while(digitalRead(switchSS) == DESLIGADO) {
        if(digitalRead(freio) != PRESSIONADO){
          if(pos <= 80) {
            pos+=10;
            motor.servoWrite(pos);
            motor.printVelocidade();
            Serial.print("    ; Posicao: "); 
            Serial.println(pos);
            Serial.println("Motor ligado");
            FSMstate = stateMonitoraVec;
          }
        } else if(digitalRead(freio) == PRESSIONADO ){
          FSMstate = stateFreiando;
        }
      }
       
      FSMstate = motor.desligaStartStop();
    break;
  

    case stateDesligaMotor:
      Serial.println("FSMstate = Desliga Motor");

      while(digitalRead(switchSS == DESLIGADO)){
        if(analogRead(vecAtual)<vecMin && digitalRead(freio) != PRESSIONADO){
          motor.ligaMotor();
          FSMstate = stateLigaMotor;
        } else if(digitalRead(freio) == PRESSIONADO ){
          FSMstate = stateFreiando;
        } 
      }

      FSMstate = motor.desligaStartStop();
    break;
  

    case stateLigaMotor:
      Serial.println("FSMstate = Liga Motor"); 

      while(digitalRead(switchSS) == DESLIGADO){
        if(digitalRead(freio) != PRESSIONADO) {
          motor.ligaMotor();
          FSMstate = stateMonitoraVec;
        } else if(digitalRead(freio) == PRESSIONADO){
          FSMstate = stateFreiando;
        }    
      }
    
      FSMstate = motor.desligaStartStop();
    break;
  

    case stateFreiando:
      Serial.println("FSMstate = Freio Pressionado"); 
    
      while(digitalRead(switchSS) == DESLIGADO) {
        if(digitalRead(freio) == PRESSIONADO){
          pos = 0;
          motor.servoWrite(pos);
        } else if(digitalRead(freio) != PRESSIONADO ){
          FSMstate = stateMonitoraVec;
        }
      }

      FSMstate = motor.desligaStartStop();
    break;
  
    default: FSMstate = stateSS_off;

  }
}
