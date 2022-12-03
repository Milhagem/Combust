#include "Motor.h"

#define pinLigaMotor    13 // CONFERIR
#define pinDesligaMotor 12 // CONFERIR
#define freio           11 
#define pinServo         8
#define switchSS        10

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
// boolean on_off; // chave liga desliga autopilot
boolean estadoMotor = DESLIGADO;
// int vec;  // valor de velocidade (sera definido ainda)
int FSMstate = 0;

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
  
  motor.servoWrite(0);
  estadoMotor = DESLIGADO;
  digitalWrite(pinLigaMotor,LOW);
  digitalWrite(pinDesligaMotor,LOW);
  
  Serial.begin(9600);
}


void loop() {
  switch (FSMstate)
  { 
    case stateSS_off:
      Serial.println("FSMstate = StartStop OFF");

      if(digitalRead(switchSS) == 0 && analogRead(vecAtual)>vecMin){
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
        if(digitalRead(freio) != PRESSIONADO) { //&& digitalRead(Lx)==1{
          // Resposta da comparacao do sinal LM2097 (diferenca de tensao se manteve 2V)
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
        } else if(digitalRead(freio) != PRESSIONADO ){// Freio solto
          FSMstate = stateMonitoraVec;
        }
      }

      FSMstate = motor.desligaStartStop();
    break;
  
    default: FSMstate = stateSS_off;

  }
}

