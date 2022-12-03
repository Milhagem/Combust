#include "Motor.h"

#define pLigaMotor    13
#define pDesligaMotor 12
#define freio         11 
#define pServo         8
#define switchSS      10

#define FALSE         0
#define TRUE          1
#define LIGADO        1
#define DESLIGADO     0
#define PRESSIONADO   0

#define vecMin       100
#define vecMax       300
#define ZEROvec       50
#define tensaoMotorON  5


#define stateSS_off       0
#define stateMonitoraVec  1
#define stateIncrementVec 2
#define stateDesligaMotor 3
#define stateLigaMotor    4
#define stateFreiando     5

Motor motor;
Servo servo;
int pos;
// boolean on_off; // chave liga desliga autopilot
boolean estadoMotor = DESLIGADO;
// int vec;  // valor de velocidade (sera definido ainda)
int FSMstate = 0;

int valorInicial; 
float tensao;

void setup() {
  servo.attach(pServo);
  
  pinMode(pLigaMotor,OUTPUT);
  pinMode(pDesligaMotor,OUTPUT);
  pinMode(freio,INPUT_PULLUP);  // (Fa)
  pinMode(switchSS,INPUT_PULLUP); // (Z)
  pinMode(vecAtual,INPUT);  // velocidade atual
  pinMode(comparaTensao,INPUT);  // comparador de tensao 
  pinMode(9,OUTPUT);
  
  servo.write(0); // Inicia motor posição zero
  estadoMotor = DESLIGADO;
  digitalWrite(pLigaMotor,LOW);
  digitalWrite(pDesligaMotor,LOW);
  
  Serial.begin(9600);
}


void loop() {
  switch (FSMstate)
  { 
    case 0:
      Serial.println("FSMstate = StartStop OFF");

      if(digitalRead(switchSS) == 0 && analogRead(vecAtual)>vecMin){
        FSMstate = stateMonitoraVec;
      }
    break;
  

    case 1:
      Serial.println("FSMstate = StartStop OFF");

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
  

    case 2: 
      Serial.println("FSMstate = Incrementa Velocidade");
      while(digitalRead(switchSS) == DESLIGADO) {
        if(digitalRead(freio) != PRESSIONADO){
          if(pos <= 80) {
            pos+=10;
            servo.write(pos);
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
  

    case 3:
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
  

    case 4:
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
  

    case 5:
      Serial.println("FSMstate = Freio Pressionado"); 
        while(digitalRead(switchSS) == DESLIGADO) {
          if(digitalRead(freio) == PRESSIONADO){
            pos = 0;
            servo.write(pos);
          } else if(digitalRead(freio) != PRESSIONADO ){// Freio solto
            FSMstate = stateMonitoraVec;
          }
        }

      FSMstate = motor.desligaStartStop();
    break;
  
    default: FSMstate = stateSS_off;

  }
}

