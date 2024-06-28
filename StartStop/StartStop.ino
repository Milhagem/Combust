#include "Motor.h"
#include "Velocidade.h"
#include "IncrementaVelocidade.h"
#include "Display.h"

#define pinFreio A15
#define switchSS A13

#define stateSS_off       0
#define stateSS_on        1
#define stateMonitoraVel  2
#define stateIncrementVel 3
#define stateDesligaMotor 4
#define stateLigaMotor    5
#define stateFreando      6

#define PRESSIONADO     0x0
#define NOT_PRESSIONADO 0x1

extern volatile int picoLeituraHall;
extern float speed;
extern float last_speed;
extern int posServo;


Motor motor;
Display display;

int FSMstate = stateSS_off;

void setup() {
  Serial.begin(115200);
  
  pinMode(pinLigaMotor, OUTPUT);
  pinMode(pinDesligaMotor, OUTPUT);
  pinMode(pinVelPedal, INPUT); 
  pinMode(LM2907, INPUT);
  pinMode(pinSensorHall, INPUT);
  pinMode(pinFreio, INPUT_PULLUP);
  pinMode(switchSS, INPUT_PULLUP);
  
  digitalWrite(pinLigaMotor, LOW);
  digitalWrite(pinDesligaMotor, LOW);
  analogWrite(pinPedalGND, 0);    // Pino A8  -> GND Pedal
  analogWrite(pinPedalVCC, 1023); // Pino A10 -> VCC Pedal

  motor.setEstadoMotor(DESLIGADO);

  // Sensor Hall
  picoLeituraHall = 0;
  speed = 0;
  last_speed = 0;

  // Servo
  posServo = 0;
  motor.servoAttach(pinServo);
  motor.servoWrite(posZeroServo);  // Poe o servo na posicao inicial
  
  // Display LCD
  display.iniciaDisplay();
}


void loop() {
  switch (FSMstate) {
    case stateSS_off:
      speed = calculaVelocidade(speed);
      if(digitalRead(switchSS) == PRESSIONADO){
        FSMstate = stateSS_on;
      }
    break;
  
    case stateSS_on:
      if(digitalRead(switchSS) == NOT_PRESSIONADO){
        FSMstate = motor.desligaStartStop();
        break;
      }
      speed = calculaVelocidade(speed);
      if(speed > velZERO){
        FSMstate = stateMonitoraVel;
        break;
      }
    break;

    case stateMonitoraVel:
      if(digitalRead(switchSS) == NOT_PRESSIONADO) {
        FSMstate = motor.desligaStartStop();
        break;
      } 
      
      if(digitalRead(pinFreio) == PRESSIONADO) {
        FSMstate = stateFreando;
        break;
      }
      
      speed = calculaVelocidade(speed);
      if(speed<velMin && speed>velZERO) {
        if(motor.checaEstadoMotor() == DESLIGADO) {
          FSMstate = stateLigaMotor;
          break;
        } else if(motor.checaEstadoMotor() == LIGADO) { 
          FSMstate = stateIncrementVel;
          break;
        }
      }

      speed = calculaVelocidade(speed);
      if(speed > velMax && motor.checaEstadoMotor() == LIGADO) {
        FSMstate = stateDesligaMotor;
        break;
      } 
    break;
  

    case stateIncrementVel: 
      if(digitalRead(switchSS) == NOT_PRESSIONADO) {
        FSMstate = motor.desligaStartStop();
        break;
      }

      if(digitalRead(pinFreio) == PRESSIONADO) {
        FSMstate = stateFreando;
        break;
      }
      
      speed = calculaVelocidade(speed);
      if(speed<velMax) {
        /* Desenvolver logica de incremento de velocidade*/
        break;
      }

      speed = calculaVelocidade(speed);
      if(speed>=velMax) {
        FSMstate = stateMonitoraVel;
        break;
      }
    break;
  

    case stateDesligaMotor:
      if(digitalRead(switchSS) == NOT_PRESSIONADO) {
        FSMstate = motor.desligaStartStop(); ;
        break;
      }

      posServo = posZeroServo;
      motor.desligaMotor();
      FSMstate = stateMonitoraVel;
    break;
  

    case stateLigaMotor:
      if(digitalRead(switchSS) == NOT_PRESSIONADO) {
        FSMstate = motor.desligaStartStop();
        break;
      }

      if(digitalRead(pinFreio) == PRESSIONADO) {
        FSMstate = stateFreando;
        break;
      }
      
      motor.ligaMotor();
      FSMstate = stateMonitoraVel;
    break;

    case stateFreando:
      if(digitalRead(switchSS) == NOT_PRESSIONADO) {
        FSMstate = motor.desligaStartStop();
        break;
      }

      if(digitalRead(pinFreio) == PRESSIONADO) {
        posServo = posZeroServo;
        motor.servoWrite(posServo);
        FSMstate = stateFreando;
        break;
      } else {
        FSMstate = stateMonitoraVel;
        break;
      }
    break;
    
    default: FSMstate = stateSS_off;
  }

  display.atualizaDisplay(motor, speed, FSMstate);
  motor.setEstadoMotor(motor.checaEstadoMotor()); 
}
