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

extern unsigned long timerCalcVel;
extern unsigned long lastTimerCalcVel;
extern unsigned long pulseInterval;
extern unsigned long lastPulseInterval;
extern float velocidade;
extern unsigned long pulseIntervals[sampleSize];
extern int pulseIndex;

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
  timerCalcVel = 0;
  lastTimerCalcVel = 0;
  pulseInterval = 0;
  lastPulseInterval = 0;
  velocidade = 0.0;
  for (int i = 0; i < sampleSize; i++) { pulseIntervals[i] = 0; }
  pulseIndex = 0;

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
    calculaVelocidade(velocidade);
      if(digitalRead(switchSS) == PRESSIONADO){
        FSMstate = stateSS_on;
      }
    break;
  
    case stateSS_on:
      calculaVelocidade(velocidade);
      if(digitalRead(switchSS) == NOT_PRESSIONADO){
        FSMstate = motor.desligaStartStop();
        break;
      }

      if(velocidade > velZERO){
        FSMstate = stateMonitoraVel;
        break;
      }
    break;

    case stateMonitoraVel:
      calculaVelocidade(velocidade);
      if(digitalRead(switchSS) == NOT_PRESSIONADO) {
        FSMstate = motor.desligaStartStop();
        break;
      } 
      
      if(digitalRead(pinFreio) == PRESSIONADO) {
        FSMstate = stateFreando;
        break;
      }
      
      if(velocidade<velMin && velocidade>velZERO) {
        if(motor.checaEstadoMotor() == DESLIGADO) {
          FSMstate = stateLigaMotor;
          break;
        } else if(motor.checaEstadoMotor() == LIGADO) { 
          FSMstate = stateIncrementVel;
          break;
        }
      }

      if(velocidade > velMax && motor.checaEstadoMotor() == LIGADO) {
        FSMstate = stateDesligaMotor;
        break;
      } 
    break;
  

    case stateIncrementVel:
      calculaVelocidade(velocidade);
      if(digitalRead(switchSS) == NOT_PRESSIONADO) {
        FSMstate = motor.desligaStartStop();
        break;
      }

      if(digitalRead(pinFreio) == PRESSIONADO) {
        FSMstate = stateFreando;
        break;
      }

      if(velocidade<velMax) {
        /* Desenvolver logica de incremento de velocidade*/
        break;
      }

      if(velocidade>=velMax) {
        FSMstate = stateMonitoraVel;
        break;
      }
    break;
  

    case stateDesligaMotor:
      calculaVelocidade(velocidade);
      if(digitalRead(switchSS) == NOT_PRESSIONADO) {
        FSMstate = motor.desligaStartStop(); ;
        break;
      }

      posServo = posZeroServo;
      motor.desligaMotor();
      FSMstate = stateMonitoraVel;
    break;
  

    case stateLigaMotor:
      calculaVelocidade(velocidade);
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
      calculaVelocidade(velocidade);
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

  display.atualizaDisplay(motor, velocidade, FSMstate);
  motor.setEstadoMotor(motor.checaEstadoMotor()); 
}
