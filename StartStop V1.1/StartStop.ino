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

// Variaveis que precisam ser obrigatoriamente declaradas aqui e em outros arquivos--
extern unsigned long timerIncrementoServo;
extern volatile unsigned long pulseInterval;     // ms
extern volatile unsigned long lastPulseInterval; // ms
extern volatile unsigned long pulseIntervals[sampleSize];
extern volatile int pulseIndex;
extern unsigned long lastTimerTax;  // ms
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// Variaveis de verificação do estado do motor
extern unsigned long lastTime;          // ms
extern unsigned long timeAjust;         // ms

// Ponteiro para o calculo da velocidade
extern float velocidade;                // km/h 

// Variavel para acelerar o motor
extern int posServo; // representação de uma determinada angulatura
//-----------------------------------------------------------------------------------

Motor motor;
Display display;

int FSMstate = stateIncrementVel;
unsigned long timeEstabilizaVel;
unsigned long timeEstabilizacao; 

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
  // analogWrite(pinPedalGND, 0);    // Pino A8  -> GND Pedal
  // analogWrite(pinPedalVCC, 1023); // Pino A10 -> VCC Pedal

  motor.setEstadoMotor(DESLIGADO);

  //--------------------------------------------------------------
  timerCalcVel = 0;
  lastTimerCalcVel = 0;
  pulseInterval = 0;
  lastPulseInterval = 0;
  for (int i = 0; i < sampleSize; i++) { pulseIntervals[i] = 0; }
  pulseIndex = 0;
  //--------------------------------------------------------------

  lastTime = 0;
  timeAjust = 1500; //1.5 segundos de espera para tentar religar o motor não ligou
  
  //pontero para o calculo da velocidade
  velocidade = 0.0;

  // Acelerar motor
  posServo = 0;
  motor.servoAttach(pinServo);
  motor.servoWrite(posZeroServo);  // Poe o servo na posicao inicial
  timerIncrementoServo = 0; 

  timeEstabilizaVel = 0;
  timeEstabilizacao = 4000 //Tempo de estabilização de 4 segundos
  
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

      if(velocidade >= velZERO){
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
      
      if(velocidade<velMin && velocidade>=velZERO) {
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

      if(velocidade<velMax && posServo < posMaxServo) {
        giraServoMotor(motor);
        FSMstate = stateIncrementVel;
        break;
      }

      if(velocidade>=velMax) {
        FSMstate = estabilizaVel;
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
      motor.desligaMotor(velocidade);
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
      

      motor.ligaMotor(velocidade);

      if (motor.estadoMotor == TRUE){
        FSMstate = stateMonitoraVel;
      }
      else{
        FSMstate = stateNaoLigou;
      }

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

    case stateNaoLigou:
      calculaVelocidade(velocidade);
      if(digitalRead(switchSS) == NOT_PRESSIONADO) {
        FSMstate = motor.desligaStartStop();
        break;
      }

      if (millis() - lastTime >= timeAjust && motor.estadoMotor = TRUE)
      {
        lastTime = millis();
        FSMstate = stateLigaMotor;
        break;
      }
      lastTime = millis();
      FSMstate = stateNaoLigou;
      
    break;

    case estabilizaVel:
      calculaVelocidade(velocidade);
      if(digitalRead(switchSS) == NOT_PRESSIONADO){
        FSMstate = motor.desligaStartStop();
        break;
      }

      if(digitalRead(pinFreio) == PRESSIONADO){
        FSMstate = stateFreando;
        break;
      }

      if(millis() - timeEstabilizaVel >= timeEstalizacao){
        FSMstate = stateDesligaMotor;
        break;
      }

    break

      
    default: FSMstate = stateSS_off;
  }

  display.atualizaDisplay(motor, velocidade, FSMstate);
  motor.setEstadoMotor(motor.checaEstadoMotor()); 
}
