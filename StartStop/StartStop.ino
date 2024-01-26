#include "Motor.h"
#include "Display.h"

#define pedalGND  A8
#define pedalVCC A10
#define pinServo   8
#define freio    A15
#define switchSS A14
#define ledRed     2

#define PRESSIONADO   0

#define stateSS_off       0
#define stateMonitoraVel  1
#define stateIncrementVel 2
#define stateDesligaMotor 3
#define stateLigaMotor    4
#define stateFreiando     5

/* Variaveis para o stateIncrementVel*/
#define posMaxServo         180 // 100 graus (angulo)
#define intervIncrementaVel 200 // ms
#define increvementoServo   1   // graus
long int timeIncrement = 0;

#define posicaoZeroServo     0  // graus
#define posicaoInicialServo  40

Motor motor;
Display display;

int FSMstate = stateSS_off;
int posicaoServo = posicaoZeroServo;

void setup() {
  motor.setEstadoMotor(DESLIGADO);
  
  pinMode(pinLigaMotor, OUTPUT);
  pinMode(pinDesligaMotor, OUTPUT);
  pinMode(freio, INPUT_PULLUP);
  pinMode(switchSS, INPUT_PULLUP);
  pinMode(velAtual, INPUT); 
  pinMode(LM2907, INPUT);

  digitalWrite(pinLigaMotor, LOW);
  digitalWrite(pinDesligaMotor, LOW);
  analogWrite(pedalGND, 0);    // Pino 8 -> GND Pedal
  analogWrite(pedalVCC, 1023); // Pino 10 -> Vcc Pedal

  // Servo
  motor.servoAttach(pinServo);
  motor.servoWrite(0);  // Poe o servo na posicao inicial
  
  // Display LCD
  display.iniciaDisplay();

}


void loop() {

  switch (FSMstate)
  { 
    case stateSS_off:

      if(digitalRead(switchSS) == LOW && analogRead(velAtual)>ZEROVel){
        FSMstate = stateMonitoraVel;
      }

    break;
  

    case stateMonitoraVel: 

      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) == PRESSIONADO) {
          FSMstate = stateFreiando;
        }

        if(analogRead(velAtual)<VelMin && analogRead(velAtual)>ZEROVel) {
            if(motor.checaEstadoMotor() == DESLIGADO) {
              FSMstate = stateLigaMotor;
            } else { 
              posicaoServo = 15;
              FSMstate = stateIncrementVel;
            }
        } 
        if(analogRead(velAtual)>VelMax && motor.checaEstadoMotor() == LIGADO) {
          FSMstate = stateDesligaMotor;
        } 

      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateIncrementVel: 

      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) == PRESSIONADO){
          FSMstate = stateFreiando;
        }

        if(analogRead(velAtual)<VelMax && posicaoServo <= posMaxServo && millis() - timeIncrement > intervIncrementaVel) {
          posicaoServo += increvementoServo;
          motor.servoWrite(posicaoServo);

          timeIncrement = millis();
        }
        if(analogRead(velAtual)>=VelMax)
          FSMstate = stateMonitoraVel;

      } else { FSMstate = motor.desligaStartStop(); }
       
    break;
  

    case stateDesligaMotor:

      if(digitalRead(switchSS) == LOW){
        posicaoServo = posicaoZeroServo;
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
          posicaoServo = posicaoZeroServo;
          motor.servoWrite(posicaoServo);
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

  display.atualizaDisplay(motor, FSMstate, posicaoServo );

  motor.setEstadoMotor(motor.checaEstadoMotor()); 
}
