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
#define stateSS_on        1
#define stateMonitoraVel  2
#define stateIncrementVel 3
#define stateDesligaMotor 4
#define stateLigaMotor    5
#define stateFreiando     6
#define TensaoMotorAcelerando   2.7

/* Variaveis para o stateIncrementVel */
#define posMaxServo         180 // 100 graus (angulo)
#define intervIncrementaVel 200 // ms
#define increvementoServo   1   // graus
long int timeIncrement = 0;

#define posicaoZeroServo     0  // graus
#define posicaoInicialServo  40 // graus

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

      motor.desligaStartStop();
      if(digitalRead(switchSS) == LOW && analogRead(velAtual)<ZEROVel){
          FSMstate = stateSS_on;
        }else{
          if(digitalRead(switchSS) == LOW && analogRead(velAtual) > ZEROVel){
            FSMstate = stateMonitoraVel;
          }
        }

    break;


    case stateSS_on:

      if(digitalRead(switchSS) == PRESSIONADO) {
        if(digitalRead(switchSS) == PRESSIONADO) {
          FSMstate = stateFreando;
        } else {
          if(digitalRead(velAtual) > ZEROVel){
            FSMstate = stateMonitoraVel;
          }
        }
      } else { FSMstate = motor.desligaStartStop(); }
      
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

      if(digitalRead(switchSS) == PRESSIONADO) {
        if(digitalRead(freio) == PRESSIONADO) {
          FSMstate = stateFreando;
        }

        if(analogRead(velAtual)<VelMin && analogRead(velAtual)>ZEROVel) {
            if(motor.checaEstadoMotor() == DESLIGADO) {
              FSMstate = stateLigaMotor;
            } else { 
              posicaoServo = posicaoInicialServo;
              FSMstate = stateIncrementVel;
            }
        } 
        if(analogRead(velAtual)>VelMax && motor.checaEstadoMotor() == LIGADO) {
          FSMstate = stateDesligaMotor;
        } 

      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateIncrementVel: 

      if(digitalRead(switchSS) == PRESSIONADO) {
        if(digitalRead(freio) == PRESSIONADO){
          FSMstate = stateFreando;
        }

        if(analogRead(VelAtual)<VelMax && pos <= 180 && (millis() - timeIncrement > intervIncrementaVel)) { //incremento ocorrendo a cada 200ms
          /* pos += 1;
          motor.servoWrite(pos);
          time_ac = millis(); */
          time_ac = millis();
          if(time_ac > 2000 &&  tensao < TensaoMotorAcelerando){
            pos = 70;
            motor.servoWrite(pos);
            time_ac = millis();
          }else{
              pos += 5;
              motor.servoWrite(pos);
              time_ac = millis();
          }

        }
        
        if(analogRead(velAtual)>=VelMax)
          FSMstate = stateMonitoraVel;

      } else { FSMstate = motor.desligaStartStop(); }
       
    break;
  

    case stateDesligaMotor:

      if(digitalRead(switchSS) == PRESSIONADO){
        posicaoServo = posicaoZeroServo;
        motor.desligaMotor();
        FSMstate = stateMonitoraVel;
        
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateLigaMotor:

      if(digitalRead(switchSS) == PRESSIONADO){
        if(digitalRead(freio) == PRESSIONADO) {
          FSMstate = stateFreando;
        } else {
          motor.ligaMotor();
          FSMstate = stateMonitoraVel;
        }
          
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateFreando:

      if(digitalRead(switchSS) == PRESSIONADO) {
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
