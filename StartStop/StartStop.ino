#include "Motor.h"
#include "Display.h"

#define pedalGND  A8
#define pedalVCC A9
#define pinServo   8
#define freio    A15
#define switchSS A13
#define ledRed     2

#define PRESSIONADO     0x0
#define NAO_PRESSIONADO 0x1

#define stateSS_off       0
#define stateSS_on        1
#define stateMonitoraVel  2
#define stateIncrementVel 3
#define stateDesligaMotor 4
#define stateLigaMotor    5
#define stateFreiando     6

/* Variaveis para medicao de velocidade */
#define MPS_to_KMPH   3.6      // metros por segundo p/ quilometro por hora
#define MS_to_S     000.1      // milisegundos p/ segundos
volatile int picoLeituraHall;
float distanciaPercorrida = 0; // m
float velocidadeRaw;           // m/ms
int velocidadeAtual;           // m/s
unsigned long lastmillis;      // ms  

/* Variaveis para o stateIncrementVel*/
#define posMaxServo         180 // 100 graus (angulo)
#define increvementoServo   1   // graus
#define posicaoZeroServo     0  // graus
#define posicaoInicialServo  40
#define intervIncrementaVel 200 // ms
long int timeIncrement;

Motor motor;
Display display;

int FSMstate = stateSS_off;
int posicaoServo = posicaoZeroServo;

void atualizaVelocidadeAtual();
void variador () { picoLeituraHall++; }

void setup() {
  motor.setEstadoMotor(DESLIGADO);
  
  pinMode(pinLigaMotor, OUTPUT);
  pinMode(pinDesligaMotor, OUTPUT);
  pinMode(freio, INPUT_PULLUP);
  pinMode(switchSS, INPUT_PULLUP);
  pinMode(pinVelPedal, INPUT); 
  pinMode(LM2907, INPUT);

  digitalWrite(pinLigaMotor, LOW);
  digitalWrite(pinDesligaMotor, LOW);
  analogWrite(pedalGND, 0);    // Pino 8 -> GND Pedal
  analogWrite(pedalVCC, 1023); // Pino 10 -> Vcc Pedal

  // Sensor Hall
  picoLeituraHall = 0;
  pinMode(pinSensorHall, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinSensorHall), variador, RISING);
  lastmillis = 0;

  // Servo
  timeIncrement = 0;
  motor.servoAttach(pinServo);
  motor.servoWrite(posicaoZeroServo);  // Poe o servo na posicao inicial
  
  // Display LCD
  display.iniciaDisplay();

}


void loop() {

  switch (FSMstate)
  { 
    case stateSS_off:
      if(digitalRead(switchSS) == PRESSIONADO){
        FSMstate = stateSS_on;
      }
    break;
  
  case stateSS_on:
    if(analogRead(pinVelPedal) > ZEROVel){
        FSMstate = stateMonitoraVel;
    }
    if(digitalRead(switchSS) == NAO_PRESSIONADO){
      FSMstate = stateSS_off;
    } 
    break;

    case stateMonitoraVel: 
      if(digitalRead(switchSS) == PRESSIONADO) {
        if(digitalRead(freio) == PRESSIONADO) {
          FSMstate = stateMonitoraVel;
        }

        if(velocidadeAtual<VelMin && velocidadeAtual>ZEROVel) {
            if(motor.checaEstadoMotor() == DESLIGADO) {
              FSMstate = stateMonitoraVel;
            } else { 
              posicaoServo = 15;
              FSMstate = stateMonitoraVel;
            }
        } 
        if(analogRead(pinVelPedal)> VelMax && motor.checaEstadoMotor() == LIGADO) {
          FSMstate = stateMonitoraVel;
        } 

      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateIncrementVel: 

      if(digitalRead(switchSS) == PRESSIONADO) {
        if(digitalRead(freio) == PRESSIONADO){
          FSMstate = stateFreiando;
        }else{
            if(analogRead(pinVelPedal)<VelMax /*  && posicaoServo <= 180 */ /* && (millis() - timeIncrement > intervIncrementaVel) */) { //incremento ocorrendo a cada 200ms
             
              motor.servoWrite(posicaoServo);
            }
          }
        
        /* if(analogRead(pinVelPedal)<VelMax && pos <= 180 && (millis() - timeIncrement > intervIncrementaVel)) { //incremento ocorrendo a cada 200ms
           pos += 1;
          motor.servoWrite(pos);
          time_ac = millis(); 
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

        }*/
        if(analogRead(pinVelPedal)>=VelMax)
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
          FSMstate = stateFreiando;
        } else {
          motor.ligaMotor();
          FSMstate = stateMonitoraVel;
        }
          
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateFreiando:
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

  display.atualizaDisplay(motor, velocidadeAtual, FSMstate, posicaoServo);

  motor.setEstadoMotor(motor.checaEstadoMotor()); 
}

void incrementaPosicaoServo(){

  /*Quatro situações:
  1 - Posicionamento do servo até realizar a tensão do cabo mas sem acelerar
  2 - Início da aceleração, o servo deve se mover de forma ainda rápida e com maior ângulo para vencer a força oposta da borboleta
  3 - Borboleta diminui a força oposta, logo o servo pode se movimentar mais devagar e com menor amplitude

  Todas as situações acima dependem da análise da velocidade do veículo, caso a velocidade não modifique, deve-se entrar em estado de exceção
  Em teste de bancada ainda não se possui a medição da velocidade de fato, para realizar essa 

  4 - Estado de exceção
  */ 

  /* pos += 1;
  motor.servoWrite(pos);
  time_ac = millis(); */

  timeIncrement = millis();
  float tensao = motor.analisaTensao(); 
  int intervaloTempoServo = millis() - timeIncrement; 
  short int tempoParaAcelerar = 2000; // Tempo para simular a velocidade do carro, isso é, espera-se que em 2000ms

  // Situação 1
  if(pinVelPedal < 250 && tensao < TensaoMotorAcelerando){
    posicaoServo = 70;
    motor.servoWrite(posicaoServo);
  }
  if(timeIncrement > tempoParaAcelerar && tensao >= TensaoMotorAcelerando && (intervaloTempoServo > intervIncrementaVel)){
    posicaoServo += 5;
    motor.servoWrite(posicaoServo);
  }
}

void atualizaVelocidadeAtual() {
  if((millis() - lastmillis) > taxaAtualizacaoVel){ 
    
    detachInterrupt(digitalPinToInterrupt(pinSensorHall));

    distanciaPercorrida = picoLeituraHall/quantImas * tamanhoRodaUrban;
    velocidadeRaw = distanciaPercorrida/taxaAtualizacaoVel;
    velocidadeAtual = velocidadeAtual * MS_to_S * MPS_to_KMPH;

    lastmillis = millis();
    picoLeituraHall = 0;

    attachInterrupt(digitalPinToInterrupt(pinSensorHall), variador, RISING);
  }
}