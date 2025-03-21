#include "Motor.h"
#include "Velocidade.h"
#include "IncrementaVelocidade.h"
#include "Display.h"

#define pinFreio A15
#define pinSensorHall DD2

/*Estados antigos
#define desligaSS 0
#define stateSS_on 1
#define LigaMotor 2
#define Aguarda 3
#define ManipulaServo 4
#define MantemVelMax 5*/

//---------------------------------------------
//Variaveis do andre para manipulação da maquina de estados

unsigned long tempoUltimoIncremento = 0;

#define stateSwitchON 0
#define stateSwitchOFF 1
#define stateLigaMotor 2
#define stateDesligaMotor 3
#define stateEstabilizaAcelera 4
#define stateEstabilizaVelocidade 5
#define stateManipulaBorboleta 6
#define stateStart 7
#define stateStop 8
#define stateFreando 9
#define stateNotLigou 10
#define stateNotDesligou 11
#define stateDesligaStartStop 12

#define erroAceitavel 0.1
#define aceleraIdeal 0.5
#define posServoInicial 15
#define velocidadeMinima 5
#define velocidadeMax 30
#define tempoIncrementoIdeal 100
#define velZERO 0
//#define PRESSIONADO 1
//#define NOT_PRESSIONADO 0  
#define switchSS 4
#define pinFreio 21
#define tempoMaximoVelocidade 10000


int tentativasLigar = 0;
int tentativasDesligar = 0;
bool inicioVel = 0;
float tempoInicioVel = 0;

//-----------------------------------------------------

#define PRESSIONADO     0x0
#define NOT_PRESSIONADO 0x1

// Variaveis que precisam ser obrigatoriamente declaradas aqui e em outros arquivos--
extern unsigned long timerIncrementoServo;
extern volatile unsigned long pulseInterval;     // ms
extern volatile unsigned long lastPulseInterval; // ms
extern volatile unsigned long timerCalcVel;      // ms
extern volatile unsigned long lastTimerCalcVel;  // ms
extern volatile unsigned long timeEstabilizaVel;
extern volatile unsigned long timeEstabilizacao; 
extern volatile unsigned long pulseIntervals[sampleSize];
extern volatile int pulseIndex;
extern unsigned long lastTimerTax;  // ms
//------------------------------------------Variaveis de teste
unsigned long lastTimeinit;
unsigned long timerIncrementoServo = 0; // ms
volatile unsigned long timerMantemVelMax = 0;
volatile unsigned long tempoMaxPista;
volatile unsigned long timerAtualizaDisplay = 0;
int guarda_angulo = 0;
volatile int time_max = 0;
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// Variaveis de verificação do estado do motor
extern unsigned long lastTime;          // ms
extern unsigned long timeAjust;         // ms

// Ponteiro para o calculo da velocidade
extern float velocidade;                // km/h 

// Variavel para acelerar o motor
extern volatile int posServo; // representação de uma determinada angulatura
//-----------------------------------------------------------------------------------

Motor motor;
Display display;

int FSMstate = stateSwitchOFF;

void setup() {
  Serial.begin(115200);
  
  pinMode(pinLigaMotor, OUTPUT);
  pinMode(pinDesligaMotor, OUTPUT);
  //pinMode(pinVelPedal, INPUT); 
  pinMode(LM2907, INPUT);
  pinMode(pinSensorHall, INPUT);
  pinMode(pinFreio, INPUT_PULLUP);
  pinMode(switchSS, INPUT_PULLUP);

  
  digitalWrite(pinLigaMotor, LOW);
  digitalWrite(pinDesligaMotor, LOW);
  // analogWrite(pinPedalGND, 0);    // Pino A8  -> GND Pedal
  // analogWrite(pinPedalVCC, 1023); // Pino A10 -> VCC Pedal

  motor.setEstadoMotor(DESLIGADO);

  //-----------------------------------------------Variaveis de teste
  timerCalcVel = 0;
  lastTimerCalcVel = 0;
  pulseInterval = 0;
  lastPulseInterval = 0;
  for (int i = 0; i < sampleSize; i++) { pulseIntervals[i] = 0; }
  pulseIndex = 0;

  attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);

  lastTime = 0;
  timeAjust = 1500; //1.5 segundos de espera para tentar religar o motor não ligou

  lastTimeinit = 6000.0;
  //------------------------------------------------------------------

  tempoMaxPista = 40000;
  
  //pontero para o calculo da velocidade
  velocidade = 0.0;

  // Acelerar motor
  //posServo = 0;
  motor.servoAttach(pinServo);
  motor.servoWrite(posZeroServo);  // Poe o servo na posicao inicial
  timerIncrementoServo = 0; 

  timeEstabilizaVel = 0.0;
  //timeEstabilizacao = 4000.0; //Tempo de estabilização de 4 segundos
  
  // Display LCD
  display.iniciaDisplay();
}


void loop() {

  if(millis() - timerAtualizaDisplay >= 500){
    timerAtualizaDisplay = millis();
    calculaVelocidade(velocidade);
  }
  
  switch (FSMstate) {

    case stateSwitchOFF:
      if (digitalRead(switchSS) == LOW) {
        FSMstate = stateSwitchON;
      } else { FSMstate = stateSwitchOFF;}
      delay(1000);
    break;

    case stateSwitchON:

      if (digitalRead(switchSS) == HIGH){ FSMstate = stateSwitchOFF; }else{
        FSMstate = stateSwitchON;
      }
      if (velocidade >= velocidadeMinima ) {
          FSMstate = stateStop;
      } else if (velocidade  >= velZERO ) {
          FSMstate = stateStart;
      } else { FSMstate = stateSwitchON; }
      
    break;

    case stateEstabilizaAcelera:
      if (digitalRead(switchSS) == NOT_PRESSIONADO) { FSMstate = stateDesligaStartStop; }

      if (digitalRead(pinFreio) == PRESSIONADO) { FSMstate = stateFreando; }
  
      if (motor.checaEstadoMotor()) { FSMstate = stateStart; }  
      
      if (velocidade >= (velocidadeMax - velocidadeMax*erroAceitavel) &&  velocidade <= (velocidadeMax + velocidadeMax*erroAceitavel)) {
          FSMstate = stateStop;
      }

      if ( velocidade >= (aceleraIdeal - aceleraIdeal*erroAceitavel)) {
          FSMstate = stateEstabilizaAcelera;
      } else {
         //borboleta = manterAcelera;
          FSMstate = stateManipulaBorboleta;
      }
      break;

      case stateManipulaBorboleta:

      if (digitalRead(switchSS) == NOT_PRESSIONADO) { FSMstate = stateDesligaStartStop; }

      if (digitalRead(pinFreio) == PRESSIONADO) { FSMstate = stateFreando; }

      if ( millis() - tempoUltimoIncremento >= tempoIncrementoIdeal) {
          if ( velocidade <  aceleraIdeal) {
              giraServoMotor_aceleracao(motor, guarda_angulo);
              tempoUltimoIncremento = millis();
              FSMstate = stateManipulaBorboleta;
          } else { FSMstate = stateEstabilizaAcelera; }
      } else { FSMstate = stateManipulaBorboleta; }

      break;
    /* Duplicado
    case :

      if (motor.ligaMotor(velocidade)) {   
        tentativasLigar = 0;    
        FSMstate = stateStart;
    } else { FSMstate = stateNotLigou; }

    break;*/

    case stateLigaMotor:

      if ( motor.ligaMotor(velocidade)) {   
          tentativasLigar = 0;    
          FSMstate = stateStart;
      } else { FSMstate = stateNotLigou; }

    break;

    case stateDesligaMotor :

      if (digitalRead(switchSS) == NOT_PRESSIONADO) { FSMstate = stateDesligaStartStop; }

      if (digitalRead(pinFreio) == PRESSIONADO) { FSMstate = stateFreando; }

      if (motor.desligaMotor(velocidade)) { 
          tentativasDesligar = 0;      
          FSMstate = stateStop;
      } else { FSMstate = stateNotDesligou; }

    break;

    case stateStart:

      if (digitalRead(switchSS) == NOT_PRESSIONADO) { FSMstate = stateDesligaStartStop; }

      if (digitalRead(pinFreio) == PRESSIONADO) { FSMstate = stateFreando; }

      if (motor.checaEstadoMotor()) { FSMstate = stateLigaMotor; }


      if (velocidade < velocidadeMinima) {
        FSMstate = stateStart;
      } else { FSMstate = stateEstabilizaAcelera; }

    break;

    case stateStop:

      if (digitalRead(switchSS) == NOT_PRESSIONADO) { FSMstate = stateDesligaStartStop; }

      if (motor.checaEstadoMotor()) { FSMstate = stateDesligaMotor; }

      if (velocidade > (velocidadeMinima - velocidadeMinima*erroAceitavel) ) {
          FSMstate = stateStop;
      } else { FSMstate = stateStart; }

    break;

    case stateFreando:

    if (digitalRead(switchSS) == NOT_PRESSIONADO) { FSMstate = stateDesligaStartStop; }

    FSMstate = stateStop;

    break;

    case stateDesligaStartStop :

    if (motor.desligaMotor(velocidade)) {       
        FSMstate = stateSwitchOFF;
    } else { FSMstate = stateNotDesligou; }

    break;

    case stateNotLigou:

    if (tentativasLigar <= 2) {
        tentativasDesligar++;
        delay(1500);
        FSMstate = stateLigaMotor;
    } else {
        //prin
        FSMstate = stateDesligaStartStop;
    }

    case stateNotDesligou:

    if (tentativasDesligar <= 2) {
        tentativasDesligar++;
        delay(1500);
        FSMstate = stateDesligaMotor;
    } else {
        //print
        FSMstate = stateSwitchOFF;
    }

    break;

  }
  motor.setEstadoMotor(motor.checaEstadoMotor());
  display.atualizaDisplay(motor, velocidade, FSMstate);
}
