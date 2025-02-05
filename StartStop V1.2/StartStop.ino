#include "Motor.h"
#include "Velocidade.h"
#include "IncrementaVelocidade.h"
#include "Display.h"

#define pinFreio A15
#define switchSS 20

/*#define stateSS_off       0
#define stateSS_on        1
#define stateMonitoraVel  2
#define stateIncrementVel 3
#define stateDesligaMotor 4
#define stateLigaMotor    5
#define stateFreando      6
#define stateNaoLigou     7
#define estabilizaVel     8*/
#define switchSS A13
//#define pinSensorHall DD2

#define stateSS_off 0
#define stateSS_on  1
#define LigaMotor  2
#define Aguarda 3
#define ManipulaServo 4
#define MantemVelMax 5

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
unsigned long lastTimeinit = 0;
unsigned long timerIncrementoServo = 0; // ms
unsigned long timerMantemVelMax = 0;
unsigned long tempoMaxPista;
unsigned long timerAtualizaDisplay = 0;
int guarda_angulo = 0;
int time_max = 0;
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

int FSMstate = stateSS_off;

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
  
  switch (FSMstate) {

    case stateSS_off:
      if(digitalRead(switchSS)) {FSMstate = stateSS_on;}
    break;

    case stateSS_on:
      if (!digitalRead(switchSS)) {
        FSMstate = stateSS_off;
        motor.desligaStartStop();
        break;
      }
      if(millis() - lastTimeinit >= 5000){

  if(millis() - timerAtualizaDisplay >= 500){
    timerAtualizaDisplay = millis();
    calculaVelocidade(velocidade);
  }
  
  switch (FSMstate) {

    case desligaSS:
      giraServoMotor_desaceleracao(motor);
      delay(3000);
      motor.desligaMotor(velocidade);
      while(1);//Trava execução inicialmente.
    break;

    case stateSS_on:

      if(millis() - lastTimeinit >= 10000){
        FSMstate = LigaMotor;
        break;
      }else {
        FSMstate = stateSS_on;
      }

      FSMstate = stateSS_on;
      
    break;

    case LigaMotor:
       if (!digitalRead(switchSS)) {
        FSMstate = stateSS_off;
        motor.desligaStartStop();
        break;
      }
      motor.ligaMotor(velocidade);
      if(motor.analisaTensao() <= 2.7){
        delay(5000);
        FSMstate = LigaMotor;
      }
      delay(3000);
      FSMstate = Aguarda;
    break;  

    case Aguarda:
       if (!digitalRead(switchSS)) {
        FSMstate = stateSS_off;
        motor.desligaStartStop();
        break;
      }
      if(time_max == 1){
        FSMstate = stateSS_off;
        break;
      }

      if(guarda_angulo >= 20){
        FSMstate = MantemVelMax;
        break;
      }

      FSMstate = ManipulaServo;
    break;

    case ManipulaServo:
       if (!digitalRead(switchSS)) {
        FSMstate = stateSS_off;
        motor.desligaStartStop();
        break;
      }
      if(guarda_angulo >= 25){

      if(guarda_angulo >= 20){
        FSMstate = MantemVelMax;
        break;
      }

      if(millis() - timerIncrementoServo >= 200) {
        giraServoMotor_aceleracao(motor,guarda_angulo);
        timerIncrementoServo = millis();
        FSMstate = ManipulaServo;
        break;
      }

    break;

    case MantemVelMax:
       if (!digitalRead(switchSS)) {
        FSMstate = stateSS_off;
        motor.desligaStartStop();
        break;
      }
      if(millis() - timerMantemVelMax <= tempoMaxPista) {
      mantemVel(motor,guarda_angulo);
      if(millis() - timerMantemVelMax >= tempoMaxPista) {
        timerMantemVelMax = millis();
        time_max = 1;
        FSMstate = Aguarda;
        break;
      }
      FSMstate = MantemVelMax;
    break;

    default: FSMstate = stateSS_off;
  }
  if(millis() - timerAtualizaDisplay >= 500){
    timerAtualizaDisplay = millis();
    calculaVelocidade(velocidade);
    motor.setEstadoMotor(motor.checaEstadoMotor());
    display.atualizaDisplay(motor, velocidade, FSMstate);
    display.mostraTensaoEVel(motor,velocidade);
  }
  
  motor.setEstadoMotor(motor.checaEstadoMotor());
  display.atualizaDisplay(motor, velocidade, FSMstate);
}

