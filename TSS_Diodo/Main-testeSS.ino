#include "Velocidade.hpp"
#include "Display.hpp"
#define pinLigaMotor    6
#define pinDesligaMotor 5
#define LM2907          A2
#define pinServo        8
#define switchSS 4
#define pinFreio 21

enum StatesStartStop {
    stateSwitchON,
    stateSwitchOFF,
    stateLigaMotor,
    stateDesligaMotor,
    stateEstabilizaAcelera,
    stateEstabilizaVelocidade,
    stateManipulaBorboleta,
    stateStart,
    stateStop,
    stateFreando,
    stateNotLigou,
    stateNotDesligou,
    stateDesligaStartStop
};

extern unsigned long timerIncrementoServo;
extern volatile unsigned long pulseInterval;     // ms
extern volatile unsigned long lastPulseInterval; // ms
extern volatile unsigned long timerCalcVel;      // ms
extern volatile unsigned long lastTimerCalcVel;  // ms
extern volatile unsigned long timeEstabilizaVel;
extern volatile unsigned long timeEstabilizacao; 
extern volatile unsigned long pulseIntervals[sampleSize];
extern volatile int pulseIndex;
extern unsigned long lastTimerTax;


float tempoUltimoIncremento = 0;

Display display; 
unsigned int time = 0; 

void setup() {
   Serial.begin(9600);
    
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
    
    //-----------------------------------------------Variaveis de teste
     timerCalcVel = 0;
    lastTimerCalcVel = 0;
    pulseInterval = 0;
    lastPulseInterval = 0;
    for (int i = 0; i < sampleSize; i++) { pulseIntervals[i] = 1; }
    pulseIndex = 0;
  
    attachInterrupt (digitalPinToInterrupt (pinSensorHall), Velocidade::calc, RISING);
  
    //------------------------------------------------------------------
  
    
    // Display LCD
    display.iniciaDisplay(); 
    float vel=0.0;
    Velocidade::setVelocidade(vel);
}

void loop() {
    display.atualizaDisplay(Velocidade::getVelocidade(), Velocidade::getAcelera());
}  

   