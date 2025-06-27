#include "Velocidade.hpp"

volatile unsigned long pulseInterval;     // ms
volatile unsigned long lastPulseInterval; // ms
volatile unsigned long pulseIntervals [sampleSize];
volatile int pulseIndex;
volatile unsigned long lastTime;          // ms
volatile unsigned long timeAjust;         // ms
volatile unsigned long timerCalcVel;      // ms
volatile unsigned long lastTimerCalcVel;  // ms
volatile unsigned long timeEstabilizaVel;
unsigned long lastTimerTax;  // ms
volatile  float velocOld;
volatile unsigned long averagePulseIntervalOld;

float Velocidade::acelera = 0;

float Velocidade::velocidade = 0;

void Velocidade::setVelocidade (float &vel) { velocidade = vel; }

float Velocidade::getVelocidade () { return velocidade; }

float Velocidade::getAcelera () { return acelera; }

float Velocidade::calculaVelocidade () {

  if(millis() - lastTimerTax >= taxaAtualizacaoVel) {
    lastTimerTax = millis();

    detachInterrupt(digitalPinToInterrupt(pinSensorHall));
    unsigned long averagePulseInterval = 0;
    // filtro media movel -----------------------------------
    for (int i = 0; i < sampleSize; i++) {
      averagePulseInterval += pulseIntervals[i];
    }
    averagePulseInterval /= sampleSize;
    //-------------------------------------------------------

    velocidade = circunfRoda/(pulsosPorVolta*averagePulseInterval) * MS_to_S * MPS_to_KMPH;
    acelera = (velocidade - velocOld) / (taxaAtualizacaoVel/3600000.0);
    velocOld = velocidade;
    averagePulseIntervalOld = averagePulseInterval; 

    attachInterrupt(digitalPinToInterrupt (pinSensorHall), calc, RISING);
    //Serial.print("veloc:");
    return velocidade;
  } else { return velocidade; }
}

float Velocidade::filtroVelocVariacoesGrandes (float velocidadeOld, float velocidadeNew) {
  const int limiteSuperiorVel = 1.9;
  const int limiteInferiorVel = 0.4;

  if(velocidadeNew >= limiteSuperiorVel*velocidadeOld && velocidadeOld != 0){
    return velocidadeOld;
   } else if(velocidadeNew <= limiteInferiorVel*velocidadeOld && velocidadeOld != 0){
    return velocidadeOld;
   } else {
    return velocidadeNew;
   }
 }

void Velocidade::calc() {
  pulseInterval = millis() - lastPulseInterval;
  lastPulseInterval = millis();
  pulseIntervals[pulseIndex] = pulseInterval;
  pulseIndex = (pulseIndex + 1) % sampleSize; 
}

int Velocidade::calculaVelocidade_Pedal(int pinPedal) {
  return analogRead(pinPedal);
}