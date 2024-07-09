#include "Velocidade.h"

unsigned long timerCalcVel;         // ms
unsigned long lastTimerCalcVel;     // ms
unsigned long pulseInterval;     // ms
unsigned long lastPulseInterval; // ms
float velocidade;                // km/h     

unsigned long pulseIntervals[sampleSize];
int pulseIndex;

void calculaVelocidade(float &veloc) {
  timerCalcVel = millis() - lastTimerCalcVel;

  if(timerCalcVel >= taxaAtualizacaoVel) {
    timerCalcVel = millis();

    detachInterrupt(digitalPinToInterrupt(pinSensorHall));
    unsigned long averagePulseInterval = 0;  // ms
    for (int i = 0; i < sampleSize; i++) {
      averagePulseInterval += pulseIntervals[i];
    }
    averagePulseInterval /= sampleSize;
    veloc = circunfRoda/(pulsosPorVolta*pulseInterval) * MS_to_S * MPS_to_KMPH;
    attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
  }
}

float filtroVelocVariacoesGrandes(float velocidadeOld, float velocidadeNew) {
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

void calc() {
  pulseInterval = millis() - lastPulseInterval;
  lastPulseInterval = millis();
  pulseIntervals[pulseIndex] = timerCalcVel;
  pulseIndex = (pulseIndex + 1) % sampleSize; 
}

int calculaVelocidade_Pedal(int pinPedal) {
  return analogRead(pinPedal);
}
