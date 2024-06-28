#include "Velocidade.h"

const unsigned long maximoSample = MS_to_S * circunfRoda/(pulsosPorVolta) * MPS_to_KMPH; // Isso não faz o menor sentido!!!!!!!!

volatile int picoLeituraHall;

unsigned long lastPulseTime; // ms
unsigned long currentMillis; // ms
unsigned long timecontador;  // ms
unsigned long pulseTime;     // ms
float speed;            // km/h
float last_speed;

unsigned long pulseTimes[sampleSize];
int pulseIndex;

float calculaVelocidade(float speed) {
  currentMillis = millis();
  pulseTime = currentMillis - lastPulseTime;

  if(pulseTime >= taxaAtualizacaoVel) {
    currentMillis = millis();
    lastPulseTime = currentMillis;
    detachInterrupt(digitalPinToInterrupt(pinSensorHall));
    unsigned long averagePulseTime = 0;  // ms
    for (int i = 0; i < sampleSize; i++) {
      averagePulseTime += pulseTimes[i];
    }
    averagePulseTime /= sampleSize;
    speed = circunfRoda/(pulsosPorVolta*averagePulseTime) * MS_to_S * MPS_to_KMPH;
    attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
  }
}

float filtroVelocidade(float velocidadeAtual, float velocidadeNew) {
  const int limiteSuperiorVel = 1.9;
  const int limiteInferiorVel = 0.4;

  if(velocidadeNew >= limiteSuperiorVel*speed && velocidadeAtual != 0){
    return velocidadeAtual;
  } else if(velocidadeNew <= limiteInferiorVel*speed && velocidadeAtual != 0){
    return velocidadeAtual;
  } else {
    return velocidadeNew;
  }
}

void calc() {
  currentMillis = millis();
  pulseTime = currentMillis - lastPulseTime;
  timecontador = millis();
  lastPulseTime = currentMillis;
  pulseTimes[pulseIndex] = pulseTime;
  pulseIndex = (pulseIndex + 1) % sampleSize; 
  lastPulseTime = pulseTime;
}

int calculaVelocidade_Pedal(int pinPedal) {
  return analogRead(pinPedal);
}
