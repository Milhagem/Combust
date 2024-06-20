#include "Velocidade.h"

int velocidadeAtual;          // km/h
unsigned long lastmillis;     // ms
volatile int picoLeituraHall;

int atualizaVelocidadeAtual(int velocidadeAtual) {
  if((millis() - lastmillis) > taxaAtualizacaoVel){ 
    
    detachInterrupt(digitalPinToInterrupt(pinSensorHall));

    float distanciaPercorrida = picoLeituraHall/quantImas * diametroRodaURban; // m
    float velocidadeRaw = distanciaPercorrida/taxaAtualizacaoVel;             // m/ms
    float velocidadeNew = velocidadeRaw * MS_to_S * MPS_to_KMPH;                

    lastmillis = millis();
    picoLeituraHall = 0;

    attachInterrupt(digitalPinToInterrupt(pinSensorHall), variador, RISING);

    return velocidadeNew;
  }
  return velocidadeAtual;
}

void variador() { picoLeituraHall++; }
