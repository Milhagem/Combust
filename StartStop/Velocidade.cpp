#include "Velocidade.h"

int atualizaVelocidadeAtual(int velocidadeAtual) {
  if((millis() - lastmillis) > taxaAtualizacaoVel){ 
    
    detachInterrupt(digitalPinToInterrupt(pinSensorHall));

    float distanciaPercorrida = picoLeituraHall/quantImas * diametroRodaURban; // m
    float velocidadeRaw = distanciaPercorrida/taxaAtualizacaoVel;             // m/ms
    velocidadeAtual = velocidadeAtual * MS_to_S * MPS_to_KMPH;                

    lastmillis = millis();
    picoLeituraHall = 0;

    attachInterrupt(digitalPinToInterrupt(pinSensorHall), variador, RISING);

    return velocidadeAtual;
  }
  return velocidadeAtual;
}