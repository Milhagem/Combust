#include "Velocidade.hpp"

//#include <PinChangeInterrupt.h> // Inclua a nova biblioteca aqui também

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


unsigned long Velocidade::pulsos=0;
unsigned long Velocidade::timeAcelera =0;
float Velocidade::acelera_2 = 0.0;

float Velocidade::velocidade_rpm = 0.0;

float Velocidade::acelera = 0;

float Velocidade::velocidade = 0;

float MS_TO_S_FACTOR = 1000.0;
// Constantes de conversão para as unidades corretas
#define MS_to_S_FACTOR 1000.0 // Milissegundos para segundos
#define MPS_to_KMPH_FACTOR 3.6 // Metros por segundo para Quilômetros por hora
#define SECONDS_to_HOURS_FACTOR 3600.0 // Segundos para horas

void Velocidade::setVelocidade (float &vel) { velocidade = vel; }

void Velocidade::setAceleracao (float &aceleracao) {
  acelera = aceleracao; 
}

float Velocidade::getVelocidade () {
  if(millis() - lastTimerTax >= taxaAtualizacaoVel) { 
    lastTimerTax = millis();
    calculaVelocidade();
    return velocidade;
  } else { return velocidade; }
}

float Velocidade::getAcelera () { 
  calculaAcelera();
  return acelera; 
}

float Velocidade::calculaVelocidade () {

    detachInterrupt(digitalPinToInterrupt(pinSensorHall));
    unsigned long averagePulseInterval = 0;
    // filtro media movel -----------------------------------
    for (int i = 0; i < sampleSize; i++) {
      averagePulseInterval += pulseIntervals[i];
    }
    averagePulseInterval /= sampleSize;
    //-------------------------------------------------------

    float delta_tempo_segundos = taxaAtualizacaoVel / MS_to_S_FACTOR;
    //velocidade = circunfRoda/(pulsosPorVolta*averagePulseInterval) * MS_to_S * MPS_to_KMPH;
    //velocidade = (circunfRoda / (pulsosPorVolta * (averagePulseInterval / MS_to_S_FACTOR))) * MPS_to_KMPH_FACTOR;
    velocidade = (circunfRoda / (pulsosPorVolta * (averagePulseInterval / MS_to_S_FACTOR)));
    if(averagePulseInterval == 0) {
      velocidade = 0;
    }
    Velocidade::acelera_2 = (velocidade / (pulsosPorVolta * (averagePulseInterval / MS_to_S_FACTOR)));
    Velocidade::velocidade_rpm = 60000 / (pulsosPorVolta*averagePulseInterval);
    //velocidade = velocidade_rpm * circunfRoda * 60 / 100000;
    //acelera = (velocidade - velocOld) / (taxaAtualizacaoVel/3600000.0);
    //acelera = (velocidade - velocOld) / delta_tempo_segundos;
    //acelera = (velocidade - velocOld) / delta_tempo_horas;
    //ultimo funcional
    velocidade = 2*velocidade;
    //Velocidade::acelera = acelera;
    averagePulseIntervalOld = averagePulseInterval; 
    //--------------------------------------------------------------

  /*const int limiteSuperiorVel = 1.9;
  const int limiteInferiorVel = 0.4;

  if(velocidade >= limiteSuperiorVel*velocOld && velocOld != 0){
    return velocOld;
   } else if(velocidade <= limiteInferiorVel*velocOld && velocOld != 0){
    return velocOld;
   } else {
    return velocidade;
   }*/
    velocidade = velocidade * MPS_to_KMPH_FACTOR;
    Velocidade::velocidade = velocidade; 
    //Velocidade::velocidade = filtroVelocVariacoesGrandes (velocOld, velocidade);

    attachInterrupt(digitalPinToInterrupt (pinSensorHall), calc, RISING);
    //attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(10), calc, RISING);
    //Serial.print("veloc:");
    return velocidade;
}

float Velocidade::calculaAcelera() {
  unsigned long deltaT = millis() - timeAcelera;
  if (deltaT >= 200) {
    float veloc_atual = calculaVelocidade()/MPS_to_KMPH_FACTOR;
    acelera = (veloc_atual - velocOld)/ (deltaT / MS_TO_S_FACTOR);
    timeAcelera = millis();
    velocOld = veloc_atual;
  }
  return acelera;
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

void Velocidade::incrementaPulsos(){
  pulsos++;
}

float Velocidade::calculaVelocidade2(int tempoUltimoCal) {

  unsigned int intervaloTempo = millis() - tempoUltimoCal;
  if(intervaloTempo >= taxaAtualizacaoVel) {
    detachInterrupt (digitalPinToInterrupt(pinSensorHall));
    velocidade = (pulsos/pulsosPorVolta)/(intervaloTempo/MS_to_S_FACTOR);
    velocidade = velocidade*circunfRoda*MPS_to_KMPH_FACTOR;
    return velocidade;
    attachInterrupt(digitalPinToInterrupt (pinSensorHall), incrementaPulsos, RISING);
  } else { return velocidade; }
}


int Velocidade::calculaVelocidade_Pedal(int pinPedal) {
  return analogRead(pinPedal);
}