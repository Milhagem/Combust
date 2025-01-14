#include "Velocidade.h"
#include "Display.h"

volatile unsigned long pulseInterval;     // ms
volatile unsigned long lastPulseInterval; // ms
volatile unsigned long pulseIntervals[sampleSize];
volatile int pulseIndex;
volatile unsigned long lastTime;          // ms
volatile unsigned long timeAjust;         // ms
volatile unsigned long timerCalcVel;      // ms
volatile unsigned long lastTimerCalcVel;  // ms
volatile unsigned long timeEstabilizaVel;
volatile float velocidade;
unsigned long lastTimerTax;  // ms
unsigned long averagePulseInterval = 0;

//Variaveis calculo aceleração --- Variaveis de teste
float velocOld;
volatile unsigned long averagePulseIntervalOld;
volatile float aceleracao;




void calculaVelocidade(float &veloc, float &aceleracao) {

  if(millis() - lastTimerTax >= taxaAtualizacaoVel) {
    //lastTimerTax = millis();
    unsigned long timer_agora = millis();
    detachInterrupt(digitalPinToInterrupt(pinSensorHall));
    averagePulseInterval = 0;
    // filtro media movel -----------------------------------
    for (int i = 0; i < sampleSize; i++) {
      averagePulseInterval += pulseIntervals[i];
    }
    averagePulseInterval /= sampleSize;
    //-------------------------------------------------------

    veloc = circunfRoda/(pulsosPorVolta*averagePulseInterval) * MS_to_S * MPS_to_KMPH;
    //veloc = filtroVelocVariacoesGrandes(velocOld, velocNew);

    //Calculo aceleração.


    aceleracao = (veloc - velocOld) / ((timer_agora - lastTimerTax) * MS_to_S);
    aceleracao = velocOld;
    lastTimerTax = millis();
    velocOld = veloc;
    averagePulseIntervalOld = averagePulseInterval;
    attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
    Serial.print("veloc:");
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
  pulseIntervals[pulseIndex] = pulseInterval;
  pulseIndex = (pulseIndex + 1) % sampleSize; 
}

int calculaVelocidade_Pedal(int pinPedal) {
  return analogRead(pinPedal);
}