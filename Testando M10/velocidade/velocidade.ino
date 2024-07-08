#include "Arduino.h"

#define pinSensorHall DD2

#define pulsosPorVolta     5
#define sampleSize         3
#define velocidadeMaxima   35   // km/h
#define taxaAtualizacaoVel 100 // ms
#define circunfRoda        0.47  // m

#define MPS_to_KMPH 3.6  // metros por segundo p/ quilometros por hora
#define MS_to_S     1000 // milisegundos p/ segundos

unsigned long pulseInterval;
unsigned long lastPulseInterval;
unsigned long timerCalcVel;     // ms
unsigned long lastTimerCalcVel; // ms
float velocidade;                 // km/h
float lastVelocidade;

unsigned long pulseTimes[sampleSize];
int pulseIndex;

/**
 * @brief Armazena o valor de periodo da onda no vetor usado para o calculo de velocidade
 * 
 */
void calc();

void setup() {
  Serial.begin(9600);

  pinMode(pinSensorHall, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
  
  pulseInterval = 0;
  lastPulseInterval = 0;
  timerCalcVel = 0;
  lastTimerCalcVel = 0;
  velocidade = 0.0;
  lastVelocidade = 0.0;
  for (int i = 0; i < sampleSize; i++) { pulseTimes[i] = 0; }
  pulseIndex = 0;
}

void loop() {
  timerCalcVel = millis() - lastTimerCalcVel;
  if(timerCalcVel >= taxaAtualizacaoVel) {
    lastTimerCalcVel = millis();
    detachInterrupt(digitalPinToInterrupt(pinSensorHall));
    unsigned long averagePulseTime = 0;  // ms
    for (int i = 0; i < sampleSize; i++) {
      averagePulseTime += pulseTimes[i];
    }
    averagePulseTime /= sampleSize;
    if(velocidade >= 1.4*lastVelocidade && velocidade != 0){
    velocidade = lastVelocidade;
    } else if(velocidade <= 0.4*lastVelocidade && velocidade != 0){
    velocidade = lastVelocidade;
    } else {
      velocidade = circunfRoda/(pulsosPorVolta*averagePulseTime) * MS_to_S * MPS_to_KMPH;
    }  
    lastVelocidade = velocidade;
    attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
    Serial.print("velocidade:");
    Serial.println(velocidade);
  }
}

void calc() {
  pulseInterval = millis() - lastPulseInterval;
  lastPulseInterval = millis();
  pulseTimes[pulseIndex] = pulseInterval;
  pulseIndex = (pulseIndex + 1) % sampleSize; 
}
