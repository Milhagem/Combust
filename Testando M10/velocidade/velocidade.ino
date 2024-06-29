#include "Arduino.h"

#define pinSensorHall DD2 

#define pulsosPorVolta     5
#define sampleSize         3
#define velocidadeMaxima   35   // km/h
#define taxaAtualizacaoVel 1000 // ms
#define circunfRoda        0.47  // m

#define MPS_to_KMPH 3.6  // metros por segundo p/ quilometros por hora
#define MS_to_S     1000 // milisegundos p/ segundos

const unsigned long maximoSample = MS_to_S * circunfRoda/(pulsosPorVolta) * MPS_to_KMPH; // ?

unsigned long lastPulseTime; // ms
unsigned long currentMillis; // ms
unsigned long timeContador;  // ms
unsigned long pulseTime;     // ms
float speed;                 // km/h

unsigned long pulseTimes[sampleSize];
int lastSpeed;
int pulseIndex;

/**
 * @brief Armazena o valor de periodo da onda no vetor usado para o calculo de velocidade
 * 
 */
void calc();

void setup() {
  Serial.begin(115200);

  pinMode(pinSensorHall, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
  
  for (int i = 0; i < sampleSize; i++) { pulseTimes[i] = 0; }

  lastPulseTime = 0;
  currentMillis = 0;
  timeContador = 0;
  pulseTime = 0;
  speed = 0.0;
  lastSpeed = 0;
  pulseIndex = 0;
}

void loop() {
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
    if(speed >= 1.9*lastSpeed && speed != 0){
    speed = lastSpeed;
    } else if(speed <= 0.4*lastSpeed && speed != 0){
    speed = lastSpeed;
    } else {
      speed = circunfRoda/(pulsosPorVolta*averagePulseTime) * MS_to_S * MPS_to_KMPH;
    }  
    lastSpeed = speed;
    attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
    Serial.print("Speed:");
    Serial.println(speed);
  }
}

void calc() {
  currentMillis = millis();
  pulseTime = currentMillis - lastPulseTime;
  timeContador = millis();
  lastPulseTime = currentMillis;
  pulseTimes[pulseIndex] = pulseTime;
  pulseIndex = (pulseIndex + 1) % sampleSize; 
  lastPulseTime = pulseTime;
}
