#include "Arduino.h"

#define pinSensorHall      DD2

#define velZERO            150  // Valores para teste
#define velMin             400  // Valores para teste
#define velMax             700  // Valores para teste
#define taxaAtualizacaoVel 1000 // ms
#define circunfRoda        0.47 // m
#define pulsosPorVolta     5
#define sampleSize         3    // Numero de amostras para calcular velocidade

#define MPS_to_KMPH 3.6  // metros por segundo p/ quilometro por hora
#define MS_to_S     1000 // milisegundos p/ segundos

unsigned long timerCalcVel;         // ms
unsigned long lastTimerCalcVel;     // ms
unsigned long pulseInterval;     // ms
unsigned long lastPulseInterval; // ms
float velocidade;                // km/h     

unsigned long pulseTimes[sampleSize];
int pulseIndex;

/** 
 * @brief Armazena o valor de periodo da onda no vetor usado para o calculo de velocidade
 * 
 */
void calc();

/**
 * @brief Atualiza a velocidade com base nos pulsos do Sensor Hall 
 * 
 * @details DEFINIR
 * 
 * @param velocidadeAtual velocidade antes da chamada da funcao.
 * 
 * @return velocidade (m/s)
 */
void calculaVelocidade(float &veloc) {
  timerCalcVel = millis() - lastTimerCalcVel;

  if(timerCalcVel >= taxaAtualizacaoVel) {
    timerCalcVel = millis();

    detachInterrupt(digitalPinToInterrupt(pinSensorHall));
//    unsigned long averagePulseInterval = 0;  // ms
//    for (int i = 0; i < sampleSize; i++) {
//      averagePulseInterval += pulseTimes[i];
//    }
//    averagePulseInterval /= sampleSize;
    veloc = circunfRoda/(pulsosPorVolta*pulseInterval) * MS_to_S * MPS_to_KMPH;
    attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
    Serial.print("veloc:");
    Serial.println(veloc);
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(pinSensorHall, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
  
  timerCalcVel = 0;
  lastTimerCalcVel = 0;
  pulseInterval = 0;
  lastPulseInterval = 0;
  velocidade = 0.0;
  for (int i = 0; i < sampleSize; i++) { pulseTimes[i] = 0; }
  pulseIndex = 0;
}

void loop() {
    calculaVelocidade(velocidade);
}

void calc() {
  pulseInterval = millis() - lastPulseInterval;
  lastPulseInterval = millis();
//  pulseTimes[pulseIndex] = timerCalcVel;
//  pulseIndex = (pulseIndex + 1) % sampleSize; 
}
